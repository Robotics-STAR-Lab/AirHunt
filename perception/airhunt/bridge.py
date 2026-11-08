import numpy as np
import logging
import math
import time
import cv2
import airsim
from scipy.spatial.transform import Rotation as R
from airsim import ImageRequest, ImageType
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import message_filters


def pose_memory_to_normal(pose):
    return np.dot(
        np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]), pose
    )


def pose_normal_to_tsdf(pose):
    return np.dot(
        pose, np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    )


def pose_airsim_to_memory(pose):
    return np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]) @ pose


def euler_to_quaternion(pitch, yaw, roll):
    r = R.from_euler("xyz", [pitch, yaw, roll])
    quat = r.as_quat()
    return quat


class AirsimBridge:

    def __init__(self, env_name, cfg, logging_path, formatter):
        self.cfg = cfg
        self.env_name = env_name
        self._client = airsim.VehicleClient()
        self._client.confirmConnection()
        self.path_id = 1
        logging.info("init airsim bridge")
        root_logger = logging.getLogger()
        for handler in root_logger.handlers[:]:
            root_logger.removeHandler(handler)
        logging.basicConfig(
            level=logging.INFO,
            format="%(message)s",
            handlers=[
                logging.FileHandler(logging_path, mode="w"),
                logging.StreamHandler(),
            ],
        )
        for handler in logging.getLogger().handlers:
            handler.setFormatter(formatter)
        logging.info("ROS planner mode enabled")
        self.path_publisher = rospy.Publisher("/planned_path", Path, queue_size=1)
        self.global_map_publisher = rospy.Publisher(
            "/value_map_global", Path, queue_size=1
        )
        self.local_map_publisher = rospy.Publisher(
            "/value_map_local", Path, queue_size=1
        )
        self.zone_info_publisher = rospy.Publisher(
            "/global_zone_info", String, queue_size=1
        )
        self.task1_pose_publisher = rospy.Publisher(
            "/task1/pose", PoseStamped, queue_size=10
        )
        self.task1_depth_publisher = rospy.Publisher(
            "/task1/depth", Image, queue_size=10
        )
        self.task2_pose_publisher = rospy.Publisher(
            "/task2/pose", PoseStamped, queue_size=10
        )
        self.fsm_state = None
        self.fsm_subscriber = rospy.Subscriber(
            "/fsm_state", String, self.fsm_state_callback
        )
        self.bridge = CvBridge()
        self._data_lock = threading.Lock()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_pose = None
        self.latest_left_rgb = None
        self.latest_left_depth = None
        self.latest_left_pose = None
        self.latest_right_rgb = None
        self.latest_right_depth = None
        self.latest_right_pose = None
        self.camera_data_buffer = []
        self.camera_data_buffer = [[], [], []]
        self.buffer_idx = {"front": 0, "left": 1, "right": 2}
        self.buffer_lock = threading.Lock()
        self.min_time_interval = 0.5
        self.max_buffer_size = 20
        self.front_rgb_sub = message_filters.Subscriber("/camera/front/rgb", Image)
        self.front_depth_sub = message_filters.Subscriber("/camera/front/depth", Image)
        self.front_pose_sub = message_filters.Subscriber(
            "/camera/front/pose", PoseStamped
        )
        self.front_sync = message_filters.TimeSynchronizer(
            [self.front_rgb_sub, self.front_depth_sub, self.front_pose_sub],
            queue_size=10,
        )
        self.front_sync.registerCallback(self.front_synchronized_callback)
        self.left_rgb_sub = message_filters.Subscriber("/camera/left/rgb", Image)
        self.left_depth_sub = message_filters.Subscriber("/camera/left/depth", Image)
        self.left_pose_sub = message_filters.Subscriber(
            "/camera/left/pose", PoseStamped
        )
        self.left_sync = message_filters.TimeSynchronizer(
            [self.left_rgb_sub, self.left_depth_sub, self.left_pose_sub], queue_size=10
        )
        self.left_sync.registerCallback(self.left_synchronized_callback)
        self.right_rgb_sub = message_filters.Subscriber("/camera/right/rgb", Image)
        self.right_depth_sub = message_filters.Subscriber("/camera/right/depth", Image)
        self.right_pose_sub = message_filters.Subscriber(
            "/camera/right/pose", PoseStamped
        )
        self.right_sync = message_filters.TimeSynchronizer(
            [self.right_rgb_sub, self.right_depth_sub, self.right_pose_sub],
            queue_size=10,
        )
        self.right_sync.registerCallback(self.right_synchronized_callback)
        logging.info("Ros node initialized and subscriber set up")
        if not self.cfg.get("ros_planner", False):
            initial_pose = airsim.Pose(
                airsim.Vector3r(0, 0, -2), airsim.Quaternionr(0, 0, 0, 1)
            )
            self._client.simSetVehiclePose(initial_pose, True)
        time.sleep(2)
        logging.info("AirSim drone initialization complete")

    def fsm_state_callback(self, msg):
        self.fsm_state = msg.data

    def front_synchronized_callback(self, rgb_msg, depth_msg, pose_msg):
        with self._data_lock:
            try:
                self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
                self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
                self.latest_pose = pose_msg
                self._add_to_buffer("front", rgb_msg, depth_msg, pose_msg)
            except Exception as e:
                rospy.logerr(f"Front camera synchronization error: {e}")

    def left_synchronized_callback(self, rgb_msg, depth_msg, pose_msg):
        with self._data_lock:
            try:
                self.latest_left_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
                self.latest_left_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
                self.latest_left_pose = pose_msg
                self._add_to_buffer("left", rgb_msg, depth_msg, pose_msg)
            except Exception as e:
                rospy.logerr(f"Left camera synchronization error: {e}")

    def right_synchronized_callback(self, rgb_msg, depth_msg, pose_msg):
        with self._data_lock:
            try:
                self.latest_right_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
                self.latest_right_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
                self.latest_right_pose = pose_msg
                self._add_to_buffer("right", rgb_msg, depth_msg, pose_msg)
            except Exception as e:
                rospy.logerr(f"Right camera synchronization error: {e}")

    def set_drone_pos(self, x, y, z, pitch, yaw, roll):
        x, y, z = (float(x), float(y), float(z))
        pitch, yaw, roll = (float(pitch), float(yaw), float(roll))
        new_pos = airsim.Vector3r(x, y, z)
        new_orientation = airsim.to_quaternion(pitch, roll, yaw)
        target_pose = airsim.Pose(new_pos, new_orientation)
        self._client.simSetVehiclePose(target_pose, True)

    def set_drone_pos_ros(self, x, y, z, pitch, yaw, roll, is_frontier, threshold=0.3):
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        curr_pos, curr_orientation = self.get_pose()
        path = []
        if not is_frontier:
            path.append([x, y, z, yaw])
        else:
            path.append([curr_pos[0], curr_pos[1], curr_pos[2], yaw])
        if path is not None:
            self.publish_path_to_ros(path)
        while True:
            curr_pos, curr_orientation = self.get_pose()
            _, _, curr_yaw = airsim.to_eularian_angles(curr_orientation)
            position_reached = (
                np.linalg.norm(
                    np.array([curr_pos[0] - x, curr_pos[1] - y, curr_pos[2] - z])
                )
                < 0.5
                and abs(curr_yaw - yaw) < 0.2
            )
            fsm_ready = self.current_fsm_state == "WAIT_FOR_GOAL"
            if position_reached and fsm_ready:
                break
            elif not position_reached and fsm_ready:
                fsm_ready = False
                path = [[x, y, z, yaw]]
                self.publish_path_to_ros(path)
                rospy.sleep(0.1)
                continue
            elif position_reached:
                continue

    def publish_path_to_ros(self, path):
        if path is None or len(path) == 0:
            return
        ros_path = Path()
        ros_path.header = Header()
        ros_path.header.stamp = rospy.Time.now()
        ros_path.header.frame_id = "world"
        for i, point in enumerate(path):
            normalized_yaw = math.atan2(math.sin(point[3]), math.cos(point[3]))
            target_x, target_y, target_z, target_yaw = (
                point[1],
                point[0],
                -point[2],
                math.atan2(
                    math.sin(math.pi / 2 - normalized_yaw),
                    math.cos(math.pi / 2 - normalized_yaw),
                ),
            )
            quaternion = euler_to_quaternion(0, 0, target_yaw)
            pose_stamped = PoseStamped()
            pose_stamped.header = ros_path.header
            pose_stamped.pose.position.x = float(target_x)
            pose_stamped.pose.position.y = float(target_y)
            pose_stamped.pose.position.z = float(target_z)
            pose_stamped.pose.orientation.x = float(quaternion[0])
            pose_stamped.pose.orientation.y = float(quaternion[1])
            pose_stamped.pose.orientation.z = float(quaternion[2])
            pose_stamped.pose.orientation.w = float(quaternion[3])
            ros_path.poses.append(pose_stamped)
        self.path_publisher.publish(ros_path)

    def get_camera_data(self):
        s = time.time()
        responses = self._client.simGetImages(
            [
                ImageRequest("front_camera", ImageType.Scene, False, False),
                ImageRequest("front_camera_depth", ImageType.DepthPlanar, True, False),
                ImageRequest("seg_camera", ImageType.Segmentation, False, False),
            ]
        )
        groups = []
        for i in range(0, len(responses), 3):
            rgb_response = responses[i]
            depth_response = responses[i + 1]
            seg_response = responses[i + 2]
            img1d = np.frombuffer(rgb_response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(rgb_response.height, rgb_response.width, 3)
            img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
            img1d = np.array(depth_response.image_data_float, dtype=np.float32)
            img_depth = img1d.reshape(depth_response.height, depth_response.width)
        return (img_rgb, img_depth)

    def get_all_camera_data_from_ros(self, cameras=None):
        if cameras is None:
            cameras = ["front", "left", "right"]
        with self._data_lock:
            result = {}
            if "front" in cameras:
                result["front"] = {
                    "rgb": self.latest_rgb,
                    "depth": self.latest_depth,
                    "pose": self.latest_pose,
                }
            if "left" in cameras:
                result["left"] = {
                    "rgb": self.latest_left_rgb,
                    "depth": self.latest_left_depth,
                    "pose": self.latest_left_pose,
                }
            if "right" in cameras:
                result["right"] = {
                    "rgb": self.latest_right_rgb,
                    "depth": self.latest_right_depth,
                    "pose": self.latest_right_pose,
                }
            return result

    def get_pose(self):
        current_pose = self._client.simGetVehiclePose()
        position = current_pose.position
        orientation = current_pose.orientation
        pos = np.array([position.x_val, position.y_val, position.z_val])
        return (pos, orientation)

    def _add_to_buffer(
        self,
        camera_name,
        rgb_msg,
        depth_msg,
        pose_msg,
        min_time_interval=1,
        max_buffer_size=100,
        min_pos_diff=2,
        min_angle_diff=5.0,
    ):
        ros_timestamp = rgb_msg.header.stamp.to_sec()
        buffer_idx = self.buffer_idx[camera_name]
        with self.buffer_lock:
            if len(self.camera_data_buffer[buffer_idx]) > 0:
                last_timestamp = self.camera_data_buffer[buffer_idx][-1]["timestamp"]
                if ros_timestamp - last_timestamp < self.min_time_interval:
                    return
            try:
                rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
                depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
                data_entry = {
                    "timestamp": ros_timestamp,
                    "ros_timestamp": rgb_msg.header.stamp,
                    "camera_name": camera_name,
                    "rgb": rgb_cv,
                    "depth_cv": depth_cv,
                    "depth_msg": depth_msg,
                    "pose": pose_msg,
                }
                self.camera_data_buffer[buffer_idx].append(data_entry)
                if len(self.camera_data_buffer[buffer_idx]) > self.max_buffer_size:
                    self.camera_data_buffer[buffer_idx].pop(0)
                logging.debug(
                    f"Added {camera_name} camera data to buffer. Buffer size: {len(self.camera_data_buffer)}, ROS timestamp: {ros_timestamp:.3f}"
                )
            except Exception as e:
                rospy.logerr(f"Error adding {camera_name} camera data to buffer: {e}")

    def get_buffered_camera_data(self, buffer_size=None, cameras=None, first=False):
        if cameras is None:
            cameras = ["front", "left", "right"]
        with self.buffer_lock:
            if not first:
                latest_data = []
                cameras = ["front", "left", "right"]
                for camera_name in cameras:
                    buffer_idx = self.buffer_idx[camera_name]
                    if self.camera_data_buffer[buffer_idx]:
                        latest_data.append(self.camera_data_buffer[buffer_idx][-1])
                latest_data.sort(key=lambda x: x["camera_name"])
                logging.info(f"first = false: 返回{len(latest_data)}个相机的最新图像")
                return latest_data
            all_buffer = [
                item for sublist in self.camera_data_buffer for item in sublist
            ]
            filtered_data = [
                entry for entry in all_buffer if entry["camera_name"] in cameras
            ]
            filtered_data.sort(key=lambda x: x["timestamp"])
            if buffer_size is not None and len(filtered_data) > buffer_size:
                total_samples = len(filtered_data)
                step = (total_samples - 1) / (buffer_size - 1) if buffer_size > 1 else 0
                sampled_indices = []
                for i in range(buffer_size):
                    if i == buffer_size - 1:
                        sampled_indices.append(total_samples - 1)
                    else:
                        sampled_indices.append(int(round(i * step)))
                filtered_data = [filtered_data[idx] for idx in sampled_indices]
                logging.info(
                    f"均匀采样: 从{total_samples}个数据中采样{buffer_size}个，索引: {sampled_indices}"
                )
            else:
                logging.info(f"未进行采样: 返回{len(filtered_data)}个数据")
            return filtered_data

    def clear_camera_buffer(self):
        with self.buffer_lock:
            buffer_size_before = sum(
                (len(camera_buffer) for camera_buffer in self.camera_data_buffer)
            )
            self.camera_data_buffer = [[], [], []]
            logging.info(
                f"Cleared camera buffer. Removed {buffer_size_before} entries."
            )

    def get_buffer_status(self):
        with self.buffer_lock:
            if not self.camera_data_buffer:
                return {
                    "size": 0,
                    "oldest_timestamp": None,
                    "newest_timestamp": None,
                    "time_span": 0,
                }
            oldest = math.inf
            newest = -math.inf
            size = 0
            has_data = False
            for camera_buffer in self.camera_data_buffer:
                if camera_buffer:
                    timestamps = [entry["timestamp"] for entry in camera_buffer]
                    oldest = min(min(timestamps), oldest)
                    newest = max(max(timestamps), newest)
                    has_data = True
                size += len(camera_buffer)
            if not has_data:
                return {
                    "size": 0,
                    "oldest_timestamp": None,
                    "newest_timestamp": None,
                    "time_span": 0,
                }
            return {
                "size": size,
                "oldest_timestamp": oldest,
                "newest_timestamp": newest,
                "time_span": newest - oldest,
            }

    def publish_task1_data(self, task1_results):
        if not task1_results:
            return
        current_time = rospy.Time.now()
        for result in task1_results:
            pose_msg = result["pose"]
            score = float(result["score"])
            depth_msg = result["depth"]
            pose_msg.header.frame_id = str(score)
            pose_msg.header.stamp = current_time
            depth_msg.header.frame_id = str(score)
            depth_msg.header.stamp = current_time
            self.task1_pose_publisher.publish(pose_msg)
            self.task1_depth_publisher.publish(depth_msg)
        logging.info(
            f"Published {len(task1_results)} task1 results with scores to ROS topics  ROS time: {current_time.to_sec()}, seq: {pose_msg.header.seq}"
        )

    def publish_task2_data(self, task2_results, objects):
        choice = task2_results["max_point_choice"]
        if choice is None:
            logging.warning("No valid choice found in task2 results")
            return
        obj_centers = [objects[obj_id]["bbox"].center for obj_id in choice.cluster]
        current_time = rospy.Time.now()
        if obj_centers:
            obj_pose = np.mean(obj_centers, axis=0)
        else:
            logging.warning("No object centers found for task2 results")
            return
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = current_time
        pose_msg.pose.position.x = float(obj_pose[0])
        pose_msg.pose.position.y = float(obj_pose[1])
        pose_msg.pose.position.z = float(obj_pose[2])
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.task2_pose_publisher.publish(pose_msg)
        logging.info(
            f"Published task2 results with object pose {obj_pose} to ROS topics - ROS time: {current_time.to_sec()}, seq: {pose_msg.header.seq}"
        )
        return None


def pos_airsim_to_memory(position):
    """NED to memory frame: east, up, negative north."""
    return np.array([position[1], -position[2], -position[0]])
