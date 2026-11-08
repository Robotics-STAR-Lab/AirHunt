#!/usr/bin/env python3
"""
Author: ning-zelin && zl.ning@qq.com
Date: 2024-12-04 18:33:42
LastEditTime: 2025-02-12 12:17:37
Description:

Copyright (c) 2025 by ning-zelin, All Rights Reserved.
"""
from concurrent.futures import thread
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import airsim
from airsim import MultirotorClient, ImageRequest, ImageType, Pose
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)  # 确保能导入同目录的模块
from frames import quat_enu_to_ned, rotate_quat_z


def setodom_cb(msg):

    global last_render_time
    global odom_pose
    global client

    px = msg.pose.pose.position.y
    py = msg.pose.pose.position.x
    pz = -msg.pose.pose.position.z

    ox_ned = msg.pose.pose.orientation.x
    oy_ned = msg.pose.pose.orientation.y
    oz_ned = msg.pose.pose.orientation.z
    ow_ned = msg.pose.pose.orientation.w
    [ow, ox, oy, oz] = quat_enu_to_ned([ow_ned, ox_ned, oy_ned, oz_ned])
    [ow, ox, oy, oz] = rotate_quat_z([ow, ox, oy, oz], np.pi / 2)

    pose = Pose()
    pose.position = airsim.Vector3r(px, py, pz)
    pose.orientation = airsim.Quaternionr(ox, oy, oz, ow)

    odom_pose = pose
    # if time.time() - last_render_time > 0.1:
    #     last_render_time = time.time()
    # The msgpack/Tornado client is not thread-safe. The main thread applies
    # this latest pose and renders; ROS subscriber threads only update it.


def render(client):
    s = time.time()
    responses = client.simGetImages(
        [
            ImageRequest("front_camera", ImageType.Scene, False, False),
            ImageRequest("front_camera_depth", ImageType.DepthPlanar, True, False),
            ImageRequest("left_camera", ImageType.Scene, False, False),
            ImageRequest("left_camera_depth", ImageType.DepthPlanar, True, False),
            ImageRequest("right_camera", ImageType.Scene, False, False),
            ImageRequest("right_camera_depth", ImageType.DepthPlanar, True, False),

            ImageRequest("front_camera_depth_type2", ImageType.DepthPerspective, True, False),
        ]
    )

    # 定义相机组
    camera_groups = [
        {"name": "front", "rgb_idx": 0, "depth_idx": 1, "publishers": front_pubs},
        {"name": "left", "rgb_idx": 2, "depth_idx": 3, "publishers": left_pubs},
        {"name": "right", "rgb_idx": 4, "depth_idx": 5, "publishers": right_pubs},
    ]

    # 处理第三种深度图（DepthPerspective）
    depth_perspective_response = responses[6]

    try:
        current_time = rospy.Time.now()

        # 处理每个相机组
        for group in camera_groups:
            rgb_response = responses[group["rgb_idx"]]
            depth_response = responses[group["depth_idx"]]

            rospy.logdebug(f"{group['name']} - RGB: {rgb_response.width}x{rgb_response.height}, Depth: {depth_response.width}x{depth_response.height}")

            # 处理RGB图像
            img1d = np.frombuffer(rgb_response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(rgb_response.height, rgb_response.width, 3)
            img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)

            # 处理深度图像
            img1d = np.array(depth_response.image_data_float, dtype=np.float32)
            img_depth = img1d.reshape(depth_response.height, depth_response.width, 1) * depth_scale

            # 将超过30m的深度值设为无穷大，适配map_ros.cpp建图逻辑
            img_depth[img_depth > 30.0] = np.inf

            # 转换为ROS消息
            rgb_msg = bridge.cv2_to_imgmsg(img_rgb, "rgb8")
            depth_msg = bridge.cv2_to_imgmsg(img_depth.astype(np.float32), "32FC1")

            # 设置消息头
            rgb_msg.header.stamp = current_time
            rgb_msg.header.frame_id = f"{group['name']}_camera"
            depth_msg.header.stamp = current_time
            depth_msg.header.frame_id = f"{group['name']}_camera"

            # 发布RGB和深度图像
            group["publishers"]["rgb"].publish(rgb_msg)
            group["publishers"]["depth"].publish(depth_msg)

            # 发布相机位姿
            sensor_pose = PoseStamped()
            sensor_pose.header.frame_id = "world"
            sensor_pose.header.stamp = current_time
            sensor_pose.pose.position.x = rgb_response.camera_position.y_val
            sensor_pose.pose.position.y = rgb_response.camera_position.x_val
            sensor_pose.pose.position.z = -rgb_response.camera_position.z_val

            current_quat = [
                rgb_response.camera_orientation.x_val,
                rgb_response.camera_orientation.y_val,
                rgb_response.camera_orientation.z_val,
                rgb_response.camera_orientation.w_val,
            ]
            current_rot_matrix = R.from_quat(current_quat).as_matrix()
            rr = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

            current_rot_matrix = rr @ current_rot_matrix
            euler_angles = R.from_matrix(current_rot_matrix).as_euler("xyz", degrees=False)

            euler_angles[1] = -euler_angles[1]
            euler_angles[2] = -euler_angles[2]
            new_rot_matrix = R.from_euler("xyz", euler_angles, degrees=False).as_matrix()
            new_quat = R.from_matrix(new_rot_matrix).as_quat()
            sensor_pose.pose.orientation.w = new_quat[3]
            sensor_pose.pose.orientation.x = new_quat[0]
            sensor_pose.pose.orientation.y = new_quat[1]
            sensor_pose.pose.orientation.z = new_quat[2]

            group["publishers"]["pose"].publish(sensor_pose)

        # 处理前置相机的第二种深度图（DepthPerspective）
        if depth_perspective_response:
            imgld = np.array(depth_perspective_response.image_data_float, dtype=np.float32)
            img_depth_perspective = imgld.reshape(depth_perspective_response.height, depth_perspective_response.width, 1) * depth_scale

            depth_perspective_msg = bridge.cv2_to_imgmsg(img_depth_perspective.astype(np.float32), "32FC1")
            depth_perspective_msg.header.stamp = current_time
            depth_perspective_msg.header.frame_id = "front_camera"

            front_pubs["depth_perspective"].publish(depth_perspective_msg)

        e = time.time()
        rospy.logdebug(f"render time cost: {(e - s) * 1000} ms")
    except Exception as e:
        rospy.logerr(f"Error in render function: {e}")



if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("airsim_bridge", anonymous=True)
    # Environment-specific calibration; other environments retain unit scale.
    depth_scale = float(rospy.get_param("~depth_scale", 1.0))

    bridge = CvBridge()

    # 为每个相机组创建发布者
    front_pubs = {
        "rgb": rospy.Publisher("/camera/front/rgb", Image, queue_size=1),
        "depth": rospy.Publisher("/camera/front/depth", Image, queue_size=1),
        "depth_perspective": rospy.Publisher("/camera/front/depth_perspective", Image, queue_size=1),
        "pose": rospy.Publisher("/camera/front/pose", PoseStamped, queue_size=1)
    }

    left_pubs = {
        "rgb": rospy.Publisher("/camera/left/rgb", Image, queue_size=1),
        "depth": rospy.Publisher("/camera/left/depth", Image, queue_size=1),
        "pose": rospy.Publisher("/camera/left/pose", PoseStamped, queue_size=1)
    }

    right_pubs = {
        "rgb": rospy.Publisher("/camera/right/rgb", Image, queue_size=1),
        "depth": rospy.Publisher("/camera/right/depth", Image, queue_size=1),
        "pose": rospy.Publisher("/camera/right/pose", PoseStamped, queue_size=1)
    }

    # 连接到AirSim
    client = MultirotorClient()
    client.confirmConnection()
    print("Connected to AirSim!")

    # 设置无人机的初始位置和朝向
    # initial_pose = airsim.Pose(
    #     position_val=airsim.Vector3r(69.0, -127.0, -1.0),  # 位置 (X, Y, Z)
    #     # position_val=airsim.Vector3r(0.0, 0.0, -1.0),  # 位置 (X, Y, Z)
    #     orientation_val=airsim.Quaternionr(0, 0, 0, 1)  # 朝向 (四元数)
    # )
    # # 应用初始位置
    # client.simSetVehiclePose(initial_pose, ignore_collision=True)

    last_render_time = time.time()

    odom_pose = None
    # 获取场景中的所有对象
    # scene_objects = client.simListSceneObjects()

    # 自定义颜色（RGB 格式）
    car_color = (255, 0, 0)  # 蓝色
    road_color = (0, 255, 0)  # 红色
    default_color = (0, 0, 0)  # 白色

    # # 遍历对象，设置颜色
    # for obj in scene_objects:
    #     if "car" in obj.lower():  # 如果名称中包含 "car"
    #         client.simSetSegmentationObjectID(obj, 1)  # 设置分割 ID 为 1
    #         # print(f"Set {obj} to color {car_color} (Car)")
    #     elif "road" in obj.lower():  # 如果名称中包含 "road"
    #         client.simSetSegmentationObjectID(obj, 2)  # 设置分割 ID 为 2
    #         # print(f"Set {obj} to color {road_color} (Road)")
    #     else:  # 其他对象
    #         client.simSetSegmentationObjectID(obj, 0)  # 设置分割 ID 为 0
            # print(f"Set {obj} to default color {default_color} (Other)")

    # print("Segmentation colors set successfully.")
    rospy.Subscriber("/simulator/odom", Odometry, setodom_cb, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose = odom_pose
        if pose is not None:
            client.simSetVehiclePose(pose, True)
            render(client)
        rate.sleep()
