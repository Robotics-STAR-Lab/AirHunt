import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from scipy.spatial.transform import Rotation as R
import rospy
import airsim
def publish_world_lidar(client, lidar_data, lidar_pub):
    """Convert lidar data to world frame and publish"""
    if len(lidar_data.point_cloud) < 3:
        return

    # Get current vehicle pose
    vehicle_pose = client.simGetVehiclePose()

    # Convert point cloud to numpy array
    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)

    # Create rotation matrix from vehicle quaternion
    rotation = R.from_quat([
        vehicle_pose.orientation.x_val,
        vehicle_pose.orientation.y_val,
        vehicle_pose.orientation.z_val,
        vehicle_pose.orientation.w_val
    ]).as_matrix()

    # Transform points to world frame
    points_world = (rotation @ points.T).T + np.array([
        vehicle_pose.position.x_val,
        vehicle_pose.position.y_val,
        vehicle_pose.position.z_val
    ])

    # Create PointCloud2 message
    msg = PointCloud2()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"

    # Pack points into message
    msg.height = 1
    msg.width = points_world.shape[0]
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * points_world.shape[0]
    msg.is_dense = True
    msg.data = points_world.astype(np.float32).tobytes()

    # Publish message
    lidar_pub.publish(msg)
def rotate_quat_z(q, angle):
    """
    Rotate quaternion around Z axis by given angle
    Args:
        q: original quaternion [w,x,y,z]
        angle: rotation angle in radians
    Returns:
        rotated quaternion [w,x,y,z]
    """
    # Create rotation quaternion for Z axis
    qz = np.array([
        np.cos(angle/2),
        0,
        0,
        np.sin(angle/2)
    ])

    # Multiply quaternions
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = qz

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    return np.array([w, x, y, z])
def quat_enu_to_ned(q_enu):
    """
    Convert quaternion from ENU (East-North-Up) to NED (North-East-Down) frame
    Args:
        q_enu: quaternion in ENU frame [w,x,y,z]
    Returns:
        q_ned: quaternion in NED frame [w,x,y,z]
    """
    # Rotation matrix for converting from ENU to NED
    R = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])

    # Convert quaternion to rotation matrix
    w, x, y, z = q_enu
    R_enu = np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y],
        [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
        [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y]
    ])

    # Transform rotation matrix
    R_ned = R @ R_enu @ R.T

    # Convert back to quaternion
    tr = np.trace(R_ned)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R_ned[2,1] - R_ned[1,2]) / S
        y = (R_ned[0,2] - R_ned[2,0]) / S
        z = (R_ned[1,0] - R_ned[0,1]) / S
    else:
        if R_ned[0,0] > R_ned[1,1] and R_ned[0,0] > R_ned[2,2]:
            S = np.sqrt(1.0 + R_ned[0,0] - R_ned[1,1] - R_ned[2,2]) * 2
            w = (R_ned[2,1] - R_ned[1,2]) / S
            x = 0.25 * S
            y = (R_ned[0,1] + R_ned[1,0]) / S
            z = (R_ned[0,2] + R_ned[2,0]) / S
        elif R_ned[1,1] > R_ned[2,2]:
            S = np.sqrt(1.0 + R_ned[1,1] - R_ned[0,0] - R_ned[2,2]) * 2
            w = (R_ned[0,2] - R_ned[2,0]) / S
            x = (R_ned[0,1] + R_ned[1,0]) / S
            y = 0.25 * S
            z = (R_ned[1,2] + R_ned[2,1]) / S
        else:
            S = np.sqrt(1.0 + R_ned[2,2] - R_ned[0,0] - R_ned[1,1]) * 2
            w = (R_ned[1,0] - R_ned[0,1]) / S
            x = (R_ned[0,2] + R_ned[2,0]) / S
            y = (R_ned[1,2] + R_ned[2,1]) / S
            z = 0.25 * S

    return np.array([w, x, y, z])
