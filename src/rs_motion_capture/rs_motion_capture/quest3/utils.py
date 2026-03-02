from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R


def pose_to_matrix(pose: Pose):
    T = np.eye(4)
    quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    T[:3, :3] = R.from_quat(quat).as_matrix()
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    """将 4x4 矩阵转为 geometry_msgs/Pose"""
    r = R.from_matrix(T[:3, :3])
    quat = r.as_quat()  # xyzw
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = T[:3, 3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    return pose


def left_hand_to_right_hand_axis(pose: Pose):
    '''
    The pose from Quest3 Unity is left hand axis,
    need to convert to right hand axis.
    https://robosense.feishu.cn/wiki/LTbawTIQuiMN6akF3VGcJdhHnkf
    '''
    pose.position.z = -pose.position.z
    pose.orientation.x = -pose.orientation.x
    pose.orientation.y = -pose.orientation.y
    return pose


def opt_inv_pose(pose_matrix: np.ndarray):
    pose_matrix_inv = np.eye(4)
    pose_matrix_inv[:3, :3] = pose_matrix[:3, :3].T
    pose_matrix_inv[:3, 3] = -pose_matrix_inv[:3, :3] @ pose_matrix[:3, 3]
    return pose_matrix_inv
