from geometry_msgs.msg import Pose
import numpy as np
from dex_retargeting.retargeting_config import RetargetingConfig
import yaml
# from rs_motion_capture.quest3.utils import pose_to_matrix, matrix_to_pose, left_hand_to_right_hand_axis
from quest3.utils import pose_to_matrix, matrix_to_pose, left_hand_to_right_hand_axis, opt_inv_pose
from typing import List
from vr_control_msgs.msg import HandSkeleton


class HandTracking:
    def __init__(self, retarget_config: yaml.Node,
                 quest3_to_target: np.ndarray = np.eye(4),
                 frame: str = 'left',
                 ):
        self.frame = frame
        self.quest3_to_target = quest3_to_target
        self.unity_hand_to_target_hand = None

        if self.frame == 'left':
            self.unity_hand_to_target_hand = np.array([
                [0.0, 0.0, -1.0],
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ])
            self.retarget = RetargetingConfig.from_dict(
                retarget_config['left']).build()
        elif self.frame == 'right':
            self.unity_hand_to_target_hand = np.array([
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ])
            self.retarget = RetargetingConfig.from_dict(
                retarget_config['right']).build()
        self.last_pose = None
        self.is_control = False

    def trans_pose(self, pose: Pose):
        trans_matrix = pose_to_matrix(pose)
        trans_matrix = self.quest3_to_target @ trans_matrix
        trans_matrix[:3, :3] = trans_matrix[:3, :3] @ self.unity_hand_to_target_hand
        pose = matrix_to_pose(trans_matrix)
        return pose

    def check_control_mode(self, bone_poses_matrix: np.ndarray):
        thumb_tip = bone_poses_matrix[0, :3, 3]
        ring_tip = bone_poses_matrix[3, :3, 3]
        pinky_tip = bone_poses_matrix[4, :3, 3]

        # print(f"thumb_tip---ring_tip: {np.linalg.norm(thumb_tip - ring_tip)}")
        # print(f"thumb_tip---pinky_tip: {np.linalg.norm(thumb_tip - pinky_tip)}")
        if np.linalg.norm(thumb_tip - ring_tip) < 0.01:
            self.is_control = False
        elif np.linalg.norm(thumb_tip - pinky_tip) < 0.01:
            if not self.is_control:
                self.last_pose = None
            self.is_control = True

    def get_final_pose(self, hand_track_msg: HandSkeleton):
        base_pose: Pose = hand_track_msg.wrist_pose
        base_pose_matrix = pose_to_matrix(base_pose)
        # unity world to unity wrist
        # base_pose_matrix_inv = np.linalg.inv(base_pose_matrix)
        base_pose_matrix_inv = opt_inv_pose(base_pose_matrix)

        bone_poses: List[Pose] = hand_track_msg.bone_poses
        bone_poses_matrix = [np.dot(base_pose_matrix_inv, pose_to_matrix(left_hand_to_right_hand_axis(p)))
                             for p in bone_poses]
        if self.frame == 'left':
            for matrix in bone_poses_matrix:
                tmp = matrix[0, 3]
                matrix[0, 3] = -matrix[1, 3]
                matrix[1, 3] = tmp
        else:
            for matrix in bone_poses_matrix:
                tmp = matrix[0, 3]
                matrix[0, 3] = matrix[1, 3]
                matrix[1, 3] = -tmp
        bone_poses_matrix = np.array(bone_poses_matrix)[-6:-1]

        self.check_control_mode(bone_poses_matrix)

        qpos = self.retarget.retarget(bone_poses_matrix[:, :3, 3])[
            self.retarget.optimizer.idx_pin2target]
        #
        # Thumb Pitch: 0 - 0.5
        # Index: 0 - 1.7
        # Thumb Yaw: -0.1, 1.3
        qpos[0] = - (qpos[0] / 0.5) * 1000. + 1000.
        qpos[1:5] = - (qpos[1:5] / 1.7) * 1000. + 1000.
        qpos[5] = - ((qpos[5] + 0.1) / 1.4) * 1000. + 1000.

        qpos = np.clip(qpos, a_min=0., a_max=1000.)

        bone_poses_format = [matrix_to_pose(m) for m in bone_poses_matrix]

        return bone_poses_format, qpos
