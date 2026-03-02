import numpy as np
from geometry_msgs.msg import Pose
# from rs_motion_capture.quest3.utils import pose_to_matrix, matrix_to_pose
from quest3.utils import pose_to_matrix, matrix_to_pose
from scipy.spatial.transform import Rotation


class Controller:
    def __init__(self, quest3_to_target: np.ndarray = np.eye(4),
                 frame: str = 'left'):
        self.frame = frame
        self.quest3_to_target = quest3_to_target

        # unity hand's axis z is point to body,
        # need to translate to outside target
        self.vr_controller_to_target_hand = None

        if self.frame == 'left':
            self.vr_controller_to_target_hand = np.array([
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0],  # invert z
            ])
        elif self.frame == 'right':
            self.vr_controller_to_target_hand = np.array([
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],  # invert z
            ])

    def set_calib_target_hand(self, quat):
        self.vr_controller_to_target_hand = Rotation.from_quat(quat).as_matrix()

    def trans_pose(self, pose: Pose):
        trans_matrix = pose_to_matrix(pose)
        trans_matrix = self.quest3_to_target @ trans_matrix
        trans_matrix[:3, :3] = trans_matrix[:3, :3] @ self.vr_controller_to_target_hand
        pose = matrix_to_pose(trans_matrix)
        return pose

    def trans_pose2(self, pose: Pose, new_axis_local_translate=None):
        trans_matrix = pose_to_matrix(pose)
        trans_matrix = self.quest3_to_target @ trans_matrix

        local_transform = np.eye(4)
        local_transform[:3,:3] = self.vr_controller_to_target_hand
        if new_axis_local_translate is not None:
            local_transform[:3, 3] = np.dot(self.vr_controller_to_target_hand, new_axis_local_translate)
        trans_matrix = trans_matrix @ local_transform
        pose = matrix_to_pose(trans_matrix)
        return pose

    def get_hand_pose(self, grip: bool, trigger: bool):
        poses = []
        # if trigger:
        #     poses.extend([500., 500., 500.])
        # else:
        #     poses.extend([1000., 1000., 1000.])

        # if grip:
        #     poses.extend([0., 0., 0.])
        # else:
        #     poses.extend([1000., 1000., 0.])
        if trigger and grip:
            poses.extend([500., 500., 500., 0., 0., 0.])
        elif not trigger and grip:
            poses.extend([1000., 1000., 1000., 0., 0., 0.])
        elif trigger and not grip:
            poses.extend([500., 500., 500., 1000., 1000., 0.])
        else:
            poses.extend([1000., 1000., 1000., 1000., 1000., 0.])

        return poses

    def get_rs_hand_pose(self, grip: bool, trigger: bool, grip_linearity: float, trigger_linearity: float):
        # min_pose = [0,0,0,0,0,0,-300,-300]                    # open
        min_pose = [0] * 8
        min_pose[-2] = -300
        # max_pose = [900,900,900,900,900,1500,300,300]         # close
        # target_pose = [210, 430, 380, 900, 900, 965, -21, 2]  # 对指数值
        # trigger_index = [0,1,2,5,6,7]
        # grip_index = [3,4]

        target_pose = [56, 641, 635, 882, 760, 1254, 251, -100]  # 对指数值
        trigger_index = [0,1,2,3,4]
        grip_index = [5,6,7]
        poses = [0] * 8

        # if trigger:
        for idx in trigger_index:
            poses[idx] = int(trigger_linearity * (target_pose[idx] - min_pose[idx]) + min_pose[idx])

        # if grip:
        for idx in grip_index:
            poses[idx] = int(grip_linearity * (target_pose[idx] - min_pose[idx]) + min_pose[idx])
        
        # print("orin grip: {} trigger:{} grip_linearity:{} trigger_linearity:{}".format(grip, trigger, grip_linearity, trigger_linearity))
        # print("poses: {}".format(poses))
        # print('')
        return poses
