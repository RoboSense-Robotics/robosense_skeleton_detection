#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
import transforms3d as tfs
from vr_control_msgs.msg import Quest3Input
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import JointState as stdJointState
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener
from rclpy.executors import MultiThreadedExecutor

# from rs_motion_capture.quest3.controller import Controller
# from rs_motion_capture.quest3.hand_tracking import HandTracking
# from rs_motion_capture.quest3.utils import pose_to_matrix, matrix_to_pose, left_hand_to_right_hand_axis
from quest3.controller import Controller
from quest3.hand_tracking import HandTracking
from quest3.utils import pose_to_matrix, matrix_to_pose, left_hand_to_right_hand_axis, opt_inv_pose
from dex_retargeting.retargeting_config import RetargetingConfig
from scipy.spatial.transform import Rotation
from interface.msg import JointState
# try:
# except:
#     from scipy.spatial import Rotation
from enum import Enum
import numpy as np
import threading
import yaml
from typing import List
from copy import deepcopy


class ARM_MODE(Enum):
    BOTH = 0,
    LEFT = 1,
    RIGHT = 2,


class CONTROL_MODE(Enum):
    CONTROLLER = 0,
    HAND_TRACKING = 1,


def normalize_angle_rad(diff):
    return (diff + np.pi) % (2 * np.pi) - np.pi


def trans_pose_2_local(target_pose, poses: List[Pose]):
    target_pose_matrix = pose_to_matrix(target_pose)
    # target_pose_matrix_inv = np.linalg.inv(target_pose_matrix)
    target_pose_matrix_inv = opt_inv_pose(target_pose_matrix)

    new_poses = []
    for pose in poses:
        pose_matrix = pose_to_matrix(pose)

        pose_matrix = target_pose_matrix_inv @ pose_matrix

        new_poses.append(matrix_to_pose(pose_matrix))

    return new_poses

def create_local_rotation_z(degrees):
    rad = np.radians(degrees/2)
    w = np.cos(rad)
    z = np.sin(rad)
    return np.array([0.,0.,z,w])


class Quest3TeleopNode(Node):
    def __init__(self):
        super().__init__('quest3_teleop_server')
        self.quest3_input_sub = self.create_subscription(
            Quest3Input,
            '/vr_control_msgs/quest3_input',  # 订阅的话题名，改成你的
            self.input_process,
            10
        )

        # self.button_cmd = self.create_subscription(
        #     String,
        #     "/auxiliary_button_cmd",
        #     self.button_callback,
        #     10
        # )
        self.stop_send_pose = True
        self.declare_parameter("arm_mode", "both")  # both left right
        # controller hand_tracking
        self.declare_parameter("control_mode", "controller")
        self.declare_parameter("hand_type", "rs_hand") # inspire_hand rs_hand

        self.declare_parameter(
            "retarget_urdf_dir", "/teleop_ws/src/rs_motion_capture/config/config_retarget/models")
        self.declare_parameter(
            "retarget_config", "/teleop_ws/src/rs_motion_capture/config/config_retarget/configs/inspire_hand.yml")

        self.arm_mode = ARM_MODE[str(
            self.get_parameter("arm_mode").value).upper()]
        self.control_mode = CONTROL_MODE[str(self.get_parameter(
            "control_mode").value).upper()]
        self.hand_type = self.get_parameter("hand_type").value
        self.get_logger().info(f"arm_mode: {self.arm_mode.name}, control_mode: {self.control_mode.name} hand_type: {self.hand_type}")

        self.retarget_urdf_dir = self.get_parameter("retarget_urdf_dir").value
        RetargetingConfig.set_default_urdf_dir(self.retarget_urdf_dir)

        self.retarget_config = self.get_parameter("retarget_config").value

        with open(self.retarget_config, "r") as f:
            self.retarget_cfg = yaml.safe_load(f)

        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=3600*5))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.smooth_head_position = None
        self.smooth_head_yaw = None
        self.square_ratio = 0.15
        self.yaw_ratio = 10 / 180 * 3.1415926

        self.left_controller: None | Controller = None
        self.right_controller: None | Controller = None

        self.left_hand_tracking: None | HandTracking = None
        self.right_hand_tracking: None | HandTracking = None

        self.cur_left_control: None | Controller | HandTracking = None
        self.cur_right_control: None | Controller | HandTracking = None

        self.tf_lock = threading.Lock()
        self.tf_cv = threading.Condition(self.tf_lock)
        self.tf_ready = False
        self.read_tf_thread = threading.Thread(
            target=self.wait_for_tf_process, daemon=True
        )
        self.read_tf_thread.start()

        self.pub_left_control = self.create_publisher(
            PoseStamped, '/target_pose/left_arm_pose_3d', 10)
        self.pub_right_control = self.create_publisher(
            PoseStamped, '/target_pose/right_arm_pose_3d', 10)

        self.left_joint_state_publisher_inspire = self.create_publisher(
            stdJointState, '/target_pose/left_hand_position', 10
        )
        self.right_joint_state_publisher_inspire = self.create_publisher(
            stdJointState, '/target_pose/right_hand_position', 10
        )
        self.all_inspire_hand_register = self.create_publisher(
            stdJointState, '/smart_hand/hand_register', 10
        )
        self.hand_joints = []

        self.left_joint_state_publisher_rs_hand = self.create_publisher(
            JointState, '/left_hand/position_control', 10
        )

        self.right_joint_state_publisher_rs_hand = self.create_publisher(
            JointState, '/right_hand/position_control', 10
        )

        self.left_rot_degree = -120.0
        self.right_rot_degree = 120.0

        self.left_rot_matrix = Rotation.from_quat(create_local_rotation_z(self.left_rot_degree)).as_matrix()
        self.right_rot_matrix = Rotation.from_quat(create_local_rotation_z(self.right_rot_degree)).as_matrix()



        # debug
        self.pub_rel_head_pose = self.create_publisher(
            PoseStamped, '/head_pose/rel_pose', 10)
        self.pub_now_head_pose = self.create_publisher(
            PoseStamped, '/head_pose/now_pose', 10)


    # def button_callback(self, msg: String):
    #     self.get_logger().info(f"receive {msg.data}")
    #     if msg.data == "start/stop":
    #         self.stop_send_pose = not self.stop_send_pose


    def get_transform(self, source_frame: str, target_frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            # time_t = rclpy.time.Time(seconds=1754567636, nanoseconds=789735596)
            # time_t = rclpy.time.Time(seconds=1754567635, nanoseconds=610995476)
            # time_t = self.get_clock().now()
            # transform = self.tf_buffer.lookup_transform(
            #     target_frame, source_frame, time_t)
            translation = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            quat_rotation = [
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
            ]
            T = tfs.affines.compose(translation, tfs.quaternions.quat2mat(
                quat_rotation), [1, 1, 1])  # (4,4)
            return T
        except Exception as e:
            print(f"TF lookup failed: {str(e)}")
            return None

    def wait_for_tf_process(self):
        flag_left = False
        flag_right = False
        if self.arm_mode == ARM_MODE.BOTH or self.arm_mode == ARM_MODE.LEFT:
            while rclpy.ok():
                T_left_to_left_arm = self.get_transform(
                    "quest3",
                    "left_arm_dummy_base_link"
                )
                # T_left_to_left_arm = np.eye(4)
                if T_left_to_left_arm is None:
                    time.sleep(1.0)
                    continue
                else:
                    flag_left = True
                    self.get_logger().info("read quest3 ---> left_arm_dummy_base_link done")
                    self.get_logger().info(f"{self.control_mode == CONTROL_MODE.CONTROLLER}")
                    if self.control_mode == CONTROL_MODE.CONTROLLER:
                        self.left_controller = Controller(
                            frame="left", quest3_to_target=T_left_to_left_arm)
                    elif self.control_mode == CONTROL_MODE.HAND_TRACKING:
                        self.left_hand_tracking = HandTracking(
                            retarget_config=self.retarget_cfg,
                            frame="left", quest3_to_target=T_left_to_left_arm
                        )
                    else:
                        raise f"{self.control_mode.name} does not be supported, only support {[t for t in CONTROL_MODE._member_names_]}"
                    break
        if self.arm_mode == ARM_MODE.BOTH or self.arm_mode == ARM_MODE.RIGHT:
            while rclpy.ok():
                T_right_to_right_arm = self.get_transform(
                    'quest3',
                    'right_arm_dummy_base_link',
                )
                # T_right_to_right_arm = np.eye(4)
                if T_right_to_right_arm is None:
                    # 防止cpu过载
                    # rclpy.spin_once(self, timeout_sec=1.0)
                    time.sleep(1.0)
                    continue
                else:
                    flag_right = True
                    self.get_logger().info("read quest3 ---> right_arm_dummy_base_link done")
                    if self.control_mode == CONTROL_MODE.CONTROLLER:
                        self.right_controller = Controller(
                            frame='right', quest3_to_target=T_right_to_right_arm)
                    elif self.control_mode == CONTROL_MODE.HAND_TRACKING:
                        self.right_hand_tracking = HandTracking(
                            retarget_config=self.retarget_cfg,
                            frame='right', quest3_to_target=T_right_to_right_arm)
                    else:
                        raise f"{self.control_mode.name} does not be supported, only support {[t for t in CONTROL_MODE._member_names_]}"
                    break
        if flag_left and flag_right:
            self.tf_ready = True
            # with self.tf_cv:
            #     self.tf_cv.notify(1)

    def cal_cross_pose(self, pose_matrix):
        z_axis_xy = pose_matrix[[0, 2], 2]
        z_axis_len = np.linalg.norm(z_axis_xy)

        if z_axis_len < 0.3:
            return None, None
        else:
            position = pose_matrix[:3, 3]
            yaw = np.arctan2(pose_matrix[0, 2], pose_matrix[2, 2])

        return position, yaw

    def get_smooth_pose(self, head_pose):
        pose_matrix = pose_to_matrix(head_pose)
        position, yaw = self.cal_cross_pose(pose_matrix)
        if self.smooth_head_position is None:
            if position is None:
                return False
            self.smooth_head_position = position
            self.smooth_head_yaw = yaw
            return True
        else:
            if position is None:
                self.smooth_head_position = None
                self.smooth_head_yaw = None
                return False
            distance = np.linalg.norm(position - self.smooth_head_position)

            if distance > self.square_ratio:
                dir_vec = (position - self.smooth_head_position) * \
                    ((distance - self.square_ratio) / distance)
                self.smooth_head_position = self.smooth_head_position + dir_vec

            angle_diff = normalize_angle_rad(yaw - self.smooth_head_yaw)

            if angle_diff < -self.yaw_ratio:
                self.smooth_head_yaw += (angle_diff + self.yaw_ratio)
            elif angle_diff > self.yaw_ratio:
                self.smooth_head_yaw += (angle_diff - self.yaw_ratio)

            self.smooth_head_yaw = normalize_angle_rad(self.smooth_head_yaw)

            return True

    def get_rel_pose(self) -> Pose:
        if self.smooth_head_position is None:
            return None
        pose_matrix = np.eye(4)
        pose_matrix[0, 2] = np.sin(self.smooth_head_yaw)
        pose_matrix[1, 2] = 0.
        pose_matrix[2, 2] = np.cos(self.smooth_head_yaw)
        pose_matrix[:3, 1] = np.array([0., 1., 0.])
        pose_matrix[:3, 0] = np.cross(
            np.array([0., 1., 0.]), pose_matrix[:3, 2])
        pose_matrix[:3, 3] = self.smooth_head_position
        pose_matrix[1, 3] -= 1.35
        pose = matrix_to_pose(pose_matrix)
        return pose

    def rotate_hand(self, rot_maxtrix, pose: Pose):
        ori_rot = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()
        new_rot = Rotation.from_matrix(ori_rot @ rot_maxtrix).as_quat()
        new_pose = Pose()
        new_pose.orientation.x = new_rot[0]
        new_pose.orientation.y = new_rot[1]
        new_pose.orientation.z = new_rot[2]
        new_pose.orientation.w = new_rot[3]
        new_pose.position.x = pose.position.x
        new_pose.position.y = pose.position.y
        new_pose.position.z = pose.position.z
        return new_pose

    def publish_left_controller(self, msg, rel_pose):
        left_controller_touch = msg.left_controller.thumbstick_touched
        left_controller_grip = msg.left_controller.grip_pressed
        left_controller_trigger = msg.left_controller.trigger_pressed
        left_trigger_linearity = msg.left_controller.trigger_linearity
        left_grip_linearity = msg.left_controller.grip_linearity
        left_pose = deepcopy(
            left_hand_to_right_hand_axis(msg.left_controller.pose))
        left_pose = trans_pose_2_local(
            rel_pose,
            [
                left_pose,
            ]
        )
        assert self.left_controller is not None
        # left
        left_pose = self.left_controller.trans_pose(left_pose[0])
        pose_array = PoseArray()
        pose_array.header.frame_id = 'left_arm_dummy_base_link'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = [left_pose]
        # self.pub_left_control.publish(pose_array)
        # if left_controller_touch:
        left_pose_stamped = PoseStamped()
        left_pose_stamped.header.frame_id = "left_arm_dummy_base_link"
        left_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        left_pose_stamped.pose = self.rotate_hand(self.left_rot_matrix,left_pose)
        self.pub_left_control.publish(left_pose_stamped)
        # self.get_logger().info(f"hand_type: {self.hand_type}")
        if self.hand_type == "inspire_hand":
        
            left_hand_poses = self.left_controller.get_hand_pose(
                left_controller_grip, left_controller_trigger)
            left_hand_msg = stdJointState()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "left"
            left_hand_msg.header = header
            left_hand_msg.position = left_hand_poses
            # self.get_logger().info(f"left_hand_poses: {left_hand_poses}")
            self.hand_joints.extend(left_hand_poses)
            self.left_joint_state_publisher_inspire.publish(left_hand_msg)
        elif self.hand_type == "rs_hand":
            #self.get_logger().info(f"left_controller_grip: {left_controller_grip}, left_controller_trigger: {left_controller_trigger}")
            left_hand_poses = self.left_controller.get_rs_hand_pose(
                left_controller_grip, left_controller_trigger, left_grip_linearity, left_trigger_linearity)
            left_hand_msg = JointState()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "left"
            left_hand_msg.header = header
            left_hand_msg.name = ["thumb_base_flex", "index_base_flex", "middle_base_flex", "ring_base_flex", "pinky_base_flex", "thumb_base_opposition","thumb_base_abduction", "index_base_abduction"]
            left_hand_msg.position = left_hand_poses
            self.left_joint_state_publisher_rs_hand.publish(left_hand_msg)


    def publish_right_controller(self, msg, rel_pose):
        right_controller_touch = msg.right_controller.thumbstick_touched
        right_controller_grip = msg.right_controller.grip_pressed
        right_controller_trigger = msg.right_controller.trigger_pressed
        right_trigger_linearity = msg.right_controller.trigger_linearity
        right_grip_linearity = msg.right_controller.grip_linearity
        right_pose = deepcopy(
            left_hand_to_right_hand_axis(msg.right_controller.pose))
        right_pose = trans_pose_2_local(
            rel_pose,
            [
                right_pose,
            ]
        )
        assert self.right_controller is not None
        # right
        right_pose = self.right_controller.trans_pose(right_pose[0])
        pose_array = PoseArray()
        pose_array.header.frame_id = 'right_arm_dummy_base_link'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = [right_pose]
        # self.pub_right_control.publish(pose_array)
        # if right_controller_touch:
        right_pose_stamped = PoseStamped()
        right_pose_stamped.header.frame_id = "right_arm_dummy_base_link"
        right_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        right_pose_stamped.pose = self.rotate_hand(self.right_rot_matrix, right_pose)
        self.pub_right_control.publish(right_pose_stamped)

        right_hand_poses = self.right_controller.get_hand_pose(
            right_controller_grip, right_controller_trigger)

        if self.hand_type == "inspire_hand":
            right_hand_msg = stdJointState()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "right"
            right_hand_msg.header = header
            right_hand_msg.position = right_hand_poses
            # self.get_logger().info(f"right_hand_poses: {right_hand_poses}")
            self.hand_joints.extend(right_hand_poses)
            self.right_joint_state_publisher_inspire.publish(right_hand_msg)
        elif self.hand_type == "rs_hand":
            right_hand_poses = self.right_controller.get_rs_hand_pose(
                right_controller_grip, right_controller_trigger, right_grip_linearity, right_trigger_linearity)
            right_hand_msg = JointState()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "right"
            right_hand_msg.header = header
            right_hand_msg.name = ["thumb_base_flex", "index_base_flex", "middle_base_flex", "ring_base_flex", "pinky_base_flex", "thumb_base_opposition","thumb_base_abduction", "index_base_abduction"]
            right_hand_msg.position = right_hand_poses
            self.right_joint_state_publisher_rs_hand.publish(right_hand_msg)

    def process_controller(self, msg, rel_pose):

        if self.arm_mode == ARM_MODE.BOTH or self.arm_mode == ARM_MODE.LEFT:
            if self.left_controller is not None:
                self.publish_left_controller(msg, rel_pose)
        if self.arm_mode == ARM_MODE.BOTH or self.arm_mode == ARM_MODE.RIGHT:
            if self.right_controller is not None:
                self.publish_right_controller(msg, rel_pose)

    def publish_left_hand_tracking(self, msg, rel_pose):
        left_hand = msg.left_hand_skeleton
        left_hand_pose = deepcopy(
            left_hand_to_right_hand_axis(left_hand.wrist_pose))

        left_hand_pose = trans_pose_2_local(
            rel_pose,
            [
                left_hand_pose,
            ]
        )
        # self.get_logger().info(f"left_hand_pose: {left_hand_pose}")
        left_pose = self.left_hand_tracking.trans_pose(left_hand_pose[0])
        left_tracking = False
        # poses, left_qpos = self.left_hand_tracking.get_final_pose(
        #     msg.left_hand_skeleton)

        if self.left_hand_tracking.last_pose is None:
            self.left_hand_tracking.last_pose = deepcopy(left_pose)
            left_tracking = True
        else:
            if np.linalg.norm([
                left_pose.position.x - self.left_hand_tracking.last_pose.position.x,
                left_pose.position.y - self.left_hand_tracking.last_pose.position.y,
                left_pose.position.z - self.left_hand_tracking.last_pose.position.z,
            ]) > 0.6:
                left_tracking = False
            else:
                left_tracking = True
                self.left_hand_tracking.last_pose = deepcopy(left_pose)
        # self.get_logger().info(f"is_control: {self.left_hand_tracking.is_control}, left_tracking: {left_tracking}")
        # if self.left_hand_tracking.is_control and left_tracking:
        if left_tracking:
            left_pose_stamped = PoseStamped()
            left_pose_stamped.header.frame_id = "left_arm_dummy_base_link"
            left_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            left_pose_stamped.pose = self.rotate_hand(self.left_rot_matrix ,left_pose)
            self.pub_left_control.publish(left_pose_stamped)

        # header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        # header.frame_id = "left"
        # hand_msg = JointState()
        # hand_msg.header = header
        # hand_msg.position = [float(p) for p in left_qpos]
        # self.left_joint_state_publisher_inspire.publish(hand_msg)

    def publish_right_hand_tracking(self, msg, rel_pose):
        right_hand = msg.right_hand_skeleton
        right_hand_pose = deepcopy(
            left_hand_to_right_hand_axis(right_hand.wrist_pose))
        self.get_logger().info(f"{right_hand_pose}")
        right_hand_pose = trans_pose_2_local(
            rel_pose,
            [
                right_hand_pose,
            ]
        )
        
        right_pose = self.right_hand_tracking.trans_pose(right_hand_pose[0])
        right_tracking = False
        # poses, right_qpos = self.right_hand_tracking.get_final_pose(
        #     msg.right_hand_skeleton)

        if self.right_hand_tracking.last_pose is None:
            self.right_hand_tracking.last_pose = deepcopy(right_pose)
            right_tracking = True
        else:
            if np.linalg.norm([
                right_pose.position.x - self.right_hand_tracking.last_pose.position.x,
                right_pose.position.y - self.right_hand_tracking.last_pose.position.y,
                right_pose.position.z - self.right_hand_tracking.last_pose.position.z,
            ]) > 0.6:
                right_tracking = False
            else:
                right_tracking = True
                self.right_hand_tracking.last_pose = deepcopy(right_pose)
        # self.get_logger().info(f"is_control: {self.right_hand_tracking.is_control}, right_tracking: {right_tracking}")
        # if self.right_hand_tracking.is_control and right_tracking:
        if right_tracking:
            right_pose_stamped = PoseStamped()
            right_pose_stamped.header.frame_id = "right_arm_dummy_base_link"
            right_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            right_pose_stamped.pose = self.rotate_hand(self.right_rot_matrix,right_pose)
            self.pub_right_control.publish(right_pose_stamped)

        # header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        # header.frame_id = "right"
        # hand_msg = JointState()
        # hand_msg.header = header
        # hand_msg.position = [float(p) for p in right_qpos]
        # self.right_joint_state_publisher_inspire.publish(hand_msg)

    def process_hand_tracking(self, msg, rel_pose):
        if self.arm_mode == ARM_MODE.BOTH or self.arm_mode == ARM_MODE.LEFT:
            
            if self.left_hand_tracking is not None:
                self.publish_left_hand_tracking(msg, rel_pose)
        if self.arm_mode == ARM_MODE.BOTH or self.arm_mode == ARM_MODE.RIGHT:
            # self.get_logger().info(f"right: {self.right_hand_tracking is None}")
            if self.right_hand_tracking is not None:
                self.publish_right_hand_tracking(msg, rel_pose)

    def input_process(self, msg: Quest3Input):
        # self.get_logger().info(self.tf_ready)
        # with self.tf_cv:
        #     self.tf_cv.wait_for(lambda : self.tf_ready)
        # self.get_logger().info(f"self.tf_ready: {self.tf_ready}")
        # if not self.tf_ready:
        #     return
        # self.get_logger().info(msg)
        # # if self.stop_send_pose:
        # #     return
        # self.get_logger().info("head_pose")
        head_pose = left_hand_to_right_hand_axis(msg.head_pose)
        # self.get_logger().info(f"head_pose: {head_pose}")

        if not self.get_smooth_pose(head_pose):
            return

        rel_pose = self.get_rel_pose()
        
        # self.get_logger().info(f"rel_pose: {rel_pose}")

        if rel_pose is None:
            return

        rel_pose_stamped = PoseStamped()
        rel_pose_stamped.header.frame_id = "quest3"
        rel_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        rel_pose_stamped.pose = rel_pose

        now_pose_stamped = PoseStamped()
        now_pose_stamped.header.frame_id = "quest3"
        now_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        now_pose_stamped.pose = head_pose

        self.pub_rel_head_pose.publish(rel_pose_stamped)
        self.pub_now_head_pose.publish(now_pose_stamped)


        if self.control_mode == CONTROL_MODE.CONTROLLER:
            self.process_controller(msg, rel_pose)
            # self.get_logger().info(f"---------------hand_joints: {self.hand_joints}")
            if len(self.hand_joints) <= 0:
                return
            usb_hand_msg = stdJointState()
            usb_hand_msg.header.stamp = self.get_clock().now().to_msg()
            usb_hand_msg.header.frame_id = 'base_link'
            if self.arm_mode == ARM_MODE.BOTH:
                usb_hand_msg.name = [
                    "left_pinky", "left_ring", "left_middle", "left_index", "left_thumb", "left_thumb_lateral",
                    "right_pinky", "right_ring", "right_middle", "right_index", "right_thumb", "right_thumb_lateral"
                ]

                # usb_hand_msg.position = [
                #     self.hand_joints[4],
                #     self.hand_joints[3],
                #     self.hand_joints[2],
                #     self.hand_joints[1],
                #     self.hand_joints[0],
                #     self.hand_joints[5],
                #     self.hand_joints[6+4],
                #     self.hand_joints[6+3],
                #     self.hand_joints[6+2],
                #     self.hand_joints[6+1],
                #     self.hand_joints[6+0],
                #     self.hand_joints[6+5],
                # ]
                usb_hand_msg.position = [
                    self.hand_joints[0],
                    self.hand_joints[1],
                    self.hand_joints[2],
                    self.hand_joints[3],
                    self.hand_joints[4],
                    self.hand_joints[5],
                    self.hand_joints[6+0],
                    self.hand_joints[6+1],
                    self.hand_joints[6+2],
                    self.hand_joints[6+3],
                    self.hand_joints[6+4],
                    self.hand_joints[6+5],
                ]
            elif self.arm_mode == ARM_MODE.LEFT:
                usb_hand_msg.name = [
                    "left_pinky", "left_ring", "left_middle", "left_index", "left_thumb", "left_thumb_lateral",
                ]
                usb_hand_msg.position = [
                    self.hand_joints[0],
                    self.hand_joints[1],
                    self.hand_joints[2],
                    self.hand_joints[3],
                    self.hand_joints[4],
                    self.hand_joints[5],
                ]
            elif self.arm_mode == ARM_MODE.RIGHT:
                usb_hand_msg.name = [
                    "right_pinky", "right_ring", "right_middle", "right_index", "right_thumb", "right_thumb_lateral",
                ]
                usb_hand_msg.position = [
                    self.hand_joints[0],
                    self.hand_joints[1],
                    self.hand_joints[2],
                    self.hand_joints[3],
                    self.hand_joints[4],
                    self.hand_joints[5],
                ]
            # self.get_logger().info(f"publish hand register: {usb_hand_msg.position}")
            self.all_inspire_hand_register.publish(usb_hand_msg)
            self.hand_joints.clear()
        elif self.control_mode == CONTROL_MODE.HAND_TRACKING:
            self.process_hand_tracking(msg, rel_pose)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    node = Quest3TeleopNode()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
