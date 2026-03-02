#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
import rclpy
from rclpy.node import Node
import transforms3d as tfs

from vr_control_msgs.msg import Quest3Input
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener
from rclpy.executors import MultiThreadedExecutor

from quest3.utils import pose_to_matrix, matrix_to_pose, left_hand_to_right_hand_axis

from scipy.spatial.transform import Rotation

from enum import Enum
import numpy as np
import threading
import yaml
import time
from typing import List
from copy import deepcopy


def normalize_angle_rad(diff):
    return (diff + np.pi) % (2 * np.pi) - np.pi


def trans_pose_2_local(target_pose, poses: List[Pose]):
    target_pose_matrix = pose_to_matrix(target_pose)
    target_pose_matrix_inv = np.linalg.inv(target_pose_matrix)

    new_poses = []
    for pose in poses:
        pose_matrix = pose_to_matrix(pose)

        pose_matrix = target_pose_matrix_inv @ pose_matrix

        new_poses.append(matrix_to_pose(pose_matrix))

    return new_poses


class Quest3GloveCalibNode(Node):
    def __init__(self):
        super().__init__('quest3_glove_calib')
        self.quest3_input_sub = self.create_subscription(
            Quest3Input,
            '/vr_control_msgs/quest3_input',  # 订阅的话题名，改成你的
            self.input_process,
            10
        )

        self.declare_parameter("mode", "calib")
        self.mode = self.get_parameter("mode").value

        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=3600*5))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_lock = threading.Lock()
        self.tf_cv = threading.Condition(self.tf_lock)
        self.tf_ready = False
        self.read_tf_thread = threading.Thread(
            target=self.wait_for_tf_process, daemon=True
        )
        self.read_tf_thread.start()

        self.quest3_to_map = None
        self.map_to_left_arm_dummy_base_link = None
        self.map_to_right_arm_dummy_base_link = None

        self.smooth_head_position = None
        self.smooth_head_yaw = None
        self.square_ratio = 0.15
        self.yaw_ratio = 10 / 180 * 3.1415926

        # unity --> --quest3 --> rel_pose --> map
        self.left_controller_poses = []
        self.right_controller_poses = []
        self.buffer_num = 350
        self.tf_pub = StaticTransformBroadcaster(self)

        self.left_target_pose_pub = self.create_publisher(
            PoseStamped, "/quest3/target_left", 10)
        self.right_target_pose_pub = self.create_publisher(
            PoseStamped, "/quest3/target_right", 10)

        self.height = 0.03
        self.z_offset = 0.0
        self.y_offset = 0.1

        self.dump_file = "/teleop_ws/src/rs_motion_capture/config/sensor/quest3_glove_calibration.yaml"

        # debug
        if self.mode == "vis":
            self.map_ori_left_hand_pub = self.create_publisher(
                PoseStamped, "/quest3/ori/left_hand", 10)
            self.map_ori_right_hand_pub = self.create_publisher(
                PoseStamped, "/quest3/ori/right_hand", 10)

            self.map_reflect_left_hand_pub = self.create_publisher(
                PoseStamped, "/quest3/reflect/left_hand", 10)
            self.map_reflect_right_hand_pub = self.create_publisher(
                PoseStamped, "/quest3/reflect/right_hand", 10)
            with open(self.dump_file, "r") as f:
                self.calib = yaml.safe_load(f)
                self.left_rot = Rotation.from_quat(
                    [self.calib["left"]["qx"], self.calib["left"]["qy"], self.calib["left"]["qz"], self.calib["left"]["qw"]]).as_matrix()
                self.right_rot = Rotation.from_quat(
                    [self.calib["right"]["qx"], self.calib["right"]["qy"], self.calib["right"]["qz"], self.calib["right"]["qw"]]).as_matrix()

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
        read_quest3_to_map = False
        # read_map_to_left = False
        # read_map_to_right = False
        while rclpy.ok():
            self.quest3_to_map = self.get_transform(
                "quest3",
                "map"
            )
            if self.quest3_to_map is None:
                time.sleep(1.0)
                continue
            else:
                self.get_logger().info("read quest3 ---> map done")
                read_quest3_to_map = True
                break

        # while rclpy.ok():
        #     self.map_to_left_arm_dummy_base_link = self.get_transform(
        #         "map",
        #         "left_arm_dummy_base_link"
        #     )
        #     if self.map_to_left_arm_dummy_base_link is None:
        #         time.sleep(1.0)
        #         continue
        #     else:
        #         self.get_logger().info("read map ---> left_arm_dummy_base_link done")
        #         read_map_to_left = True
        #         break
        #
        # while rclpy.ok():
        #     self.map_to_right_arm_dummy_base_link = self.get_transform(
        #         "map",
        #         "right_arm_dummy_base_link"
        #     )
        #     if self.map_to_right_arm_dummy_base_link is None:
        #         time.sleep(1.0)
        #         continue
        #     else:
        #         self.get_logger().info("read map ---> right_arm_dummy_base_link done")
        #         read_map_to_right = True
        #         break

        # if read_quest3_to_map and read_map_to_left and read_map_to_right:
        if read_quest3_to_map:
            self.tf_ready = True

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
        pose_matrix[1, 3] -= 1.2
        pose = matrix_to_pose(pose_matrix)
        return pose

    def calib_process(self):
        left_quat = [Rotation.from_matrix(
            t[:3, :3]).as_quat() for t in self.left_controller_poses[100:]]
        left_position = np.array([t[:3, 3] for t in self.left_controller_poses[100:]]).reshape(-1, 3)
        left_mean_quat= Rotation.from_quat(np.array(left_quat)).mean()
        left_mean_rot_axis= left_mean_quat.as_matrix()
        left_mean_position= left_position.mean(axis=0)

        right_quat= [Rotation.from_matrix(
            t[:3, :3]).as_quat() for t in self.right_controller_poses[100:]]
        right_position = np.array([t[:3, 3] for t in self.right_controller_poses[100:]]).reshape(-1, 3)
        right_mean_quat= Rotation.from_quat(np.array(right_quat)).mean()
        right_mean_rot_axis= right_mean_quat.as_matrix()
        right_mean_position= right_position.mean(axis=0)

        left_x_vec= left_mean_position - right_mean_position
        left_x_axis_len= np.linalg.norm(left_x_vec)
        left_x_axis= left_x_vec / left_x_axis_len
        left_y_axis= np.array([0., 0., 1.])
        left_z_axis= np.cross(left_x_axis, left_y_axis)
        target_left_axis= np.array([
            left_x_axis,
            left_y_axis,
            left_z_axis
        ]).T
        left_tf= TransformStamped()
        left_tf.header.frame_id= "map"
        left_tf.header.stamp= self.get_clock().now().to_msg()
        left_tf.child_frame_id= "target_left"
        left_tf.transform.translation.x= 0.
        left_tf.transform.translation.y= 1.
        left_tf.transform.translation.z= 0.5
        target_left_quat= Rotation.from_matrix(target_left_axis).as_quat()
        left_tf.transform.rotation.x= float(target_left_quat[0])
        left_tf.transform.rotation.y= float(target_left_quat[1])
        left_tf.transform.rotation.z= float(target_left_quat[2])
        left_tf.transform.rotation.w= float(target_left_quat[3])
        self.tf_pub.sendTransform(left_tf)


        # left_convert = target_left_axis @ np.linalg.inv(left_mean_rot_axis)
        left_convert= np.linalg.inv(left_mean_rot_axis) @ target_left_axis

        right_x_vec= right_mean_position - left_mean_position
        right_x_axis_len= np.linalg.norm(right_x_vec)
        right_x_axis= right_x_vec / right_x_axis_len
        right_y_axis= np.array([0., 0., -1.])
        right_z_axis= np.cross(right_x_axis, right_y_axis)
        target_right_axis= np.array([
            right_x_axis,
            right_y_axis,
            right_z_axis
        ]).T

        right_tf= TransformStamped()
        right_tf.header.frame_id= "map"
        right_tf.header.stamp= self.get_clock().now().to_msg()
        right_tf.child_frame_id= "target_right"
        right_tf.transform.translation.x= 0.
        right_tf.transform.translation.y= -1.
        right_tf.transform.translation.z= 0.5
        target_right_quat= Rotation.from_matrix(target_right_axis).as_quat()
        right_tf.transform.rotation.x= float(target_right_quat[0])
        right_tf.transform.rotation.y= float(target_right_quat[1])
        right_tf.transform.rotation.z= float(target_right_quat[2])
        right_tf.transform.rotation.w= float(target_right_quat[3])
        self.tf_pub.sendTransform(right_tf)
        # right_convert = target_right_axis @ np.linalg.inv(right_mean_rot_axis)
        right_convert= np.linalg.inv(right_mean_rot_axis) @ target_right_axis

        return left_convert, right_convert

    def dump_convert(self, left_convert, right_convert):
        left_quat= Rotation.from_matrix(left_convert).as_quat()
        left_euler = Rotation.from_matrix(left_convert).as_euler(seq="XYZ", degrees=True)
        right_quat= Rotation.from_matrix(right_convert).as_quat()
        right_euler = Rotation.from_matrix(right_convert).as_euler(seq="XYZ", degrees=True)
        res= {
            "left": {
                "qx": float(left_quat[0]),
                "qy": float(left_quat[1]),
                "qz": float(left_quat[2]),
                "qw": float(left_quat[3]),
                "height": self.height,
                "z_offset": self.z_offset,
                "y_offset": self.y_offset
            },
            "right": {
                "qx": float(right_quat[0]),
                "qy": float(right_quat[1]),
                "qz": float(right_quat[2]),
                "qw": float(right_quat[3]),
                "height": self.height,
                "z_offset": self.z_offset,
                "y_offset": self.y_offset
            }
        }
        with open(self.dump_file, "w") as f:
            yaml.safe_dump(res, f)


    def input_process(self, msg: Quest3Input):

        if not self.tf_ready:
            return

        head_pose= left_hand_to_right_hand_axis(msg.head_pose)
        if not self.get_smooth_pose(head_pose):
            return

        rel_pose= self.get_rel_pose()
        if rel_pose is None:
            return

        left_pose= deepcopy(
            left_hand_to_right_hand_axis(msg.left_controller.pose))
        right_pose= deepcopy(
            left_hand_to_right_hand_axis(msg.right_controller.pose))
        left_pose, right_pose= trans_pose_2_local(
            rel_pose,
            [
                left_pose,
                right_pose
            ]
        )

        map_left_pose_matrix= self.quest3_to_map @ pose_to_matrix(left_pose)
        map_right_pose_matrix= self.quest3_to_map @ pose_to_matrix(right_pose)

        if self.mode == "calib":
            self.left_controller_poses.append(map_left_pose_matrix)
            self.right_controller_poses.append(map_right_pose_matrix)
            if len(self.left_controller_poses) >= self.buffer_num:
                left_convert, right_convert= self.calib_process()
                self.dump_convert(left_convert, right_convert)
                self.get_logger().info(f"dump {self.dump_file} done")
                rclpy.shutdown()
        else:
            self.vis_process(matrix_to_pose(map_left_pose_matrix),
                             matrix_to_pose(map_right_pose_matrix))

    def vis_process(self, map_left_pose, map_right_pose):
        pub_left_ori_pose= PoseStamped()
        pub_left_ori_pose.header.frame_id= "map"
        pub_left_ori_pose.header.stamp= self.get_clock().now().to_msg()
        pub_left_ori_pose.pose= map_left_pose
        self.map_ori_left_hand_pub.publish(pub_left_ori_pose)

        pub_right_ori_pose= PoseStamped()
        pub_right_ori_pose.header.frame_id= "map"
        pub_right_ori_pose.header.stamp= self.get_clock().now().to_msg()
        pub_right_ori_pose.pose= map_right_pose
        self.map_ori_right_hand_pub.publish(pub_right_ori_pose)

        # pub reflect
        left_reflect_rot_matrix = pose_to_matrix(map_left_pose)[:3, :3] @ self.left_rot
        left_reflect_quat= Rotation.from_matrix(left_reflect_rot_matrix).as_quat()

        pub_left_reflect_pose= PoseStamped()
        pub_left_reflect_pose.header.frame_id= "map"
        pub_left_reflect_pose.header.stamp= self.get_clock().now().to_msg()
        left_tmp_pose= Pose()
        left_tmp_pose.orientation.x= left_reflect_quat[0]
        left_tmp_pose.orientation.y= left_reflect_quat[1]
        left_tmp_pose.orientation.z= left_reflect_quat[2]
        left_tmp_pose.orientation.w= left_reflect_quat[3]
        left_tmp_pose.position.x= map_left_pose.position.x
        left_tmp_pose.position.y= map_left_pose.position.y
        left_tmp_pose.position.z= map_left_pose.position.z
        pub_left_reflect_pose.pose= left_tmp_pose
        self.map_reflect_left_hand_pub.publish(pub_left_reflect_pose)


        right_reflect_rot_matrix = pose_to_matrix(map_right_pose)[:3, :3] @ self.right_rot
        right_reflect_quat= Rotation.from_matrix(right_reflect_rot_matrix).as_quat()

        pub_right_reflect_pose= PoseStamped()
        pub_right_reflect_pose.header.frame_id= "map"
        pub_right_reflect_pose.header.stamp= self.get_clock().now().to_msg()
        right_tmp_pose= Pose()
        right_tmp_pose.orientation.x= right_reflect_quat[0]
        right_tmp_pose.orientation.y= right_reflect_quat[1]
        right_tmp_pose.orientation.z= right_reflect_quat[2]
        right_tmp_pose.orientation.w= right_reflect_quat[3]
        right_tmp_pose.position.x= map_right_pose.position.x
        right_tmp_pose.position.y= map_right_pose.position.y
        right_tmp_pose.position.z= map_right_pose.position.z
        pub_right_reflect_pose.pose= right_tmp_pose
        self.map_reflect_right_hand_pub.publish(pub_right_reflect_pose)



def main(args=None):
    rclpy.init(args=args)
    executor= MultiThreadedExecutor()

    node= Quest3GloveCalibNode()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
