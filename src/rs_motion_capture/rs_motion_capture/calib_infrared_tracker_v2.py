#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, TransformStamped
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
import os.path as osp
import numpy as np
from typing import List
import threading
import yaml
from copy import deepcopy
from scipy.spatial.transform import Rotation
import time
from tf2_ros import TransformBroadcaster
import os

DUMP_DIR = "/teleop_ws/src/rs_motion_capture/config"


class DualInfraredTrackerCalibV2(Node):
    def __init__(self):
        super().__init__("DualInfraredTrackerCalib")
        self.root_path = DUMP_DIR
        self.declare_parameter("collector", "eric.li")
        self.declare_parameter("mode", "calib")
        self.mode = self.get_parameter("mode").value
        self.calib_hci_pub = self.create_publisher(
            String, "/robot_control/hci_status", 10)
        if self.mode == "calib":
            self.publish_hci_status("infrared_body_calib_step1")
        self.left_tracker_sub = message_filters.Subscriber(
            self, PoseStamped, "/tracker/left/pose")
        self.right_tracker_sub = message_filters.Subscriber(
            self, PoseStamped, "/tracker/right/pose")
        self.left_poses = []
        self.right_poses = []
        self.left_poses1 = []
        self.right_poses1 = []
        self.left_poses2 = []
        self.right_poses2 = []
        self.cur_left_pose = self.left_poses1
        self.cur_right_pose = self.right_poses1
        self.pose1_end = False
        self.pose2_end = False
        self.buffer_num = 500
        self.messaeg_idx = 0
        self.sync = self.create_synchronizer(
            [self.left_tracker_sub, self.right_tracker_sub],
            self.tracker_callback, True)
        self.get_logger().info(f"mode {self.mode}")
        if self.mode == "calib":
            self.core = threading.Thread(target=self.calib_process)
        else:
            self.core = threading.Thread(target=self.vis_fit_pose)
        self.lock = threading.Lock()
        self.buffer_full_cv = threading.Condition(self.lock)
        self.running = False
        # self.left_correct_pub = self.create_publisher(PoseStamped, "/tracker/left/adapt_pose", 10)
        # self.right_correct_pub = self.create_publisher(PoseStamped, "/tracker/right/adapt_pose", 10)
        self.neck_pose_pub = self.create_publisher(
            PoseStamped, "/tracker/neck/adapt_pose", 10)
        self.left_pose_pub = self.create_publisher(
            PoseStamped, "/target/left/adapt_pose", 10)
        self.right_pose_pub = self.create_publisher(
            PoseStamped, "/target/right/adapt_pose", 10)

        self.mqtt_sub = self.create_publisher(
            String, "/mqtt/calib_progress", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.dump_path = osp.join(self.root_path, "sensor",
                                  "infrared_tracker_calibration.yaml")
        if self.mode == "vis":
            if os.path.exists(self.dump_path):
                with open(self.dump_path, "r") as f:
                    self.calib = yaml.load(f)
        if self.mode != "calib":
            self.empty_hand_pub = self.create_publisher(
                PoseArray, "/hand_poses_noiton", 10)



    def start(self):
        self.running = True
        time.sleep(2)
        self.core.start()

    def stop(self):
        with self.buffer_full_cv:
            self.running = False
            self.buffer_full_cv.notify_all()

    def create_synchronizer(self, subscribers, callback, approx=True):
        sync = None
        if approx:
            sync = message_filters.ApproximateTimeSynchronizer(
                subscribers,
                queue_size=100,
                slop=0.005
            )
        else:
            sync = message_filters.TimeSynchronizer(
                subscribers,
                queue_size=100
            )
        sync.registerCallback(callback)
        return sync

    def check_stable(self, poses: List[np.ndarray], threshold: float):
        nd_poses = deepcopy(np.array(poses))
        # mean_x = np.mean(nd_poses[:,0])
        # mean_y = np.mean(nd_poses[:,1])
        # mean_z = np.mean(nd_poses[:,2])
        max_x = np.max(nd_poses[:, 0])
        max_y = np.max(nd_poses[:, 1])
        max_z = np.max(nd_poses[:, 2])
        min_x = np.min(nd_poses[:, 0])
        min_y = np.min(nd_poses[:, 1])
        min_z = np.min(nd_poses[:, 2])
        diff_x = max_x - min_x
        diff_y = max_y - min_y
        diff_z = max_z - min_z
        self.get_logger().info(
            f"diff_x: {diff_x}, diff_y: {diff_y}, diff_z: {diff_z}")
        # if diff_x > 0.01 or diff_y > 0.01 or diff_z > 0.01:
        #     return False

        std_x = np.std(nd_poses[:, 0])
        std_y = np.std(nd_poses[:, 1])
        std_z = np.std(nd_poses[:, 2])
        self.get_logger().info(
            f"std_x: {std_x}, std_y: {std_y}, std_z: {std_z}")
        if std_x > threshold or std_y > threshold or std_z > threshold:
            return False
        return True

    def publish_hci_status(self, status: str):
        msg = String()
        msg.data = status
        self.calib_hci_pub.publish(msg)

    def publish_empty_hand(self, header):
        empty_pose = PoseArray()
        empty_pose.header = header
        empty_pose.poses = [Pose()] * 40
        self.empty_hand_pub.publish(empty_pose)

    def tracker_callback(self, left_msg: PoseStamped, right_msg: PoseStamped):
        with self.buffer_full_cv:
            # self.buffer_full_cv.wait_for(lambda: self.running)
            if not self.running:
                return

            if len(self.left_poses1) >= self.buffer_num and self.mode == "calib":
                if not self.pose1_end:
                    self.get_logger().warn(f"pose1 is end, after 1s do pose2")
                    self.pose1_end = True
                    self.publish_hci_status("infrared_body_calib_step2")
                    self.cur_left_pose = self.left_poses2
                    self.cur_right_pose = self.right_poses2

            if len(self.left_poses2) >= self.buffer_num and self.mode == "calib":
                if not self.pose2_end:
                    self.get_logger().warn(f"pose2 is end, cal_calib")
                    self.pose2_end = True
                    self.buffer_full_cv.notify_all()
                return
            lx = left_msg.pose.position.x
            ly = left_msg.pose.position.y
            lz = left_msg.pose.position.z
            lqx = left_msg.pose.orientation.x
            lqy = left_msg.pose.orientation.y
            lqz = left_msg.pose.orientation.z
            lqw = left_msg.pose.orientation.w

            rx = right_msg.pose.position.x
            ry = right_msg.pose.position.y
            rz = right_msg.pose.position.z
            rqx = right_msg.pose.orientation.x
            rqy = right_msg.pose.orientation.y
            rqz = right_msg.pose.orientation.z
            rqw = right_msg.pose.orientation.w
            self.messaeg_idx += 1

            if self.messaeg_idx < self.buffer_num or self.messaeg_idx > self.buffer_num + 300 and self.mode == "calib":
                self.cur_left_pose.append(
                    np.array([lx, ly, lz, lqx, lqy, lqz, lqw]))
                self.cur_right_pose.append(
                    np.array([rx, ry, rz, rqx, rqy, rqz, rqw]))
                if not self.pose1_end:
                    mqtt = String()
                    mqtt.data = f"infrared_calib:{len(self.left_poses1) *50 / self.buffer_num:.1f}"
                    self.get_logger().warn(f"{mqtt.data}")
                    self.mqtt_sub.publish(mqtt)
                    self.get_logger().info(
                        f"receive pose1 message {len(self.left_poses1)}")
                if self.pose1_end and not self.pose2_end:
                    mqtt = String()
                    mqtt.data = f"infrared_calib:{50+ len(self.left_poses2)*50 / self.buffer_num : .1f}"
                    self.mqtt_sub.publish(mqtt)
                    self.get_logger().info(
                        f"receive pose1 message {len(self.left_poses1)}")

            if self.mode != "calib":
                self.left_poses.append(
                    np.array([lx, ly, lz, lqx, lqy, lqz, lqw]))
                self.right_poses.append(
                    np.array([rx, ry, rz, rqx, rqy, rqz, rqw]))

            # self.get_logger().info(f"receive msg {len(self.left_poses)} {len(self.right_poses)}")
            self.buffer_full_cv.notify_all()

    def is_fit(self):
        flag = True
        flag &= len(self.left_poses) == self.buffer_num
        flag &= len(self.right_poses) == self.buffer_num
        if not flag:
            return flag
        flag &= self.check_stable(self.left_poses, 0.2)
        flag &= self.check_stable(self.right_poses, 0.2)
        return flag

    def cal_neck_point_and_P1(self):
        nd_left_pose = deepcopy(np.array(deepcopy(self.left_poses1)))
        nd_right_pose = deepcopy(np.array(deepcopy(self.right_poses1)))
        left_mean_x = np.mean(nd_left_pose[:, 0])
        left_mean_y = np.mean(nd_left_pose[:, 1])
        left_mean_z = np.mean(nd_left_pose[:, 2])
        right_mean_x = np.mean(nd_right_pose[:, 0])
        right_mean_y = np.mean(nd_right_pose[:, 1])
        right_mean_z = np.mean(nd_right_pose[:, 2])

        center_x = (left_mean_x + right_mean_x) / 2
        center_y = (left_mean_y + right_mean_y) / 2
        center_z = (left_mean_z + right_mean_z) / 2
        P0 = np.array([center_x, center_y, center_z])
        P1 = np.array([left_mean_x, left_mean_y, left_mean_z])
        return P0, P1

    def cal_P2(self):
        nd_left_pose = deepcopy(np.array(deepcopy(self.left_poses2)))
        left_mean_x = np.mean(nd_left_pose[:, 0])
        left_mean_y = np.mean(nd_left_pose[:, 1])
        left_mean_z = np.mean(nd_left_pose[:, 2])
        P2 = np.array([left_mean_x, left_mean_y, left_mean_z])
        return P2

    def cal_rotation_and_translation(self, P0, P1, P2):
        V1 = P1 - P0  # +y
        V2 = P2 - P0
        u_y = V1 / np.linalg.norm(V1)
        proj = np.dot(V2, u_y) * u_y
        W = V2 - proj
        u_x = W / np.linalg.norm(W)
        u_z = np.cross(u_x, u_y)
        R = np.column_stack((u_x, u_y, u_z))
        self.get_logger().info(
            f"cal_rotation_and_translation: P0: {P0}, P1: {P1}, P2: {P2}")
        self.get_logger().info(
            f"cal_rotation_and_translation: V1: {V1}, V2:{V2}")
        self.get_logger().info(f"R: {R}")
        self.get_logger().info(f"u_x: {u_x}, u_y: {u_y}, u_z: {u_z}")
        t = P0

        R_inv = R.T
        t_inv = -R_inv @ t
        T_world_to_new = np.eye(4)
        T_world_to_new[:3, :3] = R_inv
        T_world_to_new[:3, 3] = t_inv
        return T_world_to_new

    def calib_process(self):
        with self.buffer_full_cv:
            self.buffer_full_cv.wait_for(
                lambda: not self.running or (self.pose1_end and self.pose2_end))
            if not self.running:
                return
            P0, P1 = self.cal_neck_point_and_P1()
            P2 = self.cal_P2()
            world2target = self.cal_rotation_and_translation(P0, P1, P2)
            target2world = np.linalg.inv(world2target)
            # target2world = np.linalg.inv(world2target)
            # res_quat = Rotation.from_matrix(target2world).as_quat()
            quat = Rotation.from_matrix(world2target[:3, :3]).as_quat()
            inv_quat = Rotation.from_matrix(target2world[:3, :3]).as_quat()

            dump_path = osp.join(self.root_path, "sensor",
                                 "infrared_tracker_calibration.yaml")
            res = {}
            res["neck"] = {
                "base_link": "world",
                "x": float(world2target[0, 3]),
                "y": float(world2target[1, 3]),
                "z": float(world2target[2, 3]),
                "qx": float(quat[0]),
                "qy": float(quat[1]),
                "qz": float(quat[2]),
                "qw": float(quat[3]),
            }
            res["inv_neck"] = {
                "base_link": "world",
                "x": float(target2world[0, 3]),
                "y": float(target2world[1, 3]),
                "z": float(target2world[2, 3]),
                "qx": float(inv_quat[0]),
                "qy": float(inv_quat[1]),
                "qz": float(inv_quat[2]),
                "qw": float(inv_quat[3]),
            }

            new_pose = PoseStamped()
            new_pose.header.frame_id = "tracker"
            new_pose.pose.position.x = float(world2target[0, 3])
            new_pose.pose.position.y = float(world2target[1, 3])
            new_pose.pose.position.z = float(world2target[2, 3])
            new_pose.pose.orientation.x = float(quat[0])
            new_pose.pose.orientation.y = float(quat[1])
            new_pose.pose.orientation.z = float(quat[2])
            new_pose.pose.orientation.w = float(quat[3])
            self.neck_pose_pub.publish(new_pose)
            self.get_logger().info(f"{res}")
            with open(dump_path, "w") as f:
                yaml.safe_dump(res, f)
            self.get_logger().info(f"dump to {dump_path} success")
            mqtt = String()
            mqtt.data = "infrared_calib:100"
            self.mqtt_sub.publish(mqtt)

            self.publish_hci_status("calib_finish")

        # self.stop()
        rclpy.shutdown()

    def publish_tf(self, header):
        t = TransformStamped()
        t.header.stamp = header.stamp
        t.header.frame_id = "base_link"  # 父坐标系（原点）
        t.child_frame_id = "target"    # 子坐标系（设备）
        # 位置（x,y,z）
        t.transform.translation.x = self.calib["inv_neck"]["x"]
        t.transform.translation.y = self.calib["inv_neck"]["y"]
        t.transform.translation.z = self.calib["inv_neck"]["z"]
        # 姿态（四元数 x,y,z,w）
        t.transform.rotation.x = self.calib["inv_neck"]["qx"]
        t.transform.rotation.y = self.calib["inv_neck"]["qy"]
        t.transform.rotation.z = self.calib["inv_neck"]["qz"]
        t.transform.rotation.w = self.calib["inv_neck"]["qw"]
        self.tf_broadcaster.sendTransform(t)

    def vis_fit_pose(self):
        self.get_logger().info("vis_fit_pose")
        while True:
            with self.buffer_full_cv:
                self.buffer_full_cv.wait_for(
                    lambda: not self.running or len(self.left_poses) > 0)
                if not self.running:
                    return

                neck_msg = PoseStamped()
                neck_msg.header.frame_id = "target"
                neck_msg.pose.position.x = self.calib["neck"]["x"]
                neck_msg.pose.position.y = self.calib["neck"]["y"]
                neck_msg.pose.position.z = self.calib["neck"]["z"]
                neck_msg.pose.orientation.x = self.calib["neck"]["qx"]
                neck_msg.pose.orientation.y = self.calib["neck"]["qy"]
                neck_msg.pose.orientation.z = self.calib["neck"]["qz"]
                neck_msg.pose.orientation.w = self.calib["neck"]["qw"]
                self.neck_pose_pub.publish(neck_msg)
                left_world2target = np.eye(4)
                r = Rotation.from_quat([self.calib["neck"]["qx"], self.calib["neck"]["qy"],
                                       self.calib["neck"]["qz"], self.calib["neck"]["qw"]]).as_matrix()
                left_world2target[:3, :3] = r
                left_world2target[:3, 3] = [self.calib["neck"]["x"],
                                            self.calib["neck"]["y"], self.calib["neck"]["z"]]
                left_pose = self.left_poses.pop(0)
                left_pos = np.eye(4)
                left_rotation = Rotation.from_quat(
                    [left_pose[3], left_pose[4], left_pose[5], left_pose[6]]).as_matrix()
                left_position = [left_pose[0], left_pose[1], left_pose[2]]
                left_pos[:3, :3] = left_rotation
                left_pos[:3, 3] = left_position
                left_fit_pos = left_world2target @ left_pos
                left_q = Rotation.from_matrix(left_fit_pos[:3, :3]).as_quat()
                left_msg = PoseStamped()
                left_msg.header.frame_id = "target"
                left_msg.pose.position.x = left_fit_pos[0, 3]
                left_msg.pose.position.y = left_fit_pos[1, 3]
                left_msg.pose.position.z = left_fit_pos[2, 3]
                left_msg.pose.orientation.x = left_q[0]
                left_msg.pose.orientation.y = left_q[1]
                left_msg.pose.orientation.z = left_q[2]
                left_msg.pose.orientation.w = left_q[3]
                self.left_pose_pub.publish(left_msg)

                right_world2target = np.eye(4)
                r = Rotation.from_quat([self.calib["neck"]["qx"], self.calib["neck"]["qy"],
                                       self.calib["neck"]["qz"], self.calib["neck"]["qw"]]).as_matrix()
                right_world2target[:3, :3] = r
                right_world2target[:3, 3] = [
                    self.calib["neck"]["x"], self.calib["neck"]["y"], self.calib["neck"]["z"]]
                right_pose = self.right_poses.pop(0)
                right_pos = np.eye(4)
                right_rotation = Rotation.from_quat(
                    [right_pose[3], right_pose[4], right_pose[5], right_pose[6]]).as_matrix()
                right_position = [right_pose[0], right_pose[1], right_pose[2]]
                right_pos[:3, :3] = right_rotation
                right_pos[:3, 3] = right_position
                right_fit_pos = right_world2target @ right_pos
                right_q = Rotation.from_matrix(right_fit_pos[:3, :3]).as_quat()
                right_msg = PoseStamped()
                right_msg.header.frame_id = "target"
                right_msg.pose.position.x = right_fit_pos[0, 3]
                right_msg.pose.position.y = right_fit_pos[1, 3]
                right_msg.pose.position.z = right_fit_pos[2, 3]
                right_msg.pose.orientation.x = right_q[0]
                right_msg.pose.orientation.y = right_q[1]
                right_msg.pose.orientation.z = right_q[2]
                right_msg.pose.orientation.w = right_q[3]
                self.right_pose_pub.publish(right_msg)
                self.publish_tf(neck_msg.header)

                # self.get_logger().info("publish")
                #

    def destroy_node(self):
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = DualInfraredTrackerCalibV2()
    node.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
