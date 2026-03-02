#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import PoseStamped,PoseArray, Pose
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
import os

DUMP_DIR = "/teleop_ws/src/rs_motion_capture/config"


class DualInfraredTrackerCalib(Node):
    def __init__(self):
        super().__init__("DualInfraredTrackerCalib")
        self.root_path = DUMP_DIR
        self.declare_parameter("collector","eric.li")
        self.declare_parameter("mode","calib")
        self.left_tracker_sub = message_filters.Subscriber(
            self, PoseStamped, "/tracker/left/pose")
        self.right_tracker_sub = message_filters.Subscriber(
            self, PoseStamped, "/tracker/right/pose")
        self.left_poses = []
        self.right_poses = []
        self.buffer_num = 500
        self.sync = self.create_synchronizer(
            [self.left_tracker_sub, self.right_tracker_sub],
            self.tracker_callback, True)
        self.mode = self.get_parameter("mode").value
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
        self.neck_pose_pub = self.create_publisher(PoseStamped, "/tracker/neck/adapt_pose", 10)
        self.left_pose_pub = self.create_publisher(PoseStamped, "/tracker/left/adapt_pose", 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, "/tracker/right/adapt_pose", 10)

        self.dump_path = osp.join(self.root_path, "sensor",
                                "infrared_tracker_calibration.yaml")
        if self.mode == "vis":
            if os.path.exists(self.dump_path):
                with open(self.dump_path, "r") as f:
                    self.calib = yaml.load(f)
        if self.mode != "calib":
            self.empty_hand_pub = self.create_publisher(PoseArray, "/hand_poses_noiton", 10)

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
            lx = left_msg.pose.position.x
            ly = left_msg.pose.position.y
            lz = left_msg.pose.position.z
            lqx = left_msg.pose.orientation.x
            lqy = left_msg.pose.orientation.y
            lqz = left_msg.pose.orientation.z
            lqw = left_msg.pose.orientation.w
            self.left_poses.append(np.array([lx, ly, lz, lqx, lqy, lqz, lqw]))
            if len(self.left_poses) > self.buffer_num:
                self.left_poses.pop(0)

            rx = right_msg.pose.position.x
            ry = right_msg.pose.position.y
            rz = right_msg.pose.position.z
            rqx = right_msg.pose.orientation.x
            rqy = right_msg.pose.orientation.y
            rqz = right_msg.pose.orientation.z
            rqw = right_msg.pose.orientation.w
            self.right_poses.append(np.array([rx, ry, rz, rqx, rqy, rqz, rqw]))
            if len(self.right_poses) > self.buffer_num:
                self.right_poses.pop(0)
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

    def calib_process(self):
        with self.buffer_full_cv:
            self.buffer_full_cv.wait_for(
                lambda: not self.running or self.is_fit())
            if not self.running:
                return
            nd_left_pose = deepcopy(np.array(deepcopy(self.left_poses)))
            left_mean_x = np.mean(nd_left_pose[:, 0])
            left_mean_y = np.mean(nd_left_pose[:, 1])
            left_mean_z = np.mean(nd_left_pose[:, 2])

            nd_right_pose = deepcopy(np.array(deepcopy(self.right_poses)))
            right_mean_x = np.mean(nd_right_pose[:, 0])
            right_mean_y = np.mean(nd_right_pose[:, 1])
            right_mean_z = np.mean(nd_right_pose[:, 2])

            center_x = (left_mean_x + right_mean_x) / 2
            center_y = (left_mean_y + right_mean_y) / 2
            center_z = (left_mean_z + right_mean_z) / 2

            right_rotations = Rotation.from_quat(nd_right_pose[:, 3:])
            right_mean_rotation = right_rotations.mean()
            right_mean_quat = right_mean_rotation.as_quat()
            right_tracker2world = np.eye(4)
            right_tracker2world[:3, :3] = Rotation.from_quat(right_mean_quat).as_matrix()
            right_tracker2world[:3, 3] = np.array([center_x, center_y, center_z])

            left_rotations = Rotation.from_quat(nd_left_pose[:, 3:])
            left_mean_rotation = left_rotations.mean()
            left_mean_quat = left_mean_rotation.as_quat()
            left_tracker2world = np.eye(4)
            left_tracker2world[:3, :3] = Rotation.from_quat(left_mean_quat).as_matrix()
            left_tracker2world[:3, 3] = np.array([center_x, center_y, center_z])

            world2left_tracker = np.linalg.inv(left_tracker2world)

            world2right_tracker = np.linalg.inv(right_tracker2world)

            # left_tracker2target = np.array([
            #     [1, 0, 0],
            #     [0, 0, -1],
            #     [0, 1, 0],
            # ])
            # left_tracker2target = np.array([
            #     [0, 1, 0],
            #     [0, 0, -1],
            #     [-1, 0, 0],
            # ])
            left_tracker2target = np.array([
                [0, 0, 1, 0],
                [0, 1, 0, 0],
                [-1, 0, 0, 0],
                [0, 0, 0, 1],
            ])

            right_tracker2target = np.array([
                [0, 0, -1, 0],
                [0, -1, 0, 0],
                [-1, 0, 0, 0],
                [0, 0, 0, 1],
            ])
            # left_tracker2target = np.eye(3)
            left_world2target = left_tracker2target @ world2left_tracker
            right_world2target = right_tracker2target @ world2right_tracker
            # target2world = np.linalg.inv(world2target)
            # res_quat = Rotation.from_matrix(target2world).as_quat()
            left_quat = Rotation.from_matrix(left_world2target[:3, :3]).as_quat()
            right_quat = Rotation.from_matrix(right_world2target[:3, :3]).as_quat()


            dump_path = osp.join(self.root_path, "sensor",
                                 "infrared_tracker_calibration.yaml")
            res = {}
            res["left_spread_pose"] = {
                "x": float(left_mean_x),
                "y": float(left_mean_y),
                "z": float(left_mean_z),
            }
            res["right_spread_pose"] = {
                "x": float(right_mean_x),
                "y": float(right_mean_y),
                "z": float(right_mean_z),
            }

            res["left_neck"] = {
                "base_link": "world",
                "x": float(left_world2target[0,3]),
                "y": float(left_world2target[1,3]),
                "z": float(left_world2target[2,3]),
                "qx": float(left_quat[0]),
                "qy": float(left_quat[1]),
                "qz": float(left_quat[2]),
                "qw": float(left_quat[3]),
            }

            res["right_neck"] = {
                "base_link": "world",
                "x": float(right_world2target[0,3]),
                "y": float(right_world2target[1,3]),
                "z": float(right_world2target[2,3]),
                "qx": float(right_quat[0]),
                "qy": float(right_quat[1]),
                "qz": float(right_quat[2]),
                "qw": float(right_quat[3]),
            }
            left_fix_pose  = PoseStamped()
            left_fix_pose.header.frame_id = "tracker"
            left_fix_pose.pose.position.x = float(center_x)
            left_fix_pose.pose.position.y = float(center_y)
            left_fix_pose.pose.position.z = float(center_z)
            left_fix_pose.pose.orientation.x = float(left_quat[0])
            left_fix_pose.pose.orientation.y = float(left_quat[1])
            left_fix_pose.pose.orientation.z = float(left_quat[2])
            left_fix_pose.pose.orientation.w = float(left_quat[3])
            self.neck_pose_pub.publish(left_fix_pose)
            self.get_logger().info(f"{res}")
            with open(dump_path, "w") as f:
                yaml.safe_dump(res, f)
            self.get_logger().info(f"dump to {dump_path} success")

            #self.stop()
        rclpy.shutdown()
        exit(0)

    def vis_fit_pose(self):
        self.get_logger().info("vis_fit_pose")
        while True:
            with self.buffer_full_cv:
                self.buffer_full_cv.wait_for(
                    lambda: not self.running or len(self.left_poses)>0)
                if not self.running:
                    return

                neck_msg = PoseStamped()
                neck_msg.header.frame_id = "target"
                neck_msg.pose.position.x = self.calib["left_neck"]["x"]
                neck_msg.pose.position.y = self.calib["left_neck"]["y"]
                neck_msg.pose.position.z = self.calib["left_neck"]["z"]
                neck_msg.pose.orientation.x = self.calib["left_neck"]["qx"]
                neck_msg.pose.orientation.y = self.calib["left_neck"]["qy"]
                neck_msg.pose.orientation.z = self.calib["left_neck"]["qz"]
                neck_msg.pose.orientation.w = self.calib["left_neck"]["qw"]
                self.neck_pose_pub.publish(neck_msg)
                left_world2target = np.eye(4)
                r = Rotation.from_quat([self.calib["left_neck"]["qx"], self.calib["left_neck"]["qy"],self.calib["left_neck"]["qz"],self.calib["left_neck"]["qw"]]).as_matrix()
                left_world2target[:3,:3] = r
                left_world2target[:3, 3] = [self.calib["left_neck"]["x"], self.calib["left_neck"]["y"],self.calib["left_neck"]["z"]]
                left_pose = self.left_poses.pop(0)
                left_pos = np.eye(4)
                left_rotation = Rotation.from_quat([left_pose[3],left_pose[4],left_pose[5],left_pose[6]]).as_matrix()
                left_position = [left_pose[0], left_pose[1],left_pose[2]]
                left_pos[:3, :3] = left_rotation
                left_pos[:3, 3] = left_position
                left_fit_pos = left_world2target @ left_pos
                left_q = Rotation.from_matrix(left_fit_pos[:3,:3]).as_quat()
                left_msg = PoseStamped()
                left_msg.header.frame_id = "target"
                left_msg.pose.position.x = left_fit_pos[0,3]
                left_msg.pose.position.y = left_fit_pos[1,3]
                left_msg.pose.position.z = left_fit_pos[2,3]
                left_msg.pose.orientation.x = left_q[0]
                left_msg.pose.orientation.y = left_q[1]
                left_msg.pose.orientation.z = left_q[2]
                left_msg.pose.orientation.w = left_q[3]
                self.left_pose_pub.publish(left_msg)


                right_world2target = np.eye(4)
                r = Rotation.from_quat([self.calib["right_neck"]["qx"], self.calib["right_neck"]["qy"],self.calib["right_neck"]["qz"],self.calib["right_neck"]["qw"]]).as_matrix()
                right_world2target[:3,:3] = r
                right_world2target[:3, 3] = [self.calib["right_neck"]["x"], self.calib["right_neck"]["y"],self.calib["right_neck"]["z"]]
                right_pose = self.right_poses.pop(0)
                right_pos = np.eye(4)
                right_rotation = Rotation.from_quat([right_pose[3],right_pose[4],right_pose[5],right_pose[6]]).as_matrix()
                right_position = [right_pose[0], right_pose[1],right_pose[2]]
                right_pos[:3, :3] = right_rotation
                right_pos[:3, 3] = right_position
                right_fit_pos = right_world2target @ right_pos
                right_q = Rotation.from_matrix(right_fit_pos[:3,:3]).as_quat()
                right_msg = PoseStamped()
                right_msg.header.frame_id = "target"
                right_msg.pose.position.x = right_fit_pos[0,3]
                right_msg.pose.position.y = right_fit_pos[1,3]
                right_msg.pose.position.z = right_fit_pos[2,3]
                right_msg.pose.orientation.x = right_q[0]
                right_msg.pose.orientation.y = right_q[1]
                right_msg.pose.orientation.z = right_q[2]
                right_msg.pose.orientation.w = right_q[3]
                self.right_pose_pub.publish(right_msg)


                # self.get_logger().info("publish")
                #
            


    def destroy_node(self):
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = DualInfraredTrackerCalib()
    node.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
