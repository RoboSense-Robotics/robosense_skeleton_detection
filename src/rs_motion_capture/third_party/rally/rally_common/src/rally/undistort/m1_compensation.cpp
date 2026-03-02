/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "rally/common/undistort/m1_compensation.h"
#include "rally/common/undistort/static_unditort.h"
#include "rally/common/undistort/dynamic_compensation.h"
#include "rally/common/algo/algo.h"

namespace rally {

void M1Compensation::compensate(const std::vector<Object3D> &frame_3d, const Odom &pre_odom, const Odom &cur_odom,
                                const pcl::PointCloud<RsPoint>::Ptr &ruby_raw_in_vehicle_ptr,
                                const pcl::PointCloud<RsPoint>::Ptr &m1p_raw_in_vehicle_ptr, double distorted_time,
                                std::vector<Object3D> &compensate_frame_3d, std::vector<bool> &valid_box_vec) {
    compensate_frame_3d = frame_3d;
    valid_box_vec.resize(frame_3d.size(), true);

    if (ruby_raw_in_vehicle_ptr == nullptr) {
        RWARN << name() << ": ruby_raw_in_vehicle_ptr is null!";
        return;
    }
    if (m1p_raw_in_vehicle_ptr == nullptr) {
        RWARN << name() << ": m1p_raw_in_vehicle_ptr is null!";
        return;
    }
    double cur_timestamp = cur_odom.timestamp;

    Eigen::Affine3d dst_pose;

    ruby_static_compensate_frame_3d_ = frame_3d;
    std::vector<double> ruby_box_vec_timestamp(frame_3d.size(), cur_timestamp);
    {
        // 第一步，对M1的点云做静态补偿，对ruby的框做静态补偿
        *m1_static_compensate_cloud_ptr_ = *m1p_raw_in_vehicle_ptr;
        dst_pose = pointsStaticUndistort(m1_static_compensate_cloud_ptr_, pre_odom, cur_odom, distorted_time);

        for (size_t i = 0; i < frame_3d.size(); ++i) {
            const auto &box = frame_3d[i].box;
            auto &out_box = ruby_static_compensate_frame_3d_[i].box;
            std::vector<int> instance_idx;
            cropInstanceFromRotatedBox(ruby_raw_in_vehicle_ptr, box, instance_idx);
            if (instance_idx.empty()) {
                valid_box_vec[i] = false;
                continue;
            }
            ruby_box_vec_timestamp[i] = ruby_raw_in_vehicle_ptr->points[instance_idx[0]].timestamp;

            boxStaticCompensation(ruby_raw_in_vehicle_ptr, pre_odom, cur_odom,
                                  instance_idx, box, distorted_time, out_box);
        }
    }

    ruby_dynamic_compensate_frame_3d_ = frame_3d;
    {
        // 第二步，对ruby的框做动态补偿
        for (size_t i = 0; i < valid_box_vec.size(); ++i) {
            if (!valid_box_vec[i]) {
                continue;
            }
            const auto &ruby_static_box = ruby_static_compensate_frame_3d_[i].box;
            const auto &vel = frame_3d[i].velocity;
            const auto &ruby_box_timestamp = ruby_box_vec_timestamp[i];
            auto &box = ruby_dynamic_compensate_frame_3d_[i].box;
            box = ruby_static_box;
            boxDynamicCompensation(box, ruby_box_timestamp, vel, dst_pose, distorted_time);
        }
    }

    m1_wheel_compensate_frame_3d_ = frame_3d;
    {
        // 第三步，生成M1的车速方向的补偿框，去除了车轮高度，并且在车速方向上做了延展的框，有助于crop出完整的M1的实例
        for (size_t i = 0; i < valid_box_vec.size(); ++i) {
            if (!valid_box_vec[i]) {
                continue;
            }

            const auto &ruby_static_box = ruby_static_compensate_frame_3d_[i].box;
            std::vector<Eigen::Vector3d> corners = ruby_static_box.getAllCorners();
            const auto &tmp_center = ruby_static_box.getCenter();
            if (std::abs(tmp_center.x()) < options_.wheel_compensation_range &&
                std::abs(tmp_center.y()) < options_.wheel_compensation_range) {
                for (int j = 0; j < 4; ++j) {
                    corners[j].z() += options_.wheel_compensation;
                }
            }
            auto &m1p_wheel_compensate_box = m1_wheel_compensate_frame_3d_[i].box;

            m1p_wheel_compensate_box.fromCorners(corners);
            auto size = m1p_wheel_compensate_box.getSize();
            const auto &center = m1p_wheel_compensate_box.getCenter();
            const auto &yaw = m1p_wheel_compensate_box.getYaw();
            const auto &vel = m1_wheel_compensate_frame_3d_[i].velocity;
            size.x() += 2 * options_.size_compensation_factor * vel.norm();
            m1p_wheel_compensate_box = RotatedBox(center, size, yaw);
        }
    }

    m1_instance_idx_vec_.clear();
    m1_instance_idx_vec_.resize(frame_3d.size());
    {
        // 第四步，将M1的实例crop出来
        for (size_t i = 0; i < valid_box_vec.size(); ++i) {
            if (!valid_box_vec[i]) {
                continue;
            }
            auto &instance_idx = m1_instance_idx_vec_[i];
            const auto &m1p_compensate_box = m1_wheel_compensate_frame_3d_[i].box;
            cropInstanceFromRotatedBox(m1_static_compensate_cloud_ptr_, m1p_compensate_box, instance_idx);
            if (instance_idx.empty()) {
                valid_box_vec[i] = false;
            }
        }
    }

    m1_dynamic_compensate_frame_3d_ = frame_3d;
    std::vector<int> is_same_direction_with_self_vec(frame_3d.size(), 0);
    {
        // 第五步，对M1的框进行动态补偿
        for (size_t i = 0; i < valid_box_vec.size(); ++i) {
            if (!valid_box_vec[i]) {
                continue;
            }
            const auto &vel = frame_3d[i].velocity;

            const auto &m1p_wheel_compensate_box = m1_wheel_compensate_frame_3d_[i].box;
            const auto &m1p_wheel_compensate_corners = m1p_wheel_compensate_box.getAllCorners();
            auto &is_same_direction_with_self = is_same_direction_with_self_vec[i];
            Eigen::Vector2d line_pt1, line_pt2;
            if (vel.x() > 0) {
                line_pt1 = m1p_wheel_compensate_corners[0].head(2);
                line_pt2 = m1p_wheel_compensate_corners[3].head(2);
                is_same_direction_with_self = 1;
            } else {
                line_pt1 = m1p_wheel_compensate_corners[1].head(2);
                line_pt2 = m1p_wheel_compensate_corners[2].head(2);
            }

            double min_pt_to_line_dis(std::numeric_limits<double>::max());
            std::vector<std::pair<int, double> > pt_idx_distance_pair_vec;
            const auto &instance_idx = m1_instance_idx_vec_[i];
            for (size_t j = 0; j < instance_idx.size(); ++j) {
                const auto &pt_idx = instance_idx[j];
                const auto &pt = m1_static_compensate_cloud_ptr_->points[pt_idx];
                Eigen::Vector2d tmp_e_pt(pt.x, pt.y);
                const auto &pt_to_line_dis = pointToLine(tmp_e_pt, line_pt1, line_pt2);
                min_pt_to_line_dis = std::min(min_pt_to_line_dis, pt_to_line_dis);
                std::pair<int, double> tmp_pair;
                tmp_pair.first = pt_idx;
                tmp_pair.second = pt_to_line_dis;
                pt_idx_distance_pair_vec.emplace_back(tmp_pair);
            }

            std::sort(pt_idx_distance_pair_vec.begin(), pt_idx_distance_pair_vec.end(),
                      [](const std::pair<int, double> &p1, const std::pair<int, double> &p2) {
                          return p1.second < p2.second;
                      });

            double min_time = std::numeric_limits<double>::max();
            double max_time = std::numeric_limits<double>::min();

            for (auto itr = pt_idx_distance_pair_vec.begin(); itr != pt_idx_distance_pair_vec.end(); ++itr) {
                if (itr->second > min_pt_to_line_dis + options_.tail_compensation) {
                    break;
                }
                const auto &tmp_timestamp = m1_static_compensate_cloud_ptr_->points[itr->first].timestamp;

                if (min_time > tmp_timestamp) {
                    min_time = tmp_timestamp;
                }
                if (max_time < tmp_timestamp) {
                    max_time = tmp_timestamp;
                }
            }

            double m1p_box_timestamp;
            if (is_same_direction_with_self) {
                m1p_box_timestamp = min_time;
            } else {
                m1p_box_timestamp = max_time;
            }

            {  // use timestamp to dynamic compensate m1p box
                const auto &in_box = ruby_dynamic_compensate_frame_3d_[i].box;
                auto &out_box = m1_dynamic_compensate_frame_3d_[i].box;
                out_box = in_box;
                boxDynamicCompensation(out_box, m1p_box_timestamp, -vel, dst_pose, distorted_time);
            }
        }
    }

    {  // 第六步，对M1的框进行微调
        for (size_t i = 0; i < valid_box_vec.size(); ++i) {
            if (!valid_box_vec[i]) {
                continue;
            }

            const auto &m1p_instance_idx = m1_instance_idx_vec_[i];
            pcl::PointCloud<RsPoint>::Ptr instance_cloud_ptr(new pcl::PointCloud<RsPoint>);
            instance_cloud_ptr->reserve(m1p_instance_idx.size());
            for (size_t j = 0; j < m1p_instance_idx.size(); ++j) {
                const auto &pt_idx = m1p_instance_idx[j];
                const auto &pt = m1_static_compensate_cloud_ptr_->points[pt_idx];
                instance_cloud_ptr->points.emplace_back(pt);
            }
            const auto &vel = m1_wheel_compensate_frame_3d_[i].velocity;
            const auto &m1p_wheel_compensate_box = m1_wheel_compensate_frame_3d_[i].box;
            const auto &in_box = m1_dynamic_compensate_frame_3d_[i].box;
            auto &out_box = compensate_frame_3d[i].box;

            refine_box_ptr_->refine(instance_cloud_ptr, vel, m1p_wheel_compensate_box, in_box, out_box);
        }
    }
}

}  // namespace rally
