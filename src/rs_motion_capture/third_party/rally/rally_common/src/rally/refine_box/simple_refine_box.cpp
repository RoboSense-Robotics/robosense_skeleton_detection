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

#include "rally/common/refine_box/simple_refine_box.h"
#include "rally/common/basic_ops/basic_ops.h"
#include "rally/common/algo/algo.h"

namespace rally {

void SimpleRefineBox::refine(const pcl::PointCloud<RsPoint>::Ptr &instance_cloud_ptr, const Eigen::Vector3d &vel,
                             const RotatedBox &compensate_box, const RotatedBox &in_box, RotatedBox &out_box) {
    out_box = in_box;
    if (instance_cloud_ptr == nullptr) {
        RWARN << name() << ": instance_cloud_ptr is null!";
        return;
    }
    if (instance_cloud_ptr->empty()) {
        RWARN << name() << ": instance_cloud_ptr is empty!";
        return;
    }
    bool is_same_direction_with_self(false);
    RsPoint tail_pt, head_pt;

    {  // step1. 根据扩展框，计算到实例分割点云的车头点和车尾点
        const auto &compensate_corners = compensate_box.getAllCorners();
        Eigen::Vector2d compensate_head_line_pt1, compensate_head_line_pt2;
        Eigen::Vector2d compensate_tail_line_pt1, compensate_tail_line_pt2;

        {
            if (vel.x() > 0) {
                is_same_direction_with_self = true;
            }
            compensate_tail_line_pt1 = compensate_corners[1].head(2);
            compensate_tail_line_pt2 = compensate_corners[2].head(2);
            compensate_head_line_pt1 = compensate_corners[0].head(2);
            compensate_head_line_pt2 = compensate_corners[3].head(2);
        }

        double min_tail_pt_to_tail_line(std::numeric_limits<double>::max());
        double min_head_pt_to_head_line(std::numeric_limits<double>::max());


        for (size_t j = 0; j < instance_cloud_ptr->size(); ++j) {
            const auto &pt = instance_cloud_ptr->points[j];
            Eigen::Vector2d tmp_e_pt(pt.x, pt.y);
            const auto &pt_to_tail_dis = pointToLine(tmp_e_pt, compensate_tail_line_pt1, compensate_tail_line_pt2);
            const auto &pt_to_head_dis = pointToLine(tmp_e_pt, compensate_head_line_pt1, compensate_head_line_pt2);

            if (pt_to_tail_dis < min_tail_pt_to_tail_line) {
                tail_pt = pt;
                min_tail_pt_to_tail_line = pt_to_tail_dis;
            }
            if (pt_to_head_dis < min_head_pt_to_head_line) {
                head_pt = pt;
                min_head_pt_to_head_line = pt_to_head_dis;
            }
        }
    }

    double tail_dis{0.};
    double head_dis{0.};
    Eigen::Vector2d tail_move;
    Eigen::Vector2d head_move;
    {   // step2. 根据实际的框，计算车头点和车尾点分别距离实际框车头车尾的距离和向量
        Eigen::Vector2d in_head_line_pt1, in_head_line_pt2;
        Eigen::Vector2d in_tail_line_pt1, in_tail_line_pt2;
        const auto &in_corners = in_box.getAllCorners();
        in_tail_line_pt1 = in_corners[1].head(2);
        in_tail_line_pt2 = in_corners[2].head(2);
        in_head_line_pt1 = in_corners[0].head(2);
        in_head_line_pt2 = in_corners[3].head(2);

        Eigen::Vector2d tmp_tail_pt(tail_pt.x, tail_pt.y);
        tail_dis = pointToLine(tmp_tail_pt, in_tail_line_pt1, in_tail_line_pt2);
        Eigen::Vector2d proj_tail_pt = projPointToLine(tmp_tail_pt, in_tail_line_pt1, in_tail_line_pt2);
        Eigen::Vector2d tmp_head_pt(head_pt.x, head_pt.y);
        head_dis = pointToLine(tmp_head_pt, in_head_line_pt1, in_head_line_pt2);
        Eigen::Vector2d proj_head_pt = projPointToLine(tmp_head_pt, in_head_line_pt1, in_head_line_pt2);

        tail_move = tmp_tail_pt - proj_tail_pt;
        head_move = tmp_head_pt - proj_head_pt;
    }
    {  // step3. 修框
        auto center = in_box.getCenter();
        const auto &size = in_box.getSize();
        const auto &yaw = in_box.getYaw();

        if (tail_dis < options_.vel_dis_compensation_thre && head_dis < options_.vel_dis_compensation_thre) {
            if (is_same_direction_with_self) {
                center.x() += tail_move.x();
                center.y() += tail_move.y();
            } else {
                center.x() += head_move.x();
                center.y() += head_move.y();
            }
        } else {
            if (tail_dis < head_dis) {
                center.x() += tail_move.x();
                center.y() += tail_move.y();
            } else {
                center.x() += head_move.x();
                center.y() += head_move.y();
            }
        }
        out_box = RotatedBox(center, size, yaw);
    }
}

}  // namespace rally
