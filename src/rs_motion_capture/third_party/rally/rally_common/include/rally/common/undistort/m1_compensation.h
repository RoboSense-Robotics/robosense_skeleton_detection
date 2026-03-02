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
#ifndef RALLY_COMMON_UNDISTORT_M1_COMPENSATION_H
#define RALLY_COMMON_UNDISTORT_M1_COMPENSATION_H

#include "rally/common/basic_ops/basic_ops.h"
#include "rally/common/basic_type/basic_type.h"
#include "rally/common/refine_box/refine_box.h"

namespace rally {

struct M1CompensationOptions {
    double wheel_compensation = 0.3;
    double tail_compensation = 0.1;
    double size_compensation_factor = 0.05;
    double wheel_compensation_range = 30.;

    double vel_dis_compensation_thre = .5;
};

class M1Compensation {
public:
    using Ptr = std::shared_ptr<M1Compensation>;

    pcl::PointCloud<RsPoint>::Ptr ruby_crop_cloud_ptr_;

    M1Compensation(const M1CompensationOptions &options) {
        options_ = options;
        {
            SimpleRefineBoxOptions refine_box_options;
            refine_box_options.vel_dis_compensation_thre = options_.vel_dis_compensation_thre;
            refine_box_ptr_.reset(new SimpleRefineBox(refine_box_options));
        }

        m1_static_compensate_cloud_ptr_.reset(new pcl::PointCloud<RsPoint>);
    }

    ///@brief 通过ruby的打标框，对M1P的打标框进行补偿
    ///@param [in] frame_3d 需要用到在车体坐标系下的 pre_odom, odom, object
    ///@param [in] ruby_raw_in_vehicle_ptr,m1p_raw_in_vehicle_ptr,distorted_time ruby在车体坐标系下的点云，M1P在车体坐标系下的点云，以及补偿时间
    ///@param [out] compensate_frame_3d,valid_box_vec 补偿之后的目标以及有用目标的标记
    void compensate(const std::vector<Object3D> &frame_3d, const Odom &pre_odom, const Odom &cur_odom,
                    const pcl::PointCloud<RsPoint>::Ptr &ruby_raw_in_vehicle_ptr,
                    const pcl::PointCloud<RsPoint>::Ptr &m1p_raw_in_vehicle_ptr, double distorted_time,
                    std::vector<Object3D> &compensate_frame_3d, std::vector<bool> &valid_box_vec);

    pcl::PointCloud<RsPoint>::Ptr getM1StaticCompensateCloud() {
        return m1_static_compensate_cloud_ptr_;
    }

    std::vector<Object3D> getRubyStaticCompensateFrame() {
        return ruby_static_compensate_frame_3d_;
    }

    std::vector<Object3D> getRubyDynamicCompensateFrame() {
        return ruby_dynamic_compensate_frame_3d_;
    }

    std::vector<Object3D> getM1WheelCompensateFrame() {
        return m1_wheel_compensate_frame_3d_;
    }

    std::vector<Object3D> getM1DynamicCompensateFrame() {
        return m1_dynamic_compensate_frame_3d_;
    }

    std::vector<std::vector<int> > getM1InstanceIdxVec() {
        return m1_instance_idx_vec_;
    }

private:
    std::string name() {
        return "M1Compensation";
    }

    M1CompensationOptions options_;
    SimpleRefineBox::Ptr refine_box_ptr_;
    pcl::PointCloud<RsPoint>::Ptr m1_static_compensate_cloud_ptr_;

    std::vector<Object3D> ruby_static_compensate_frame_3d_;
    std::vector<Object3D> ruby_dynamic_compensate_frame_3d_;
    std::vector<Object3D> m1_wheel_compensate_frame_3d_;
    std::vector<Object3D> m1_dynamic_compensate_frame_3d_;
    std::vector<std::vector<int> > m1_instance_idx_vec_;
};

}  // namespace rally

#endif  // RALLY_COMMON_UNDISTORT_M1_COMPENSATION_H
