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
#ifndef RALLY_COMMON_UNDISTORT_POINTS_UNDISTORT_H
#define RALLY_COMMON_UNDISTORT_POINTS_UNDISTORT_H

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "rally/common/basic_type/basic_type.h"

namespace rally {

class PointsUndistort {
public:
    ///@brief 对点云进行静态补偿
    ///@param[in,out] cloud_ptr 输入和输出，带时间戳的点云
    ///@param[in] pre_odom,cur_odom 两个时间点上的odom
    ///@param[in] undistort_time 希望将点云补偿到的一个时间戳
    static Eigen::Affine3d staticUndistort(const pcl::PointCloud<RsPoint>::Ptr &cloud_ptr,
                                           const Odom &pre_odom, const Odom &cur_odom, double undistort_time);

    ///@brief 对目标点进行动态补偿
    ///@param[in] corners,timestamp,vel 输入相关的点，对应点的时间戳，以及速度，注意所有的值都是在车体坐标系下的
    ///@param[in] g_pose,undistort_time 输入在全局坐标以及需要补偿的时间
    ///@param[out] corners 输出动态补偿之后的点，动态补偿之后的点依然在车坐标系下
    static void dynamicUndistort(std::vector<Eigen::Vector3d> &corners, double timestamp,
                                 const Eigen::Affine3d &g_pose, const Eigen::Vector3d &vel, double undistort_time);

    static Eigen::Affine3d poseInterp(double t, Odom const &aff1, Odom const &aff2);
};

}  // namespace rally

#endif  // RALLY_COMMON_UNDISTORT_POINTS_UNDISTORT_H
