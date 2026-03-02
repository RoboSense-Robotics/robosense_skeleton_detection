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
#ifndef RALLY_COMMON_UNDISTORT_STATIC_UNDITORT_H
#define RALLY_COMMON_UNDISTORT_STATIC_UNDITORT_H

#include <pcl/point_cloud.h>
#include "rally/common/undistort/pose_interp.h"
#include "rally/common/basic_type/point_type.h"
#include "rally/common/basic_type/rotated_box.h"

namespace rally {

///@brief 对点云进行静态补偿
///@param[in,out] cloud_ptr 输入和输出，带时间戳的点云
///@param[in] pre_odom,cur_odom 两个时间点上的odom
///@param[in] undistort_time 希望将点云补偿到的一个时间戳
Eigen::Affine3d pointsStaticUndistort(const pcl::PointCloud<RsPoint>::Ptr &cloud_ptr,
                                      const Odom &pre_odom,
                                      const Odom &cur_odom,
                                      double undistort_time);


///@brief 对点云的box进行静态补偿
///@param[in] ruby_cloud_ptr 在车体坐标系下，没有进行静态补偿的点云
///@param[in] instance_idx,pre_odom,cur_odom,in_box,compensate_time 输入信息
///@param[out] out_box 静态补偿之后的框
void boxStaticCompensation(const pcl::PointCloud<RsPoint>::Ptr &cloud_ptr,
                            const rally::Odom &pre_odom, const rally::Odom &cur_odom,
                            const std::vector<int> &instance_idx,
                            const RotatedBox &in_box, const double &compensate_time, RotatedBox &out_box);

}  // namespace rally

#endif  // RALLY_COMMON_UNDISTORT_STATIC_UNDITORT_H
