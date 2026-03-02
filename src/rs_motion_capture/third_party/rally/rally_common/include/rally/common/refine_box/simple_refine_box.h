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
#ifndef RALLY_COMMON_REFINE_BOX_SIMPLE_REFINE_BOX_H
#define RALLY_COMMON_REFINE_BOX_SIMPLE_REFINE_BOX_H

#include <pcl/point_cloud.h>
#include "rally/utils/utils.h"
#include "rally/common/basic_type/basic_type.h"

namespace rally {

///@brief 算法的参数
///@param vel_dis_compensation_thre 沿着速度方向的一个距离阈值
struct SimpleRefineBoxOptions {
    double vel_dis_compensation_thre = 0.5;
};

class SimpleRefineBox {
public:
    using Ptr = std::shared_ptr<SimpleRefineBox>;

    SimpleRefineBox(const SimpleRefineBoxOptions &options) {
        options_ = options;
    }

    ///@brief 将旋转框根据目标的实例点云，沿着速度方向，拉到与点云完全贴合的算法。
    /// 计算框的车头和车尾距离点云的最小距离，如果两个距离有一个大于了阈值，则车框移动最小的距离，如果两个距离都小于阈值，则车框沿着车速方向移动
    ///@param[in] instance_cloud_ptr, 目标的实例分割点云，车体坐标系下
    ///@param[in] vel, 目标的车体坐标系下的速度，注意是车体坐标系下的车速
    ///@param[in] compensate_box, 目标的扩展框，需要和实际框的朝向相同并且能够完整的crop目标实例分割点云出来
    ///@param[in] in_box, 目标的实际框，车体坐标系下
    ///@param[out] out_box, 目标的修复之后的框，车体坐标系下
    void refine(const pcl::PointCloud<RsPoint>::Ptr &instance_cloud_ptr,const Eigen::Vector3d &vel,
                const RotatedBox &compensate_box, const RotatedBox &in_box, RotatedBox &out_box);

private:
    std::string name() {
        return "SimpleRefineBox";
    }

    SimpleRefineBoxOptions options_;
};

}  // namespace rally

#endif  // RALLY_COMMON_REFINE_BOX_SIMPLE_REFINE_BOX_H
