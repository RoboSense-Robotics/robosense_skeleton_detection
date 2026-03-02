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
#ifndef RALLY_COMMON_BASIC_TYPE_ROTATED_BOX_H
#define RALLY_COMMON_BASIC_TYPE_ROTATED_BOX_H

#include "rally/utils/utils.h"
#include <Eigen/Dense>

namespace rally {

class RotatedBox {
public:
    RotatedBox();

    RotatedBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size, float yaw);

    Eigen::Vector3d getDirection() const;

    Eigen::Vector3d getCenter() const { return center_; }

    Eigen::Vector3d getSize() const { return size_; }

    ///@brief yaw值的范围在(-M_PI, M_PI]
    double getYaw() const { return yaw_; }

    ///@brief 旋转框需要满足长>宽，且朝向与长边平行
    void checkBox();

    ///@brief 旋转框需要满足朝向与速度的方向成锐角
    void fixDirection(const Eigen::Vector3d &vel);

    ///@brief 获取旋转框的所有角点，1）角点顺序为沿着旋转框中心逆时针；2）第一个点在朝向的左上点；3）前四个点在下面，后四个点在上面，第0个点和第4个点一一对应
    std::vector<Eigen::Vector3d> getAllCorners() const;

    ///@brief 通过8个角点，可以反推出旋转框
    void fromCorners(const std::vector<Eigen::Vector3d> &corners);

    void transform(const Eigen::Matrix4d &t_mat);

private:
    Eigen::Vector3d center_;
    Eigen::Vector3d size_;
    double yaw_;
};

inline std::ostream &operator<<(std::ostream &os, const RotatedBox &p) {
    os << "RotatedBox: " << std::endl;
    os << "{center: " << p.getCenter().transpose() <<
       ", size: " << p.getSize().transpose() <<
       ", yaw " << std::to_string(p.getYaw()) << std::endl;
    os << "}";
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_BASIC_TYPE_ROTATED_BOX_H
