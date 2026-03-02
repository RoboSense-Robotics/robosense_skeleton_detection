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
#ifndef RALLY_COMMON_BASIC_TYPE_ROTATED_BOX_INFO_H
#define RALLY_COMMON_BASIC_TYPE_ROTATED_BOX_INFO_H

#include "rally/common/basic_type/rotated_box.h"

namespace rally {

///@brief 我们的旋转框是绕框的中心，逆时针旋转。我们的框的第一个点，是朝向的左上的点，
/// 第二个点是朝向的左下的点，第三个点是朝向的右下，第四个点是右上
// TWO_PTS_FACE 表示有两点可见
// THREE_PTS_FACE 表示有三点可见
enum class RotatedBboxType {
    TWO_PTS_FACE = 0,
    THREE_PTS_FACE = 1,
};

///@brief RotatedBoxInfo 是对RotatedBox的一些几何性质的计算
struct RotatedBoxInfo {
    ///@brief 说明RotatedBox相对与自车来说，是三点可见还是两点可见
    RotatedBboxType type;

    ///@brief 如果是三点可见，则说明长边点对和短边点对分别是什么
    struct ThreePtsFaceInfo {
        std::pair<int, int> major_dir_line_pair;
        std::pair<int, int> minor_dir_line_pair;
    };
    ///@brief 如果是两点可见，则说明面对自车的点对是什么
    struct TwoPtsFaceInfo {
        std::pair<int, int> face_line_pair;
    };

    std::array<Eigen::Vector2d, 4> corners;
    int nearest_idx;
    ThreePtsFaceInfo three_face_infos;
    TwoPtsFaceInfo two_face_infos;
    ///@brief 无论是三点可见还是两点可见，都要说明顺着他车的rotated_box的角点顺序，pre指的是主方向的前一个点对，nex指的是主方向的后一个点对
    /// 通过pre_along_line_pair.second()，pre_along_line_pair.first()，nex_along_line_pair.first()，nex_along_line_pair.second()可以构成一个旋转框的四个点的索引
    /// 并且第二个和第三个点表示面向自车的线，第一个点和第四个点分别在两边
    std::pair<int, int> pre_along_line_pair;
    std::pair<int, int> nex_along_line_pair;
};

///@brief 通过旋转框，获得框相对于自车的信息
RotatedBoxInfo getBoxInfos(const RotatedBox &rotated_box);

}  // namespace rally

#endif  // RALLY_COMMON_BASIC_TYPE_ROTATED_BOX_INFO_H
