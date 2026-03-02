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

#include "rally/common/basic_type/rotated_box_info.h"

namespace rally {

RotatedBoxInfo getBoxInfos(const RotatedBox &rotated_box) {
    RotatedBoxInfo res_info;
    const auto &corners = rotated_box.getAllCorners();
    std::array<double, 4> dis_array;
    std::array<double, 4> ang_array;
    double min_ang = std::numeric_limits<double>::max();
    double max_ang = std::numeric_limits<double>::min();

    for (int i = 0; i < 4; ++i) {
        const auto &corner = corners[i];
        dis_array[i] = std::sqrt(corner.x() * corner.x() + corner.y() * corner.y());
        ang_array[i] = std::atan2(corner.y(), corner.x());  // the value is [-pi, pi]
        min_ang = std::min(min_ang, ang_array[i]);
        max_ang = std::max(max_ang, ang_array[i]);
        res_info.corners[i] = Eigen::Vector2d(corner.x(), corner.y());
    }

    // if the box cross the zero ang line, if cross, need to fix the ang_array
    if ((max_ang - min_ang) > M_PI_2) {
        for (int i = 0; i < 4; ++i) {
            auto &ang = ang_array[i];
            if (ang < 0) {
                ang += M_PI * 2.;
            }
        }
    }

    auto checkIdxFunc = [](int idx) -> int {
        while (idx < 0) {
            idx += 4;
        }
        while (idx > 3) {
            idx -= 4;
        }
        return idx;
    };

    int min_dis_idx = 0;
    float min_dis = dis_array[0];
    for (int i = 1; i < 4; ++i) {
        const auto &dis = dis_array[i];
        if (min_dis > dis) {
            min_dis = dis;
            min_dis_idx = i;
        }
    }

    int pre_idx = checkIdxFunc(min_dis_idx - 1);
    int nex_idx = checkIdxFunc(min_dis_idx + 1);

    const auto &min_dis_ang = ang_array[min_dis_idx];
    const auto &pre_ang = ang_array[pre_idx];
    const auto &nex_ang = ang_array[nex_idx];

    res_info.nearest_idx = min_dis_idx;
    if (nex_ang < min_dis_ang && pre_ang > min_dis_ang) {
        res_info.type = RotatedBboxType::THREE_PTS_FACE;
        if (min_dis_idx == 0 || min_dis_idx == 2) {
            res_info.three_face_infos.major_dir_line_pair.first = min_dis_idx;
            res_info.three_face_infos.major_dir_line_pair.second = nex_idx;

            res_info.three_face_infos.minor_dir_line_pair.first = min_dis_idx;
            res_info.three_face_infos.minor_dir_line_pair.second = pre_idx;

            res_info.pre_along_line_pair.first = min_dis_idx;
            res_info.pre_along_line_pair.second = pre_idx;

            res_info.nex_along_line_pair.first = nex_idx;
            res_info.nex_along_line_pair.second = checkIdxFunc(nex_idx + 1);
        } else {
            res_info.three_face_infos.major_dir_line_pair.first = min_dis_idx;
            res_info.three_face_infos.major_dir_line_pair.second = pre_idx;

            res_info.three_face_infos.minor_dir_line_pair.first = min_dis_idx;
            res_info.three_face_infos.minor_dir_line_pair.second = nex_idx;

            res_info.pre_along_line_pair.first = pre_idx;
            res_info.pre_along_line_pair.second = checkIdxFunc(pre_idx - 1);

            res_info.nex_along_line_pair.first = min_dis_idx;
            res_info.nex_along_line_pair.second = nex_idx;
        }
    } else {
        res_info.type = RotatedBboxType::TWO_PTS_FACE;
        if (nex_ang > min_dis_ang) {
            res_info.two_face_infos.face_line_pair.first = min_dis_idx;
            res_info.two_face_infos.face_line_pair.second = pre_idx;

            res_info.pre_along_line_pair.first = pre_idx;
            res_info.pre_along_line_pair.second = checkIdxFunc(pre_idx - 1);

            res_info.nex_along_line_pair.first = min_dis_idx;
            res_info.nex_along_line_pair.second = nex_idx;
        } else {
            res_info.two_face_infos.face_line_pair.first = min_dis_idx;
            res_info.two_face_infos.face_line_pair.second = nex_idx;

            res_info.pre_along_line_pair.first = min_dis_idx;
            res_info.pre_along_line_pair.second = pre_idx;

            res_info.nex_along_line_pair.first = nex_idx;
            res_info.nex_along_line_pair.second = checkIdxFunc(nex_idx + 1);
        }
    }

    return res_info;
}

}  // namespace rally
