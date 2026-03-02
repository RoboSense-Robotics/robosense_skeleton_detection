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

#include "rally/common/algo/geo_algo.h"

namespace rally {

double pointToLine(const Eigen::Vector2d &pt,
                   const Eigen::Vector2d &line_pt1,
                   const Eigen::Vector2d &line_pt2) {
    double A = line_pt1.y() - line_pt2.y();
    double B = line_pt2.x() - line_pt1.x();
    double C = line_pt1.x() * line_pt2.y() - line_pt1.y() * line_pt2.x();

    double distance = std::abs(A * pt.x() + B * pt.y() + C) / std::sqrt(A * A + B * B);
    return distance;
}

Eigen::Vector2d projPointToLine(const Eigen::Vector2d &pt,
                                const Eigen::Vector2d &line_pt1,
                                const Eigen::Vector2d &line_pt2) {
    Eigen::Vector2d line_dir = line_pt1 - line_pt2;
    double k = (pt.dot(line_dir) - line_pt1.dot(line_dir)) / line_dir.dot(line_dir);
    Eigen::Vector2d res_pt = line_pt1 + k * line_dir;
    return res_pt;
}

}  // namespace rally
