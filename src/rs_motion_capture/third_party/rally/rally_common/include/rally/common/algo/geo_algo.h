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
#ifndef RALLY_ALGO_GEO_ALGO_H
#define RALLY_ALGO_GEO_ALGO_H

#include <Eigen/Dense>

namespace rally {

// https://blog.csdn.net/zisuina_2/article/details/105855376
///@brief 计算点pt到直线的距离，直线为line_pt1和line_pt2组成
///@param[in] pt 计算点pt到直线的距离
///@param[in] line_pt1,line_pt2 两点组成一条直线
///@return 返回点到直线的距离
double pointToLine(const Eigen::Vector2d &pt,
                   const Eigen::Vector2d &line_pt1,
                   const Eigen::Vector2d &line_pt2);

///@brief 将一个点投影到一条直线上，得到投影点
///@param[in] pt 将点pt投影到直线上
///@param[in] line_pt1,line_pt2 两点组成一条直线
///@return 返回投影点
Eigen::Vector2d projPointToLine(const Eigen::Vector2d &pt,
                                const Eigen::Vector2d &line_pt1,
                                const Eigen::Vector2d &line_pt2);

}  // namespace rally

#endif  // RALLY_ALGO_GEO_ALGO_H
