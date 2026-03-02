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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_CONCAVE2D_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_CONCAVE2D_H

#include "rally/core/algorithm/geometry/delaunator.h"

namespace rally {

class Concave2D: public Delaunator {
public:
    using Ptr = std::shared_ptr<Concave2D>;

    std::vector<std::size_t> concaveHull(const std::vector<ShortArray2f>& vertices, float factor = 0.1);

    template <typename PointT>
    std::vector<std::size_t> concaveHull(const std::vector<PointT>& vertices, float factor = 0.1) {
        std::vector<ShortArray2f> pts(vertices.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            pts[i].x = vertices[i].x;
            pts[i].y = vertices[i].y;
        }
        return concaveHull(pts, factor);
    }

private:
    std::vector<std::size_t> getHullPointsIdx();

    float getEdgeLenth(std::size_t e_a);

    size_t getInteriorPoint(size_t e);
};

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_CONCAVE2D_H
