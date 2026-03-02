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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_DELAUNATOR_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_DELAUNATOR_H

#include "rally/utils/utils.h"
#include "rally/core/meta/meta.h"
#include "rally/core/containers/short_array.h"

namespace rally {

class Delaunator {
public:
    using Ptr = std::shared_ptr<Delaunator>;

    Delaunator() : vertices_{}, triangles_{}, half_edges_{}, hull_prev_{}, hull_next_{}, hull_tri_{}, m_hash_{} {}

    void setVertices(const std::vector<ShortArray2f> &vertices) { vertices_ = vertices; }

    void triangulate();

    float getHullArea();

    std::vector<std::size_t> getTriAngles() { return triangles_; }

protected:
    std::size_t hash_key(const ShortArray2f &pt) const;

    std::size_t add_triangle(std::size_t i0, std::size_t i1, std::size_t i2, std::size_t a, std::size_t b,
                             std::size_t c);

    void link(const std::size_t a, const std::size_t b);

    std::size_t legalize(std::size_t a);

    std::vector<ShortArray2f> vertices_;
    std::vector<std::size_t> triangles_;
    std::vector<std::size_t> half_edges_;
    std::vector<std::size_t> hull_prev_;
    std::vector<std::size_t> hull_next_;
    std::vector<std::size_t> hull_tri_;
    std::size_t hull_start_{0};
    ShortArray2f m_center_;
    std::vector<std::size_t> m_hash_;
};

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_DELAUNATOR_H
