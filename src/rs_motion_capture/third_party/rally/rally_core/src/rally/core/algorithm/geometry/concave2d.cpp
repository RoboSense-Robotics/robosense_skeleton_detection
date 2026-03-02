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

#include "rally/core/algorithm/geometry/concave2d.h"

namespace rally {

inline size_t next_halfedge(size_t e) {
    return (e % 3 == 2) ? e - 2 : e + 1;
}

inline size_t prev_halfedge(size_t e) {
    return (e % 3 == 0) ? e + 2 : e - 1;
}

std::vector<std::size_t> Concave2D::concaveHull(const std::vector<ShortArray2f>& vertices, float factor) {
    RENSURE(factor > 0.f && factor < 1.f);

    setVertices(vertices);
    triangulate();

    // Determine initial points on outside hull
    std::vector<size_t> bpoints = getHullPointsIdx();
    std::set<size_t> bset(bpoints.begin(), bpoints.end());

    // Make max heap of boundary edges with lengths
    using hpair = std::pair<size_t, float>;

    auto cmp = [](hpair left, hpair right) {
        return left.second < right.second;
    };

    std::vector<hpair> bheap(bpoints.size());

    float max_len = std::numeric_limits<float>::min();
    float min_len = std::numeric_limits<float>::max();

    for (auto point : bpoints) {
        const auto e = hull_tri_[point];
        const auto len = getEdgeLenth(e);

        bheap.push_back({e, len});
        std::push_heap(bheap.begin(), bheap.end(), cmp);

        min_len = std::min(len, min_len);
        max_len = std::max(len, max_len);
    }

    // Determine length parameter
    float length_param = factor * max_len + (1.f - factor) * min_len;

    // Iteratively add points to boundary by iterating over the triangles on the hull
    while (!bheap.empty()) {

        // Get edge with the largest length
        std::pop_heap(bheap.begin(), bheap.end(), cmp);
        const auto res = bheap.back();
        bheap.pop_back();

        // Length of edge too small for our chi factor
        if (res.second <= length_param) {
            break;
        }

        // Find interior point given edge e (a -> b)
        //       e
        //  b <----- a
        //     \   /
        //  e_b \ / e_a
        //       c
        size_t c = getInteriorPoint(res.first);

        // Point already belongs to boundary
        if (bset.count(c)) {
            continue;
        }

        // Get two edges connected to interior point
        //  c -> b
        size_t e_b = half_edges_[next_halfedge(res.first)];
        //  a -> c
        size_t e_a = half_edges_[next_halfedge(next_halfedge(res.first))];

        // Add edges to heap
        const auto len_a = getEdgeLenth(e_a);
        const auto len_b = getEdgeLenth(e_b);

        bheap.push_back({e_a, len_a});
        std::push_heap(bheap.begin(), bheap.end(), cmp);
        bheap.push_back({e_b, len_b});
        std::push_heap(bheap.begin(), bheap.end(), cmp);

        // Update outer hull and connect new edges
        size_t a = triangles_[res.first];
        size_t b = triangles_[next_halfedge(res.first)];

        hull_next_[c] = b;
        hull_prev_[c] = a;
        hull_next_[a] = hull_prev_[b] = c;

        bset.insert(c);
    }

    return getHullPointsIdx();
}

std::vector<std::size_t> Concave2D::getHullPointsIdx() {
    std::vector<size_t> hull_pts;

    size_t point = hull_start_;
    do {
        hull_pts.push_back(point);
        point = hull_next_[point];
    }
    while (point != hull_start_);

    // Wrap back around
    hull_pts.push_back(hull_start_);

    return hull_pts;
}

float Concave2D::getEdgeLenth(std::size_t e_a) {
    size_t e_b = next_halfedge(e_a);

    const auto a = vertices_[triangles_[e_a]];
    const auto b = vertices_[triangles_[e_b]];

    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

size_t Concave2D::getInteriorPoint(size_t e) { return triangles_[next_halfedge(next_halfedge(e))]; }

}  // namespace rally
