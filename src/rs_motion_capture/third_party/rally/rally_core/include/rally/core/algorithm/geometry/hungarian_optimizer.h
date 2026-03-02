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

#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_HUNGARIAN_OPTIMIZER_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_HUNGARIAN_OPTIMIZER_H

#include <vector>
#include <algorithm>
#include <deque>
#include <cassert>
#include <memory>
#include <cstdint>
#include <limits>

namespace rally {

struct WeightedBipartiteEdge {
    std::uint32_t left;
    std::uint32_t right;
    float cost;

    WeightedBipartiteEdge() : left(), right(), cost() {}

    WeightedBipartiteEdge(std::uint32_t left, std::uint32_t right, float cost) : left(left), right(right), cost(cost) {}
};

struct LeftEdge {
    std::uint32_t right;
    float cost;

    LeftEdge() : right(), cost() {}

    LeftEdge(std::uint32_t right, float cost) : right(right), cost(cost) {}

    bool operator<(LeftEdge const &otherEdge) const {
        return right < otherEdge.right || (right == otherEdge.right && cost < otherEdge.cost);
    }
};

class HungarianGraphOptimizer {
public:
    /// @brief hungarian minimum-weight perfect matching on a weighted bipartite graph
    /// @param n number of nodes on each side (Note: Edges with endpoints out of the range [0, n) are ignored.)
    /// @param allEdges weighted edges of the bipartite graph
    /// @return a minimum-weight perfect matching.
    ///         If a matching is found, returns a length-n vector, giving the nodes on the right
    ///         that the left nodes are matched to.
    ///         If no matching exists, returns an empty vector.
    std::vector<uint32_t> hungarianMinimumWeightPerfectMatching(std::uint32_t n,
                                                                std::vector<WeightedBipartiteEdge> const &allEdges);

private:
    /// @brief indicator for unmatched nodes
    const std::uint32_t unmatched = std::numeric_limits<std::uint32_t>::max();
};

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_HUNGARIAN_OPTIMIZER_H
