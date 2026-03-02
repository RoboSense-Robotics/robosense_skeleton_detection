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

#include "rally/core/algorithm/geometry/hungarian_optimizer.h"

namespace rally {

std::vector<std::uint32_t> HungarianGraphOptimizer::hungarianMinimumWeightPerfectMatching(std::uint32_t n,
        std::vector<WeightedBipartiteEdge> const &allEdges) {
    // Edge lists for each left node.
    std::vector<std::vector<LeftEdge> > leftEdges(n);

    // region Edge list initialization

    // Initialize edge lists for each left node, based on the incoming set of edges.
    // While we're at it, we check that every node has at least one associated edge.
    // (Note: We filter out the edges that invalidly refer to a node on the left or right outside [0, n).)
    {
        std::vector<std::uint32_t> leftEdgeCounts(n, 0U);
        std::vector<std::uint32_t> rightEdgeCounts(n, 0U);

        for (auto const &edge : allEdges) {
            if (edge.left < n) {
                ++leftEdgeCounts[edge.left];
            }
            if (edge.right < n) {
                ++rightEdgeCounts[edge.right];
            }
        }

        for (std::uint32_t i{0U}; i < n; ++i) {
            if ((leftEdgeCounts[i] == 0U) || (rightEdgeCounts[i] == 0U)) {
                // No matching will be possible.
                return {};
            }
        }

        // Probably unnecessary, but reserve the required space for each node, just because?
        for (std::uint32_t i{0U}; i < n; ++i) {
            leftEdges[i].reserve(leftEdgeCounts[i]);
        }
    }
    // Actually add to the edge lists now.
    for (auto const &edge : allEdges) {
        if ((edge.left < n) && (edge.right < n)) {
            leftEdges[edge.left].push_back(LeftEdge(edge.right, edge.cost));
        }
    }

    // Sort the edge lists, and remove duplicate edges (keep the edge with smallest cost).
    for (std::uint32_t i{0U}; i < n; ++i) {
        std::vector<LeftEdge> &edges = leftEdges[i];
        std::sort(edges.begin(), edges.end());

        std::uint32_t edgeCount{0U};
        std::uint32_t lastRight{unmatched};
        for(std::uint32_t edgeIndex{0U}; edgeIndex < edges.size(); ++edgeIndex) {
            LeftEdge const &edge{edges[edgeIndex]};
            if (edge.right == lastRight) {
                continue;
            }
            lastRight = edge.right;
            if (edgeIndex != edgeCount) {
                edges[edgeCount] = edge;
            }
            ++edgeCount;
        }
        edges.resize(edgeCount);
    }

    // endregion Edge list initialization

    // These hold "potentials" for nodes on the left and nodes on the right, which reduce the costs of attached edges.
    // We maintain that every reduced cost, cost[i][j] - leftPotential[i] - leftPotential[j], is greater than zero.
    std::vector<float> leftPotential(n, 0.F);
    std::vector<float> rightPotential(n, std::numeric_limits<float>::max());

    // region Node potential initialization

    // Here, we seek good initial values for the node potentials.
    // Note: We're guaranteed by the above code that at every node on the left and right has at least one edge.

    // First, we raise the potentials on the left as high as we can for each node.
    // This guarantees each node on the left has at least one "tight" edge.

    for (std::uint32_t i{0U}; i < n; ++i) {
        std::vector<LeftEdge> const &edges{leftEdges[i]};
        float smallestEdgeCost{edges[0U].cost};
        for(std::uint32_t edgeIndex{1U}; edgeIndex < edges.size(); ++edgeIndex) {
            // when compare float point number, precision should be considered
            if ((edges[edgeIndex].cost - smallestEdgeCost) < std::numeric_limits<float>::epsilon()) {
                smallestEdgeCost = edges[edgeIndex].cost;
            }
        }

        // Set node potential to the smallest incident edge cost.
        // This is as high as we can take it without creating an edge with zero reduced cost.
        leftPotential[i] = smallestEdgeCost;
    }

    // Second, we raise the potentials on the right as high as we can for each node.
    // We do the same as with the left, but this time take into account that costs are reduced
    // by the left potentials.
    // This guarantees that each node on the right has at least one "tight" edge.
    for(auto const &edge : allEdges) {
        float const reducedCost{edge.cost - leftPotential[edge.left]};
        if (rightPotential[edge.right] > reducedCost) {
            rightPotential[edge.right] = reducedCost;
        }
    }

    // endregion Node potential initialization

    // Tracks how many edges for each left node are "tight".
    // Following initialization, we maintain the invariant that these are at the start of the node's edge list.
    std::vector<std::uint32_t> leftTightEdgesCount(n, 0U);

    // region Tight edge initialization

    // Here we find all tight edges, defined as edges that have zero reduced cost.
    // We will be interested in the subgraph induced by the tight edges, so we partition the edge lists for
    // each left node accordingly, moving the tight edges to the start.
    for(std::uint32_t i{0U}; i < n; ++i) {
        std::vector<LeftEdge> &edges{leftEdges[i]};
        std::uint32_t tightEdgeCount{0U};
        for (std::uint32_t edgeIndex{0U}; edgeIndex < edges.size(); ++edgeIndex) {
            LeftEdge const &edge{edges[edgeIndex]};
            float const reducedCost{edge.cost - leftPotential[i] - rightPotential[static_cast<std::uint32_t>(edge.right)]};
            if (std::abs(reducedCost) < std::numeric_limits<float>::epsilon()) {
                if (edgeIndex != tightEdgeCount) {
                    std::swap(edges[tightEdgeCount], edges[edgeIndex]);
                }
                ++tightEdgeCount;
            }
        }
        leftTightEdgesCount[i] = tightEdgeCount;
    }

    // endregion Tight edge initialization


    // Now we're ready to begin the inner loop.

    // We maintain an (initially empty) partial matching, in the subgraph of tight edges.
    std::uint32_t currentMatchingCardinality{0U};
    std::vector<std::uint32_t> leftMatchedTo(n, unmatched);
    std::vector<std::uint32_t> rightMatchedTo(n, unmatched);

    // region Initial matching (speedup?)

    // Because we can, let's make all the trivial matches we can.
    for(std::uint32_t i{0U}; i < n; ++i) {
        std::vector<LeftEdge> const &edges{leftEdges[i]};
        for(std::uint32_t edgeIndex{0U}; edgeIndex < leftTightEdgesCount[i]; ++edgeIndex) {
            std::uint32_t const j{static_cast<std::uint32_t>(edges[edgeIndex].right)};
            if (rightMatchedTo[j] == unmatched) {
                ++currentMatchingCardinality;
                rightMatchedTo[j] = i;
                leftMatchedTo[i] = j;
                break;
            }
        }
    }

    if (currentMatchingCardinality == n) {
        // Well, that's embarassing. We're already done!
        return leftMatchedTo;
    }

    // endregion Initial matching (speedup?)

    // While an augmenting path exists, we add it to the matching.
    // When an augmenting path doesn't exist, we update the potentials so that an edge between the area
    // we can reach and the unreachable nodes on the right becomes tight, giving us another edge to explore.
    //
    // We proceed in this fashion until we can't find more augmenting paths or add edges.
    // At that point, we either have a min-weight perfect matching, or no matching exists.

    // region Inner loop state variables

    // One point of confusion is that we're going to cache the edges between the area we've explored
    // that are "almost tight", or rather are the closest to being tight.
    // This is necessary to achieve our O(N^3) runtime.
    //
    // rightMinimumSlack[j] gives the smallest amount of "slack" for an unreached node j on the right,
    // considering the edges between j and some node on the left in our explored area.
    //
    // rightMinimumSlackLeftNode[j] gives the node i with the corresponding edge.
    // rightMinimumSlackEdgeIndex[j] gives the edge index for node i.
    std::vector<float> rightMinimumSlack(n, std::numeric_limits<float>::max());
    std::vector<std::uint32_t> rightMinimumSlackLeftNode(n, unmatched);
    std::vector<std::uint32_t> rightMinimumSlackEdgeIndex(n, 0U);
    std::vector<std::uint32_t> rightBacktrack(n, unmatched);
    std::vector<std::uint8_t> leftSeen(n, 0U);
    std::deque<std::uint32_t> leftNodeQueue;

    // Note: the above are all initialized at the start of the loop.

    // endregion Inner loop state variables

    while (currentMatchingCardinality < n) {
        // region Loop state initialization

        // Clear out slack caches.
        // Note: We need to clear the nodes so that we can notice when there aren't any edges available.
        rightMinimumSlack.assign(n, std::numeric_limits<float>::max());
        rightMinimumSlackLeftNode.assign(n, unmatched);
        // Mark everything "unseen".
        leftSeen.assign(n, 0U);
        rightBacktrack.assign(n, unmatched);
        // Clear the queue.
        leftNodeQueue.clear();

        // endregion Loop state initialization
        std::uint32_t startingLeftNode{unmatched};

        // region Find unmatched starting node

        // Find an unmatched left node to search outward from.
        // By heuristic, we pick the node with fewest tight edges, giving the BFS an easier time.
        // (The asymptotics don't care about this, but maybe it helps. Eh.)
        {
            std::uint32_t minimumTightEdges{std::numeric_limits<std::uint32_t>::max()};
            for(std::uint32_t i{0U}; i < n; ++i) {
                if ((leftMatchedTo[i] == unmatched) && (leftTightEdgesCount[i] < minimumTightEdges)) {
                    minimumTightEdges = leftTightEdgesCount[i];
                    startingLeftNode = i;
                }
            }
        }

        // endregion Find unmatched starting node

        // assert(startingLeftNode != unmatched);
        // assert(leftNodeQueue.empty());
        leftNodeQueue.push_back(startingLeftNode);
        leftSeen[startingLeftNode] = 1U;

        std::uint32_t endingRightNode{unmatched};
        while (endingRightNode == unmatched) {
            // region BFS until match found or no edges to follow

            while ((endingRightNode == unmatched) && !leftNodeQueue.empty()) {
                // Implementation note: this could just as easily be a DFS, but a BFS probably
                // has less edge flipping (by my guess), so we're using a BFS.
                std::uint32_t const i{leftNodeQueue.front()};
                leftNodeQueue.pop_front();

                std::vector<LeftEdge> &edges = leftEdges[i];
                // Note: Some of the edges might not be tight anymore, hence the awful loop.
                for (std::uint32_t edgeIndex{0U}; edgeIndex < leftTightEdgesCount[i]; ++edgeIndex) {
                    LeftEdge const &edge{edges[edgeIndex]};
                    std::uint32_t const j{edge.right};

                    // ??
                    // assert(edge.cost - leftPotential[i] - rightPotential[j] >= std::numeric_limits<float>::epsilon());
                    if ((edge.cost + std::numeric_limits<float>::epsilon()) > (leftPotential[i] + rightPotential[j])) {
                        // This edge is loose now.
                        --leftTightEdgesCount[i];
                        std::swap(edges[edgeIndex], edges[leftTightEdgesCount[i]]);
                        --edgeIndex;
                        continue;
                    }

                    if (rightBacktrack[j] != unmatched) {
                        continue;
                    }

                    rightBacktrack[j] = i;
                    std::uint32_t matchedTo{rightMatchedTo[j]};
                    if (matchedTo == unmatched) {
                        // Match found. This will terminate the loop.
                        endingRightNode = j;
                    } else if (leftSeen[matchedTo] == 0U) {
                        // No match found, but a new left node is reachable. Track how we got here and extend BFS queue.
                        leftSeen[matchedTo] = 1U;
                        leftNodeQueue.push_back(matchedTo);
                    }
                }

                // region Update cached slack values

                // The remaining edges may be to nodes that are unreachable.
                // We accordingly update the minimum slackness for nodes on the right.

                if (endingRightNode == unmatched) {
                    float const potential{leftPotential[i]};
                    for (std::uint32_t edgeIndex{leftTightEdgesCount[i]}; edgeIndex < edges.size(); ++edgeIndex) {
                        LeftEdge const &edge{edges[edgeIndex]};
                        std::uint32_t j{edge.right};
                        if ((rightMatchedTo[j] == unmatched) || (leftSeen[rightMatchedTo[j]] == 0U)) {
                            // This edge is to a node on the right that we haven't reached yet.
                            float reducedCost{edge.cost - potential - rightPotential[j]};
                            // assert(reducedCost >= std::numeric_limits<float>::epsilon());

                            if ((reducedCost - rightMinimumSlack[j]) < std::numeric_limits<float>::epsilon()) {
                                // There should be a better way to do this backtracking...
                                // One array instead of 3. But I can't think of something else. So it goes.
                                rightMinimumSlack[j] = reducedCost;
                                rightMinimumSlackLeftNode[j] = i;
                                rightMinimumSlackEdgeIndex[j] = edgeIndex;
                            }
                        }
                    }
                }

                // endregion Update cached slack values
            }

            // endregion BFS until match found or no edges to follow

            // region Update node potentials to add edges, if no match found

            if (endingRightNode == unmatched) {
                // Out of nodes. Time to update some potentials.
                std::uint32_t minimumSlackRightNode = unmatched;

                // region Find minimum slack node, or abort if none exists
                float minimumSlack = std::numeric_limits<float>::max();
                for (std::uint32_t j{0U}; j < n; ++j) {
                    if ((rightMatchedTo[j] == unmatched) || (leftSeen[rightMatchedTo[j]] == 0U)) {
                        // This isn't a node reached by our BFS. Update minimum slack.
                        if ((rightMinimumSlack[j] - minimumSlack) < std::numeric_limits<float>::epsilon()) {
                            minimumSlack = rightMinimumSlack[j];
                            minimumSlackRightNode = j;
                        }
                    }
                }

                if ((minimumSlackRightNode == unmatched) ||
                    (rightMinimumSlackLeftNode[minimumSlackRightNode] == unmatched)) {
                    // The caches are all empty. There was no option available.
                    // This means that the node the BFS started at, which is an unmatched left node, cannot reach the
                    // right - i.e. it will be impossible to find a perfect matching.
                    return {};
                }

                // endregion Find minimum slack node, or abort if none exists
                // assert(minimumSlackRightNode != unmatched);

                // Adjust potentials on left and right.
                for (std::uint32_t i{0U}; i < n; ++i) {
                    if (leftSeen[i] != 0U) {
                        leftPotential[i] += minimumSlack;
                        if (leftMatchedTo[i] != unmatched) {
                            rightPotential[leftMatchedTo[i]] -= minimumSlack;
                        }
                    }
                }

                // Downward-adjust slackness caches.
                for (std::uint32_t j{0U}; j < n; ++j) {
                    if ((rightMatchedTo[j] == unmatched) || (leftSeen[rightMatchedTo[j]] == 0U)) {
                        rightMinimumSlack[j] -= minimumSlack;

                        // If the slack hit zero, then we just found ourselves a new tight edge.
                        if (std::abs(rightMinimumSlack[j]) < std::numeric_limits<float>::epsilon()) {
                            std::uint32_t const i{rightMinimumSlackLeftNode[j]};
                            std::uint32_t const edgeIndex{rightMinimumSlackEdgeIndex[j]};

                            // region Update leftEdges[i] and leftTightEdgesCount[i]

                            // Move it in the relevant edge list.
                            if (edgeIndex != leftTightEdgesCount[i]) {
                                std::vector<LeftEdge> &edges = leftEdges[i];
                                std::swap(edges[edgeIndex], edges[leftTightEdgesCount[i]]);
                            }
                            ++leftTightEdgesCount[i];

                            // endregion Update leftEdges[i] and leftTightEdgesCount[i]

                            // If we haven't already encountered a match, we follow the edge and update the BFS queue.
                            // It's possible this edge leads to a match. If so, we'll carry on updating the tight edges,
                            // but won't follow them.
                            if (endingRightNode == unmatched) {
                                // We're contemplating the consequences of following (i, j), as we do in the BFS above.
                                rightBacktrack[j] = i;
                                std::uint32_t matchedTo{rightMatchedTo[j]};
                                if (matchedTo == unmatched) {
                                    // Match found!
                                    endingRightNode = j;
                                } else if (leftSeen[matchedTo] == 0U) {
                                    // No match, but new left node found. Extend BFS queue.
                                    leftSeen[matchedTo] = 1U;
                                    leftNodeQueue.push_back(matchedTo);
                                }
                            }
                        }
                    }
                }
            }

            // endregion Update node potentials to add edges, if no match found
        }

        // At this point, we've found an augmenting path between startingLeftNode and endingRightNode.
        // We'll just use the backtracking info to update our match information.
        ++currentMatchingCardinality;

        // region Backtrack and flip augmenting path
        {
            std::uint32_t currentRightNode{endingRightNode};
            while (currentRightNode != unmatched) {
                std::uint32_t const currentLeftNode{rightBacktrack[currentRightNode]};
                std::uint32_t const nextRightNode{leftMatchedTo[currentLeftNode]};

                rightMatchedTo[currentRightNode] = currentLeftNode;
                leftMatchedTo[currentLeftNode] = currentRightNode;

                currentRightNode = nextRightNode;
            }
        }

        // endregion Backtrack and flip augmenting path
    }

    // Oh look, we're done.
    return leftMatchedTo;
}

}  // namespace rally
