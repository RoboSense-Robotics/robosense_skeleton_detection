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

#include "rally/core/algorithm/geometry/delaunator.h"

namespace rally {

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();

inline size_t fast_mod(const size_t i, const size_t c) { return i >= c ? i % c : i; }

// monotonically increases with real angle, but doesn't need expensive trigonometry
inline float pseudo_angle(float dx, float dy) {
    const float p = dx / (std::abs(dx) + std::abs(dy));
    return (dy > 0.f ? 3.f - p : 1.f + p) / 4.f; // [0..1)
}

inline float dist(const ShortArray2f &pt1, const ShortArray2f &pt2) {
    const auto pt = pt1 - pt2;
    return pt.x * pt.x + pt.y * pt.y;
}

inline float circumRadius(const ShortArray2f &pta, const ShortArray2f &ptb, const ShortArray2f &ptc) {
    const auto ptd = ptb - pta;
    const auto pte = ptc - pta;

    float bl = ptd.x * ptd.x + ptd.y * ptd.y;
    float cl = pte.x * pte.x + pte.y * pte.y;
    float d = ptd.x * pte.y - ptd.y * pte.x;

    float x = (pte.y * bl - ptd.y * cl) * 0.5f / d;
    float y = (ptd.x * cl - pte.x * bl) * 0.5f / d;

    if (isEqual(bl, 0.f) || isEqual(cl, 0.f) || isEqual(d, 0.f)) {
        return std::numeric_limits<float>::max();
    }
    return x * x + y * y;
}

inline bool orient_fast(const ShortArray2f &p, const ShortArray2f &q, const ShortArray2f &r) {
    return (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y) < 0.f;
}

inline bool in_circle_fast(const ShortArray2f &a, const ShortArray2f &b, const ShortArray2f &c, const ShortArray2f &p) {
    const auto dx = a.x - p.x;
    const auto dy = a.y - p.y;
    const auto ex = b.x - p.x;
    const auto ey = b.y - p.y;
    const auto fx = c.x - p.x;
    const auto fy = c.y - p.y;

    const auto ap = dx * dx + dy * dy;
    const auto bp = ex * ex + ey * ey;
    const auto cp = fx * fx + fy * fy;

    return (dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx)) < 0.f;
}

inline ShortArray2f circumCenter(const ShortArray2f &pta, const ShortArray2f &ptb, const ShortArray2f &ptc) {
    const auto ptd = ptb - pta;
    const auto pte = ptc - pta;

    float bl = ptd.x * ptd.x + ptd.y * ptd.y;
    float cl = pte.x * pte.x + pte.y * pte.y;
    float d = ptd.x * pte.y - ptd.y * pte.x;

    float x = pta.x + (pte.y * bl - ptd.y * cl) * 0.5f / d;
    float y = pta.y + (ptd.x * cl - pte.x * bl) * 0.5f / d;
    return ShortArray2f(x, y);
}

inline float sum(const std::vector<float> &x) {
    float sum = x[0];
    float err = 0.f;

    for (size_t i = 1; i < x.size(); i++) {
        const auto k = x[i];
        const auto m = sum + k;
        err += std::abs(sum) >= std::abs(k) ? sum - m + k : k - m + sum;
        sum = m;
    }
    return sum + err;
}

void Delaunator::triangulate() {
    size_t n = vertices_.size();

    ShortArray2f max_pt = ShortArray2f(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
    ShortArray2f min_pt = ShortArray2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

    std::vector<std::size_t> ids(n);

    for (size_t i = 0; i < n; ++i) {
        const auto &pt = vertices_[i];
        max_pt.x = std::max(max_pt.x, pt.x);
        max_pt.y = std::max(max_pt.y, pt.y);
        min_pt.x = std::min(min_pt.x, pt.x);
        min_pt.y = std::min(min_pt.y, pt.y);
        ids[i] = i;
    }

    ShortArray2f mid_pt = (max_pt + min_pt) * 0.5;

    float min_dist = std::numeric_limits<float>::max();

    std::size_t i0 = INVALID_INDEX;
    std::size_t i1 = INVALID_INDEX;
    std::size_t i2 = INVALID_INDEX;

    for (size_t i = 0; i < n; ++i) {
        const auto &pt = vertices_[i];
        const auto d = dist(pt, mid_pt);
        if (d < min_dist) {
            min_dist = d;
            i0 = i;
        }
    }
    auto i0_pt = vertices_[i0];

    min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < n; ++i) {
        if (i == i0) {
            continue;
        }
        const auto &pt = vertices_[i];
        const auto d = dist(pt, i0_pt);
        if (d < min_dist) {
            min_dist = d;
            i1 = i;
        }
    }
    auto i1_pt = vertices_[i1];

    min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < n; ++i) {
        if (i == i0 || i == i1) {
            continue;
        }
        const auto d = circumRadius(i0_pt, i1_pt, vertices_[i]);
        if (d < min_dist) {
            min_dist = d;
            i2 = i;
        }
    }

    RENSURE(min_dist < std::numeric_limits<float>::max());

    auto i2_pt = vertices_[i2];

    if (orient_fast(i0_pt, i1_pt, i2_pt)) {
        std::swap(i1, i2);
        std::swap(i1_pt, i2_pt);
    }

    m_center_ = circumCenter(i0_pt, i1_pt, i2_pt);

    std::vector<float> dists;
    dists.reserve(ids.size());

    for (const auto &id : ids) {
        const auto d = dist(vertices_[id], m_center_);
        dists.emplace_back(d);
    }
    // sort the points by distance from the seed triangle circumcenter
    std::sort(ids.begin(), ids.end(), [this, dists](std::size_t i, std::size_t j) -> bool {
        float diff1 = dists[i] - dists[j];
        float diff2 = this->vertices_[i].x - this->vertices_[j].x;
        float diff3 = this->vertices_[i].y - this->vertices_[j].y;
        if (diff1 > 0.f || diff1 < 0.f) {
            return diff1 < 0.f;
        } else if (diff2 > 0.f || diff2 < 0.f) {
            return diff2 < 0.f;
        } else {
            return diff3 < 0.f;
        }
    });

    // initialize a hash table for storing edges of the advancing convex hull
    size_t hash_size = static_cast<std::size_t>(std::llround(std::ceil(std::sqrt(n))));
    m_hash_.resize(hash_size);
    std::fill(m_hash_.begin(), m_hash_.end(), INVALID_INDEX);

    // initialize arrays for tracking the edges of the advancing convex hull
    hull_prev_.resize(n);
    hull_next_.resize(n);
    hull_tri_.resize(n);

    hull_start_ = i0;

    std::size_t hull_size = 3;

    hull_next_[i0] = hull_prev_[i2] = i1;
    hull_next_[i1] = hull_prev_[i0] = i2;
    hull_next_[i2] = hull_prev_[i1] = i0;

    hull_tri_[i0] = 0;
    hull_tri_[i1] = 1;
    hull_tri_[i2] = 2;

    m_hash_[hash_key(i0_pt)] = i0;
    m_hash_[hash_key(i1_pt)] = i1;
    m_hash_[hash_key(i2_pt)] = i2;

    std::size_t max_triangles = n < 3 ? 1 : 2 * n - 5;

    triangles_.reserve(max_triangles * 3);
    triangles_.clear();
    half_edges_.reserve(max_triangles * 3);
    half_edges_.clear();

    add_triangle(i0, i1, i2, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX);

    ShortArray2f p_pt = ShortArray2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
    for (std::size_t k = 0; k < n; k++) {
        const auto i = ids[k];
        const auto &pt = vertices_[i];

        // skip near-duplicate points
        if (k > 0 && (pt == p_pt)) {
            continue;
        }
        p_pt = pt;

        // skip seed triangle points
        if (pt == i0_pt || pt == i1_pt || pt == i2_pt) {
            continue;
        }

        // find a visible edge on the convex hull using edge hash
        std::size_t start = 0;

        std::size_t key = hash_key(pt);
        for (size_t j = 0; j < m_hash_.size(); j++) {
            start = m_hash_[fast_mod(key + j, m_hash_.size())];
            if (start != INVALID_INDEX && start != hull_next_[start]) {
                break;
            }
        }

        start = hull_prev_[start];
        std::size_t e = start;
        std::size_t q;

        while (q = hull_next_[e], !orient_fast(vertices_[i], vertices_[e], vertices_[q])) {
            e = q;
            if (e == start) {
                e = INVALID_INDEX;
                break;
            }
        }

        if (e == INVALID_INDEX) {
            continue; // likely a near-duplicate point; skip it
        }

        // add the first triangle from the point
        std::size_t t = add_triangle(e, i, hull_next_[e], INVALID_INDEX, INVALID_INDEX, hull_tri_[e]);

        hull_tri_[i] = legalize(t + 2);
        hull_tri_[e] = t;
        hull_size++;

        // walk forward through the hull, adding more triangles_ref and flipping recursively
        std::size_t next = hull_next_[e];
        while (q = hull_next_[next], orient_fast(vertices_[i], vertices_[next], vertices_[q])) {
            t = add_triangle(next, i, q, hull_tri_[i], INVALID_INDEX, hull_tri_[next]);
            hull_tri_[i] = legalize(t + 2);
            hull_next_[next] = next; // mark as removed
            hull_size--;
            next = q;
        }

        // walk backward from the other side, adding more triangles_ref and flipping
        if (e == start) {
            while (q = hull_prev_[e], orient_fast(vertices_[i], vertices_[q], vertices_[e])) {
                t = add_triangle(q, i, e, INVALID_INDEX, hull_tri_[e], hull_tri_[q]);
                legalize(t + 2);
                hull_tri_[q] = t;
                hull_next_[e] = e; // mark as removed
                hull_size--;
                e = q;
            }
        }

        // update the hull indices
        hull_prev_[i] = e;
        hull_start_ = e;
        hull_prev_[next] = i;
        hull_next_[e] = i;
        hull_next_[i] = next;

        m_hash_[hash_key(pt)] = i;
        m_hash_[hash_key(vertices_[e])] = e;
    }
}

float Delaunator::getHullArea() {
    std::vector<float> hull_area;
    size_t e = hull_start_;
    do {
        hull_area.push_back((vertices_[e].x - vertices_[hull_prev_[e]].x) *
                            (vertices_[e].y + vertices_[hull_prev_[e]].y));
        e = hull_next_[e];
    } while (e != hull_start_);
    return sum(hull_area);
}

std::size_t Delaunator::legalize(std::size_t a) {
    std::size_t i = 0;
    std::size_t ar = 0;
    std::vector<std::size_t> m_edge_stack;
    m_edge_stack.clear();

    int cc = 0;
    // recursion eliminated with a fixed-size stack
    while (true) {
        const std::size_t b_val = half_edges_[a];
        cc++;
        // if the pair of triangles doesn't satisfy the Delaunay condition
        // (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
        // then do the same check/flip recursively for the new pair of triangles
        //
        //           pl                    pl
        //          /||\                  /  \
        //       al/ || \bl            al/    \a
        //        /  ||  \              /      \
        //       /  a||b  \    flip    /___ar___\
        //     p0\   ||   /p1   =>   p0\---bl---/p1
        //        \  ||  /              \      /
        //       ar\ || /br             b\    /br
        //          \||/                  \  /
        //           pr                    pr
        //
        const std::size_t a0 = 3 * (a / 3);
        ar = a0 + (a + 2) % 3;

        if (b_val == INVALID_INDEX) {
            if (i > 0) {
                i--;
                a = m_edge_stack[i];
                continue;
            } else {
                // i = INVALID_INDEX;
                break;
            }
        }

        const std::size_t b0 = 3 * (b_val / 3);
        const std::size_t al = a0 + (a + 1) % 3;
        const std::size_t bl = b0 + (b_val + 2) % 3;

        const std::size_t p0 = triangles_[ar];
        const std::size_t pr = triangles_[a];
        const std::size_t pl = triangles_[al];
        const std::size_t p1 = triangles_[bl];

        const bool illegal = in_circle_fast(vertices_[p0], vertices_[pr], vertices_[pl], vertices_[p1]);
        if (illegal) {
            triangles_[a] = p1;
            triangles_[b_val] = p0;

            auto hbl = half_edges_[bl];

            // edge swapped on the other side of the hull (rare); fix the halfedge reference
            if (hbl == INVALID_INDEX) {
                std::size_t e = hull_start_;
                do {
                    if (hull_tri_[e] == bl) {
                        hull_tri_[e] = a;
                        break;
                    }
                    e = hull_prev_[e];
                } while (e != hull_start_);
            }
            link(a, hbl);
            link(b_val, half_edges_[ar]);
            link(ar, bl);
            std::size_t br = b0 + (b_val + 1) % 3;

            if (i < m_edge_stack.size()) {
                m_edge_stack[i] = br;
            } else {
                m_edge_stack.push_back(br);
            }
            i++;
        } else {
            if (i > 0) {
                i--;
                a = m_edge_stack[i];
                continue;
            } else {
                break;
            }
        }
    }

    return ar;
}

std::size_t Delaunator::hash_key(const ShortArray2f &pt) const {
    const ShortArray2f d = pt - m_center_;
    return fast_mod(
    static_cast<std::size_t>(std::llround(std::floor(pseudo_angle(d.x, d.y) * static_cast<float>(m_hash_.size())))),
    m_hash_.size());
}

std::size_t Delaunator::add_triangle(std::size_t i0, std::size_t i1, std::size_t i2, std::size_t a, std::size_t b,
                                     std::size_t c) {
    std::size_t t = triangles_.size();
    triangles_.emplace_back(i0);
    triangles_.emplace_back(i1);
    triangles_.emplace_back(i2);
    link(t, a);
    link(t + 1, b);
    link(t + 2, c);
    return t;
}

void Delaunator::link(const std::size_t a, const std::size_t b) {
    std::size_t s = half_edges_.size();
    if (a == s) {
        half_edges_.emplace_back(b);
    } else if (a < s) {
        half_edges_[a] = b;
    } else {
        RTHROW("Cannot link edge");
    }
    if (b != INVALID_INDEX) {
        std::size_t s2 = half_edges_.size();
        if (b == s2) {
            half_edges_.emplace_back(a);
        } else if (b < s2) {
            half_edges_[b] = a;
        } else {
            RTHROW("Cannot link edge");
        }
    }
}

}  // namespace rally
