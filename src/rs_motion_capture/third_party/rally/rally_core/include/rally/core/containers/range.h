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
#ifndef RALLY_CORE_CONTAINERS_RANGE_H
#define RALLY_CORE_CONTAINERS_RANGE_H

#include <cstdint>
#include <limits>
#include <vector>
#include "rally/utils/utils.h"

namespace rally {

class Range {
public:
    Range() : start_(0), end_(0) {}

    Range(int32_t _start, int32_t _end) : start_(_start), end_(_end) {
        RENSURE(start_ <= end_);
    }

    int32_t size() const {
        return end_ - start_;
    }

    bool empty() const {
        return start_ == end_;
    }

    static Range all() {
        return Range(std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max());
    }

    int32_t start() const {
        return start_;
    }

    int32_t end() const {
        return end_;
    }

    std::vector<Range> split_ranges(uint32_t slices) const {
        std::vector<Range> res;
        if (slices == 0) {
            return res;
        }
        res.reserve(slices);
        if (slices <= 1) {
            res.emplace_back(*this);
        } else {
            int32_t stride = (size() / static_cast<int32_t>(slices) + 1);
            for (uint32_t i = 0; i < slices; ++i) {
                int32_t start = start_ + i * stride;
                int32_t end = start_ + (i + 1) * stride;
                if (start >= end_) {
                    start = end_;
                    end = end_;
                }
                if (end >= end_) {
                    end = end_;
                }
                if (start < end) {
                    res.emplace_back(start, end);
                }
            }
        }
        return res;
    }

private:
    int32_t start_, end_;
};

}  // namespace rally

#endif  // RALLY_CORE_CONTAINERS_RANGE_H
