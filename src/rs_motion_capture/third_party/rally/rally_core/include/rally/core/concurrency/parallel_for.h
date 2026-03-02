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
#ifndef RALLY_CORE_CONCURRENCY_PARALLEL_FOR_H
#define RALLY_CORE_CONCURRENCY_PARALLEL_FOR_H

#include "rally/core/containers/range.h"
#include "rally/core/concurrency/thread_pool.h"
#include "rally/utils/utils.h"

namespace rally {

inline void parallel_for(const std::function<void(const Range &)> &func, const Range &range, int32_t n_tasks = -1) {
    if (n_tasks <= 0) {
        n_tasks = Singleton<ThreadPool>::getInstance().size();
    }
    if (n_tasks <= 0) {
        func(range);
        return;
    }

    std::vector<Range> split_ranges = range.split_ranges(static_cast<uint32_t>(n_tasks));
    std::vector<std::future<void> > res(split_ranges.size());
    for (size_t i = 0; i < split_ranges.size(); ++i) {
        res[i] = Singleton<ThreadPool>::getInstance().push([func, split_ranges, i](int) {
            func(split_ranges[i]);
        });
    }
    for (size_t i = 0; i < split_ranges.size(); ++i) {
        res[i].get();
    }
}

inline void parallel_for(const std::function<void(uint32_t)> &func, uint32_t n_tasks) {
    std::vector<std::future<void> > res(n_tasks);
    for (uint32_t i = 0; i < n_tasks; ++i) {
        res[i] = Singleton<ThreadPool>::getInstance().push([func, i](int) {
            func(i);
        });
    }
    for (uint32_t i = 0; i < n_tasks; ++i) {
        res[i].get();
    }
}

}  // namespace rally

#endif  // RALLY_CORE_CONCURRENCY_PARALLEL_FOR_H
