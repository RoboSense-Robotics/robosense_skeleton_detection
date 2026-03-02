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
#ifndef RALLY_CORE_CONCURRENCY_ATOMIC_RW_LOCK_H
#define RALLY_CORE_CONCURRENCY_ATOMIC_RW_LOCK_H

#include <unistd.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>
#include "rally/core/concurrency/rw_lock_guard.h"

namespace rally {

class AtomicRWLock {
    friend class ReadLockGuard<AtomicRWLock>;

    friend class WriteLockGuard<AtomicRWLock>;

public:
    static const int32_t RW_LOCK_FREE = 0;
    static const int32_t WRITE_EXCLUSIVE = -1;
    static const uint32_t MAX_RETRY_TIMES = 5;

    AtomicRWLock() {}

    explicit AtomicRWLock(bool write_first) : write_first_(write_first) {}

private:
    // all these function only can used by ReadLockGuard/WriteLockGuard;
    void readLock() noexcept;

    void writeLock() noexcept;

    void readUnlock() noexcept { static_cast<void>(lock_num_.fetch_sub(1)); }

    void writeUnlock() noexcept { static_cast<void>(lock_num_.fetch_add(1)); }

    RALLY_DISALLOW_COPY_AND_ASSIGN(AtomicRWLock)

    std::atomic<uint32_t> write_lock_wait_num_ = {0};
    std::atomic<int32_t> lock_num_ = {0};
    bool write_first_ = true;
};

}  // namespace rally

#endif  // RALLY_CORE_CONCURRENCY_ATOMIC_RW_LOCK_H
