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

#include "rally/core/concurrency/atomic_rw_lock.h"

namespace rally {

void AtomicRWLock::readLock() noexcept {
    uint32_t retry_times{0};
    int32_t tmp_lock_num{lock_num_.load()};
    if (write_first_) {
        do {
            while (tmp_lock_num < RW_LOCK_FREE || write_lock_wait_num_.load() > 0) {
                if (++retry_times == MAX_RETRY_TIMES) {
                    // saving cpu
                    std::this_thread::yield();
                    retry_times = 0;
                }
                tmp_lock_num = lock_num_.load();
            }
        } while (!lock_num_.compare_exchange_weak(tmp_lock_num, tmp_lock_num + 1,
                                                  std::memory_order_acq_rel,
                                                  std::memory_order_relaxed));
    } else {
        do {
            while (tmp_lock_num < RW_LOCK_FREE) {
                if (++retry_times == MAX_RETRY_TIMES) {
                    // saving cpu
                    std::this_thread::yield();
                    retry_times = 0;
                }
                tmp_lock_num = lock_num_.load();
            }
        } while (!lock_num_.compare_exchange_weak(tmp_lock_num, tmp_lock_num + 1,
                                                  std::memory_order_acq_rel,
                                                  std::memory_order_relaxed));
    }
}

void AtomicRWLock::writeLock() noexcept {
    int32_t tmp_rw_lock_free{RW_LOCK_FREE};
    uint32_t retry_times{0};
    static_cast<void>(write_lock_wait_num_.fetch_add(1));
    while (!lock_num_.compare_exchange_weak(tmp_rw_lock_free, WRITE_EXCLUSIVE,
                                            std::memory_order_acq_rel,
                                            std::memory_order_relaxed)) {
        // rw_lock_free will change after CAS fail, so init agin
        tmp_rw_lock_free = RW_LOCK_FREE;
        if (++retry_times == MAX_RETRY_TIMES) {
            // saving cpu
            std::this_thread::yield();
            retry_times = 0;
        }
    }
    static_cast<void>(write_lock_wait_num_.fetch_sub(1));
}

}  // namespace rally
