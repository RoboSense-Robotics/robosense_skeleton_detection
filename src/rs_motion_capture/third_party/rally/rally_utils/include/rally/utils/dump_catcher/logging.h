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

#ifndef RALLY_UTILS_DUMP_CATCHER_LOGGING_H
#define RALLY_UTILS_DUMP_CATCHER_LOGGING_H

#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <functional>

#if defined(linux) || defined(__linux) || defined(__linux__)
#define DUMP_CATCHER_OPEN
# endif  // defined(linux) || defined(__linux) || defined(__linux__)

#ifdef DUMP_CATCHER_OPEN

#include <execinfo.h>

namespace rally {
namespace log {

typedef int64_t int64;
typedef uint64_t uint64;

// Wrapper of __sync_val_compare_and_swap. If the GCC extension isn't
// defined, we try the CPU specific logics (we only support x86 and
// x86_64 for now) first, then use a naive implementation, which has a
// race condition.
template<typename T>
inline T sync_val_compare_and_swap(T *ptr, T oldval, T newval) {
#if defined(HAVE___SYNC_VAL_COMPARE_AND_SWAP)
    return __sync_val_compare_and_swap(ptr, oldval, newval);
#elif defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__))
    T ret;
    __asm__ __volatile__("lock; cmpxchg %1, (%2);"
    :"=a"(ret)
    // GCC may produces %sil or %dil for
    // constraint "r", but some of apple's gas
    // dosn't know the 8 bit registers.
    // We use "q" to avoid these registers.
    :"q"(newval), "q"(ptr), "a"(oldval)
    :"memory", "cc");
    return ret;
#else
    T ret = *ptr;
  if (ret == oldval) {
    *ptr = newval;
  }
  return ret;
#endif
}

// If you change this function, also change GetStackFrames below.
inline int GetStackTrace(void **result, int max_depth, int skip_count) {
    static const int kStackLength = 64;
    void *stack[kStackLength];
    int size;

    size = backtrace(stack, kStackLength);
    skip_count++;  // we want to skip the current frame as well
    int result_count = size - skip_count;
    if (result_count < 0)
        result_count = 0;
    if (result_count > max_depth)
        result_count = max_depth;
    for (int i = 0; i < result_count; i++)
        result[i] = stack[i + skip_count];

    return result_count;
}

void InstallFailureSignalHandler();

//void InstallFailureWriter(void (*writer)(const char *data, int size));
void InstallFailureWriter(const std::function<void(const char *data, int size)> &func);

}  // namespace log
}  // namespace rally

# endif  // DUMP_CATCHER_OPEN

#endif  // RALLY_UTILS_DUMP_CATCHER_LOGGING_H
