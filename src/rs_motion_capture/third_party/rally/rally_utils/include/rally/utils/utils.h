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
#ifndef RALLY_UTILS_UTILS_H
#define RALLY_UTILS_UTILS_H

#include "rally/utils/details/details.h"
#include "rally/utils/file_utils/file_utils.h"
#include "rally/utils/global/global.h"
#include "rally/utils/logger/sinks/terminal_sink.h"
#include "rally/utils/logger/log.h"
#include "rally/utils/thread/thread.h"
#include "rally/utils/timer/timer.h"
#include "rally/utils/yaml/yaml.h"
#include "rally/utils/register.h"
#include "rally/utils/synchronizer/synchronizer.h"
#include "rally/utils/init.h"

namespace rally {

inline std::size_t getHash(const std::string &key) {
    return std::hash<std::string>{}(key);
}

#define R_LOG_VARNAME(base, line) base ## line

#define R_LOG_OCCURRENCES R_LOG_VARNAME(occurrences_, __LINE__)
#define R_LOG_OCCURRENCES_MOD_N R_LOG_VARNAME(occurrences_mod_n_, __LINE__)

#define RINFO_EVERY_N(n, x)                                         \
{                                                                   \
    static int R_LOG_OCCURRENCES = 0, R_LOG_OCCURRENCES_MOD_N = 0;      \
    if (++R_LOG_OCCURRENCES_MOD_N > n) R_LOG_OCCURRENCES_MOD_N -= n;    \
    if (R_LOG_OCCURRENCES_MOD_N == 1)                                 \
        RINFO << x;                                                 \
}

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    RWARN << #ptr << " is nullptr."; \
    return;                          \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    RWARN << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)           \
  if (condition) {                     \
    RWARN << #condition << " is met."; \
    return;                            \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)  \
  if (condition) {                     \
    RWARN << #condition << " is met."; \
    return val;                        \
  }
#endif

}  // namespace rally

#endif  // RALLY_UTILS_UTILS_H
