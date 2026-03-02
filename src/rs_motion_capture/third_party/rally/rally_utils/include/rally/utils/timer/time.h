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
#ifndef RALLY_UTILS_TIMER_TIME_H
#define RALLY_UTILS_TIMER_TIME_H

#include <limits>
#include <string>
#include "rally/utils/timer/duration.h"

namespace rally {

/// @brief builtin time type Time.
class Time {
public:
    Time() = default;

    explicit Time(uint64_t nanoseconds) noexcept { nanoseconds_ = nanoseconds; }

    explicit Time(int nanoseconds) noexcept { nanoseconds_ = static_cast<uint64_t>(nanoseconds); }

    explicit Time(double seconds) noexcept { nanoseconds_ = static_cast<uint64_t>(seconds * 1000000000UL); }

    Time(uint32_t seconds, uint32_t nanoseconds) noexcept {
        nanoseconds_ = static_cast<uint64_t>(seconds) * 1000000000UL + nanoseconds;
    }

    Time(const Time &other) noexcept { nanoseconds_ = other.nanoseconds_; }

    Time &operator=(const Time &other) noexcept;

    /// @brief get the current time.
    /// @return return the current time.
    static Time getNow() noexcept;

    static void sleepUntil(const Time& time);

    /// @brief convert time to second.
    /// @return return a double value unit is second.
    double toSecond() const { return static_cast<double>(nanoseconds_) / 1000000000UL; }

    /// @brief convert time to microsecond (us).
    /// @return return a unit64_t value unit is us.
    uint64_t toMicrosecond() const { return static_cast<uint64_t>(nanoseconds_ / 1000.0); }

    /// @brief convert time to nanosecond.
    /// @return return a unit64_t value unit is nanosecond.
    uint64_t toNanosecond() const { return nanoseconds_; }

    /// @brief convert time to a string.
    /// @return return a string.
    std::string toString() const noexcept;

    bool isZero() const;

    Duration operator-(const Time &rhs) const;

    Time operator+(const Duration &rhs) const;

    Time operator-(const Duration &rhs) const;

    Time &operator+=(const Duration &rhs);

    Time &operator-=(const Duration &rhs);

    bool operator==(const Time &rhs) const { return nanoseconds_ == rhs.nanoseconds_; }

    bool operator!=(const Time &rhs) const { return nanoseconds_ != rhs.nanoseconds_; }

    bool operator>(const Time &rhs) const { return nanoseconds_ > rhs.nanoseconds_; }

    bool operator<(const Time &rhs) const { return nanoseconds_ < rhs.nanoseconds_; }

    bool operator>=(const Time &rhs) const { return nanoseconds_ >= rhs.nanoseconds_; }

    bool operator<=(const Time &rhs) const { return nanoseconds_ <= rhs.nanoseconds_; }

private:
    uint64_t nanoseconds_ = 0;
};

std::ostream &operator<<(std::ostream &os, const Time &rhs) noexcept;

/// @brief get the current time in second (s)
/// @return return a double value unit in second
inline double getNowInSeconds() { return std::move(rally::Time::getNow().toSecond()); }

/// @brief get the current tine in millsecond (ms)
/// @return return a double value unit in millsecond
inline double getNowInMillSeconds() { return std::move(rally::Time::getNow().toSecond()) * 1000.; }

/// @brief get the current time in microsecond (us)
/// @return return a uint64_t value unit in microsecond
inline uint64_t getNowInMicroSeconds() { return std::move(rally::Time::getNow().toMicrosecond()); }

/// @brief get the current time in nanosecond (ns)
/// @return return a uint64_t value unit in nanosecond
inline uint64_t getNowInNanoSeconds() { return std::move(rally::Time::getNow().toNanosecond()); }

/// @brief get the current time in string format
/// @return return a string
inline std::string getNowInString() { return std::move(rally::Time::getNow().toString()); }

/// @brief convert nanosecond to second
/// @param[in] nano_timestamp timestamp in nanosecond
/// @return return a double value unit in second
inline double toSeconds(uint64_t nano_timestamp) {
    return std::move(static_cast<double>(nano_timestamp) / 1000000000UL);
}

inline std::string toString(uint64_t nano_timestamp){
    return std::move(rally::Time(nano_timestamp).toString());
}

/// @brief convert nanosecond to second
/// @param[in] nano_timestamp timestamp in nanosecond
/// @return return a double value unit in millsecond
inline double toMilliSeconds(uint64_t nano_timestamp) {
    return std::move(static_cast<double>(nano_timestamp) / 1000000UL);
}

/// @brief convert second to nanosecond
/// @param[in] s_timestamp timestamp in second
/// @return return a uint64_t value unit in nanosecond
inline uint64_t toNanoSeconds(double s_timestamp) {
    return std::move(static_cast<uint64_t >(s_timestamp * 1000000000UL));
}

}  // namespace rally

#endif  // RALLY_UTILS_TIMER_TIME_H
