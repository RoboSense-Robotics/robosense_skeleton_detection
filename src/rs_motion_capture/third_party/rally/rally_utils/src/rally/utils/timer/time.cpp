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

#include <chrono>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include "rally/utils/timer/time.h"

namespace rally {

static constexpr double NanoToSeconds{1000000000.0};
static constexpr uint64_t NanoToSecondsNum{1000000000UL};
static constexpr uint32_t SetWidth{9};

using std::chrono::high_resolution_clock;
using std::chrono::steady_clock;
using std::chrono::system_clock;

Time &Time::operator=(const Time &other) noexcept {
    this->nanoseconds_ = other.nanoseconds_;
    return *this;
}

Time Time::getNow() noexcept {
    const auto &now = high_resolution_clock::now();
    const auto &nano_time_point =
    std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    const auto &epoch = nano_time_point.time_since_epoch();
    const uint64_t now_nano{static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count())};
    return Time(now_nano);
}


void Time::sleepUntil(const Time& time) {
    auto nano = std::chrono::nanoseconds(time.toNanosecond());
    system_clock::time_point tp(nano);
    std::this_thread::sleep_until(tp);
}

std::string Time::toString() const noexcept {
    const auto nano = std::chrono::nanoseconds(nanoseconds_);
#if __linux__
    system_clock::time_point const tp{nano};
#endif
#if __QNX__
    system_clock::time_point tp(std::chrono::duration_cast<system_clock::duration>(nano));
#endif
    const auto &l_time = system_clock::to_time_t(tp);
    struct tm stm;
    const auto &ret = localtime_r(&l_time, &stm);
    if (ret == nullptr) {
        return std::to_string(static_cast<double>(nanoseconds_) / NanoToSeconds);
    }

    std::stringstream ss;
#if __GNUC__ >= 5
    ss << std::put_time(ret, "%F %T");
    ss << "." << std::setw(SetWidth) << std::setfill('0') << nanoseconds_ % NanoToSecondsNum;
#else
    char date_time[128];
    strftime(date_time, sizeof(date_time), "%F %T", ret);
    ss << std::string(date_time) << "." << std::setw(SetWidth) << std::setfill('0')
    << nanoseconds_ % NanoToSecondsNum;
#endif
    return ss.str();
}

bool Time::isZero() const { return nanoseconds_ == 0; }

Duration Time::operator-(const Time &rhs) const {
    return Duration(static_cast<int64_t>(nanoseconds_ - rhs.nanoseconds_));
}

Time Time::operator+(const Duration &rhs) const {
    return Time(nanoseconds_ + rhs.toNanosecond());
}

Time Time::operator-(const Duration &rhs) const {
    return Time(nanoseconds_ - rhs.toNanosecond());
}

Time &Time::operator+=(const Duration &rhs) {
    *this = *this + rhs;
    return *this;
}

Time &Time::operator-=(const Duration &rhs) {
    *this = *this - rhs;
    return *this;
}

std::ostream &operator<<(std::ostream &os, const Time &rhs) noexcept {
    os << rhs.toString();
    return os;
}

}  // namespace rally
