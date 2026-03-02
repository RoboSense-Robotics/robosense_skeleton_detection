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

#include "rally/utils/timer/duration.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

namespace rally {

Duration::Duration(int64_t nano_seconds) { nano_seconds_ = nano_seconds; }

Duration::Duration(int nano_seconds) {
    nano_seconds_ = static_cast<int64_t>(nano_seconds);
}

Duration::Duration(double seconds) {
    nano_seconds_ = static_cast<int64_t>(seconds * 1000000000UL);
}

Duration::Duration(uint32_t seconds, uint32_t nano_seconds) {
    nano_seconds_ = static_cast<uint64_t>(seconds) * 1000000000UL + nano_seconds;
}

Duration::Duration(const Duration &other) { nano_seconds_ = other.nano_seconds_; }

Duration &Duration::operator=(const Duration &other) {
    this->nano_seconds_ = other.nano_seconds_;
    return *this;
}

double Duration::toSecond() const {
    return static_cast<double>(nano_seconds_) / 1000000000UL;
}

int64_t Duration::toNanosecond() const { return nano_seconds_; }

bool Duration::isZero() const { return nano_seconds_ == 0; }

void Duration::sleep() const {
    auto sleep_time = std::chrono::nanoseconds(nano_seconds_);
    std::this_thread::sleep_for(sleep_time);
}

Duration Duration::operator+(const Duration &rhs) const {
    return Duration(nano_seconds_ + rhs.nano_seconds_);
}

Duration Duration::operator-(const Duration &rhs) const {
    return Duration(nano_seconds_ - rhs.nano_seconds_);
}

Duration Duration::operator-() const { return Duration(-nano_seconds_); }

Duration Duration::operator*(double scale) const {
    return Duration(int64_t(static_cast<double>(nano_seconds_) * scale));
}

Duration &Duration::operator+=(const Duration &rhs) {
    *this = *this + rhs;
    return *this;
}

Duration &Duration::operator-=(const Duration &rhs) {
    *this = *this - rhs;
    return *this;
}

Duration &Duration::operator*=(double scale) {
    *this = Duration(int64_t(static_cast<double>(nano_seconds_) * scale));
    return *this;
}

bool Duration::operator==(const Duration &rhs) const {
    return nano_seconds_ == rhs.nano_seconds_;
}

bool Duration::operator!=(const Duration &rhs) const {
    return nano_seconds_ != rhs.nano_seconds_;
}

bool Duration::operator>(const Duration &rhs) const {
    return nano_seconds_ > rhs.nano_seconds_;
}

bool Duration::operator<(const Duration &rhs) const {
    return nano_seconds_ < rhs.nano_seconds_;
}

bool Duration::operator>=(const Duration &rhs) const {
    return nano_seconds_ >= rhs.nano_seconds_;
}

bool Duration::operator<=(const Duration &rhs) const {
    return nano_seconds_ <= rhs.nano_seconds_;
}

std::ostream &operator<<(std::ostream &os, const Duration &rhs) {
    std::ios::fmtflags before(os.flags());
    os << std::fixed << std::setprecision(9) << rhs.toSecond() << "s";
    os.flags(before);
    return os;
}

}  // namespace rally
