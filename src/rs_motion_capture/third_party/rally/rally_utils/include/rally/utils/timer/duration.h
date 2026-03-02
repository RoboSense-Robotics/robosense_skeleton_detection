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
#ifndef RALLY_UTILS_TIMER_DURATION_H
#define RALLY_UTILS_TIMER_DURATION_H

#include <cstdint>
#include <iostream>

namespace rally {

class Duration {
public:
    Duration() = default;

    explicit Duration(int64_t nano_seconds);

    explicit Duration(int nano_seconds);

    explicit Duration(double seconds);

    Duration(uint32_t seconds, uint32_t nano_seconds);

    Duration(const Duration &other);

    Duration &operator=(const Duration &other);

    ~Duration() = default;

    double toSecond() const;

    int64_t toNanosecond() const;

    bool isZero() const;

    void sleep() const;

    Duration operator+(const Duration &rhs) const;

    Duration operator-(const Duration &rhs) const;

    Duration operator-() const;

    Duration operator*(double scale) const;

    Duration &operator+=(const Duration &rhs);

    Duration &operator-=(const Duration &rhs);

    Duration &operator*=(double scale);

    bool operator==(const Duration &rhs) const;

    bool operator!=(const Duration &rhs) const;

    bool operator>(const Duration &rhs) const;

    bool operator<(const Duration &rhs) const;

    bool operator>=(const Duration &rhs) const;

    bool operator<=(const Duration &rhs) const;

private:
    int64_t nano_seconds_ = 0;
};

std::ostream &operator<<(std::ostream &os, const Duration &rhs);

}  // namespace rally

#endif  // RALLY_UTILS_TIMER_DURATION_H
