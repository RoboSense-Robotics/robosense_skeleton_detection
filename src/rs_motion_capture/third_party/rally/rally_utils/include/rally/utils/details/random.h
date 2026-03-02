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
#ifndef RALLY_UTILS_DETAILS_RANDOM_H
#define RALLY_UTILS_DETAILS_RANDOM_H

#include "rally/utils/timer/timer.h"

namespace rally {

class Random {
public:
    Random() noexcept {
        srand(static_cast<uint32_t>(rally::getNowInNanoSeconds()));
    }

    double randomDouble(double s, double t) const noexcept {
        return s + (t - s) / 16383.0 * (std::rand() & 16383);
    }

    float radomFloat(float s, float t) const noexcept {
        return s + (t - s) / 16383.f * (std::rand() & 16383);
    }

    int32_t randomInt(int32_t s, int32_t t) const noexcept {
        if (s >= t) {
            return s;
        }
        return std::move(s + std::rand() % (t - s + 1));
    }

    static Random &getInstance() noexcept {
        return rally::Singleton<rally::Random>::getInstance();
    }

private:
};

}  // namespace rally

#endif  // RALLY_UTILS_DETAILS_RANDOM_H
