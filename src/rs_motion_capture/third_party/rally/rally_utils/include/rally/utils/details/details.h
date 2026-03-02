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
#ifndef RALLY_UTILS_DETAILS_DETAILS_H
#define RALLY_UTILS_DETAILS_DETAILS_H

#include "rally/utils/details/macros.h"
#include "rally/utils/details/defines.h"
#include "rally/utils/details/enum.h"
#include "rally/utils/details/equal.h"
#include "rally/utils/details/throw.h"
#include "rally/utils/details/random.h"

namespace rally {

inline int32_t randomInt(const int32_t s, const int32_t t) {
    return std::move(Random::getInstance().randomInt(s, t));
}

inline double randomDouble(const double s, const double t) {
    return std::move(Random::getInstance().randomDouble(s, t));
}

inline float randomFloat(const float s, const float t) {
    return std::move(Random::getInstance().radomFloat(s, t));
}

}  // namespace rally

#endif  // RALLY_UTILS_DETAILS_DETAILS_H
