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

#include "rally/core/algorithm/geometry/details/sin_table.h"
#include "rally/core/algorithm/geometry/angle.h"

namespace rally {

float sin(Angle16 a) {
    int16_t idx = a.raw();

    if (idx < -Angle16::RAW_PI_2) {
        idx = static_cast<int16_t>(idx + Angle16::RAW_PI);
        return -SIN_TABLE[idx % SIN_TABLE_SIZE];
    }
    if (idx < 0) {
        return -SIN_TABLE[(-idx) % SIN_TABLE_SIZE];
    }
    if (idx < Angle16::RAW_PI_2) {
        return SIN_TABLE[idx % SIN_TABLE_SIZE];
    }
    idx = static_cast<int16_t>(Angle16::RAW_PI - idx);
    return SIN_TABLE[idx % SIN_TABLE_SIZE];
}

float cos(Angle16 a) {
    Angle16 b(static_cast<int16_t>(Angle16::RAW_PI_2 - a.raw()));
    return sin(b);
}

float tan(Angle16 a) { return sin(a) / cos(a); }

float sin(Angle8 a) {
    Angle16 b(static_cast<int16_t>(a.raw() << 8));
    return sin(b);
}

float cos(Angle8 a) {
    Angle16 b(static_cast<int16_t>(a.raw() << 8));
    return cos(b);
}

float tan(Angle8 a) {
    Angle16 b(static_cast<int16_t>(a.raw() << 8));
    return tan(b);
}

}  // namespace rally
