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
#ifndef RALLY_CORE_CONTAINERS_DETAILS_ARRAY_DATA_H
#define RALLY_CORE_CONTAINERS_DETAILS_ARRAY_DATA_H

#include "rally/utils/utils.h"

namespace rally {

/// @brief General
template <class T, uint32_t Dim>
class ArrayData {
public:
    /// @brief Raw array containing the elements.
    T raw_data[Dim];
};

/// @brief Small vectors with x,y members
template <class T>
class ArrayData<T, 2> {
public:
    ArrayData() {}
    ArrayData(const ArrayData& rhs) {
        for (int i = 0; i < 2; ++i) {
            raw_data[i] = rhs.raw_data[i];
        }
    }
    ArrayData& operator=(const ArrayData& rhs) {
        for (int i = 0; i < 2; ++i) {
            raw_data[i] = rhs.raw_data[i];
        }
        return *this;
    }
    union {
        struct {
            T x, y;
        };
        /// @brief Raw array containing the elements.
        T raw_data[2];
    };
};

/// @brief Small vectors with x,y,z members
template <class T>
class ArrayData<T, 3> {
public:
    ArrayData() {}
    ArrayData(const ArrayData& rhs) {
        for (int i = 0; i < 3; ++i) {
            raw_data[i] = rhs.raw_data[i];
        }
    }
    ArrayData& operator=(const ArrayData& rhs) {
        for (int i = 0; i < 3; ++i) {
            raw_data[i] = rhs.raw_data[i];
        }
        return *this;
    }
    union {
        struct {
            T x, y, z;
        };
        /// @brief Raw array containing the elements.
        T raw_data[3];
    };
};

/// @brief Small vectors with x,y,z,w members
template <class T>
class ArrayData<T, 4> {
public:
    ArrayData() {}
    ArrayData(const ArrayData& rhs) {
        for (int i = 0; i < 4; ++i) {
            raw_data[i] = rhs.raw_data[i];
        }
    }
    ArrayData& operator=(const ArrayData& rhs) {
        for (int i = 0; i < 4; ++i) {
            raw_data[i] = rhs.raw_data[i];
        }
        return *this;
    }
    union {
        struct {
            T x, y, z, w;
        };
        /// @brief Raw array containing the elements.
        T raw_data[4];
    };
};

}  // namespace rally

#endif  // RALLY_CORE_CONTAINERS_DETAILS_ARRAY_DATA_H
