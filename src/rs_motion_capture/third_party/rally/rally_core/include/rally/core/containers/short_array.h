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
#ifndef RALLY_CORE_CONTAINERS_SHORT_ARRAY_H
#define RALLY_CORE_CONTAINERS_SHORT_ARRAY_H

#include <cmath>

#include "rally/core/containers/details/array_data.h"
#include "rally/core/meta/future.h"
#include "rally/utils/utils.h"

namespace rally {

/// @brief Represents a vector in N-dimensional space.
/// @tparam T The scalar type on which the vector is based.
///           You can use builtin floating point or integer types. User-defined types and std::complex
///            may also work, but are not yet officially supported.
/// @tparam Dim The dimension of the vector-space. Must be a positive integer.
/// @details There is not much extraordinary to vectors, they work as you would expect.
///          you can use common vector space airhtmetic
///          you have common function like normalization
///          you can multiply them with Matrix from either side
template<class T, uint32_t Dim>
class ShortArray : public ArrayData<T, Dim> {
    RALLY_STATIC_ASSERT(Dim >= 1);
public:
    using ArrayData<T, Dim>::raw_data;

    /// @brief Returns the number of dimensions of the vector.
    constexpr uint32_t getDimension() const noexcept {
        return Dim;
    }

    /// @brief Sets all elements to the 0.
    ShortArray() {
        for (auto &v : this->raw_data) {
            v = static_cast<T>(0);
        }
    }

    ShortArray(const ShortArray&) = default;
    ShortArray& operator=(const ShortArray&) = default;

    template <typename U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
    explicit ShortArray(const ShortArray<U, Dim>& v) {
        for (uint32_t i = 0; i < Dim; ++i) {
            data()[i] = static_cast<T>(v.data()[i]);
        }
    }

    /// @brief Sets all elements to the same value.
    explicit ShortArray(T all) {
        for (auto &v : this->raw_data) {
            v = all;
        }
    }

    ShortArray(T s1, T s2) {
        RALLY_STATIC_ASSERT(Dim == 2);
        this->raw_data[0] = s1;
        this->raw_data[1] = s2;
    }

    ShortArray(T s1, T s2, T s3) {
        RALLY_STATIC_ASSERT(Dim == 3);
        this->raw_data[0] = s1;
        this->raw_data[1] = s2;
        this->raw_data[2] = s3;
    }

    ShortArray(const ShortArray<T, 2>& v12, T s3) {
        RALLY_STATIC_ASSERT(Dim == 3);
        this->raw_data[0] = v12[0];
        this->raw_data[1] = v12[1];
        this->raw_data[2] = s3;
    }

    ShortArray(T s1, T s2, T s3, T s4) {
        RALLY_STATIC_ASSERT(Dim == 4);
        this->raw_data[0] = s1;
        this->raw_data[1] = s2;
        this->raw_data[2] = s3;
        this->raw_data[3] = s4;
    }

    ShortArray(const ShortArray<T, 3>& vector123, T s4) {
        RALLY_STATIC_ASSERT(Dim == 4);
        this->raw_data[0] = vector123[0];
        this->raw_data[1] = vector123[1];
        this->raw_data[2] = vector123[2];
        this->raw_data[3] = s4;
    }

    ShortArray(const ShortArray<T, 2>& v12, const ShortArray<T, 2>& v34) {
        RALLY_STATIC_ASSERT(Dim == 4);
        this->raw_data[0] = v12[0];
        this->raw_data[1] = v12[1];
        this->raw_data[2] = v34[0];
        this->raw_data[3] = v34[1];
    }

    /// @brief returns the nth element
    T operator[](uint32_t idx) const {
        RENSURE(idx < getDimension());
        return raw_data[idx];
    }

    /// @brief returns the nth element
    T &operator[](uint32_t idx) {
        RENSURE(idx < getDimension());
        return raw_data[idx];
    }

    /// @brief Returns the nth element of the vector.
    T operator()(uint32_t idx) const {
        RENSURE(idx < getDimension());
        return raw_data[idx];
    }

    /// @brief Returns the nth element of the vector.
    T &operator()(uint32_t idx) {
        RENSURE(idx < getDimension());
        return raw_data[idx];
    }

    /// @brief Returns an iterator to the first element.
    const T *cbegin() const noexcept {
        return raw_data;
    }

    /// @brief Returns an iterator to the first element.
    const T *begin() const noexcept {
        return raw_data;
    }

    /// @brief Returns an iterator to the first element.
    T *begin() noexcept {
        return raw_data;
    }

    /// @brief Returns an iterator to the end of the vector (works like STL).
    const T *cend() const noexcept {
        return raw_data + Dim;
    }

    /// @brief Returns an iterator to the end of the vector (works like STL).
    const T *end() const noexcept {
        return raw_data + Dim;
    }

    /// @brief Returns an iterator to the end of the vector (works like STL).
    T *end() noexcept {
        return raw_data + Dim;
    }

    /// @brief Returns a pointer to the underlying array of elements.
    const T *data() const noexcept {
        return raw_data;
    }

    /// @brief Returns a pointer to the underlying array of elements.
    T *data() noexcept {
        return raw_data;
    }

    void fill(const T& all) {
        for (auto& v : *this) {
            v = all;
        }
    }

    /// @brief get the scalar product (dot product)
    /// @return return the value of dot product
    T dot(const ShortArray<T, Dim> &rhs) const noexcept {
        T sum = static_cast<T>(0);
        for (uint32_t i = 0; i < Dim; ++i) {
            sum += this->raw_data[i] * rhs.raw_data[i];
        }
        return sum;
    }

    /// @brief get the norm value of the vector
    /// @return return the norm value
    T norm() const noexcept {
        T sum = static_cast<T>(0);
        for (uint32_t i = 0; i < Dim; ++i) {
            sum += this->raw_data[i] * this->raw_data[i];
        }
        if (isEqual(sum, static_cast<T>(0))) {
            return static_cast<T>(0);
        } else {
            return std::sqrt(sum);
        }
    }

    /// @brief Makes a unit vector, but keeps direction.
    void normalize() noexcept {
        T sum = norm();
        if (isEqual(sum, static_cast<T>(0))) {
            return;
        } else {
            for (uint32_t i = 0; i < Dim; ++i) {
                this->raw_data[i] /= sum;
            }
        }
    }

    std::string const name() const noexcept {
        return std::to_string(getDimension()) + "-D Array";
    }

    std::string const infos() const noexcept {
        std::stringstream ss;
        ss << name() << "(";
        for (uint32_t i = 0; i < Dim; ++i) {
            ss << this->raw_data[i];
            if (i != Dim - 1) {
                ss << ",";
            }
        }
        ss << ")";
        return ss.str();
    }

};

using ShortArray1i = ShortArray<int32_t, 1>;
using ShortArray2i = ShortArray<int32_t, 2>;
using ShortArray3i = ShortArray<int32_t, 3>;
using ShortArray4i = ShortArray<int32_t, 4>;

using ShortArray1ui = ShortArray<uint32_t, 1>;
using ShortArray2ui = ShortArray<uint32_t, 2>;
using ShortArray3ui = ShortArray<uint32_t, 3>;
using ShortArray4ui = ShortArray<uint32_t, 4>;

using ShortArray1f = ShortArray<float, 1>;
using ShortArray2f = ShortArray<float, 2>;
using ShortArray3f = ShortArray<float, 3>;
using ShortArray4f = ShortArray<float, 4>;

/// @brief Returns the 3-dimensional cross-product.
inline ShortArray3f cross(const ShortArray3f &lhs, const ShortArray3f &rhs) {
    return ShortArray3f(lhs.y * rhs.z - lhs.z * rhs.y,
                        lhs.z * rhs.x - lhs.x * rhs.z,
                        lhs.x * rhs.y - lhs.y * rhs.x);
}

/// @brief Calculates the scalar product (dot product) of the two arguments.
template<class T, uint32_t Dim>
inline T dot(const ShortArray<T, Dim> &lhs, const ShortArray<T, Dim> &rhs) noexcept {
    T sum = static_cast<T>(0);
    for (uint32_t i = 0; i < Dim; ++i) {
        sum += lhs.raw_data[i] * rhs.raw_data[i];
    }
    return sum;
};

}  // namespace rally

#include "rally/core/containers/details/short_array_2f_arithmetic.h"
#include "rally/core/containers/details/short_array_arithmetic.h"

#endif  // RALLY_CORE_CONTAINERS_SHORT_ARRAY_H
