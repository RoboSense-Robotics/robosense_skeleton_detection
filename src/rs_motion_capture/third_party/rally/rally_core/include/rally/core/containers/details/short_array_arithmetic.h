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
#ifndef RALLY_CORE_CONTAINERS_DETAILS_SHORT_ARRAY_ARITHMETIC_H
#define RALLY_CORE_CONTAINERS_DETAILS_SHORT_ARRAY_ARITHMETIC_H

namespace rally {

template <class T, uint32_t Dim>
inline std::ostream &operator<<(std::ostream &os, const ShortArray<T, Dim> &p) {
    os << p.infos();
    return os;
}

/// @brief Exactly compares two vectors.
template <class T, uint32_t Dim>
bool operator==(const ShortArray<T, Dim>& lhs, const ShortArray<T, Dim>& rhs) {
    bool same = rally::isEqual(lhs.raw_data[0], rhs.raw_data[0]);
    for (uint32_t i = 1; i < Dim; ++i) {
        same = same && rally::isEqual(lhs.raw_data[i], rhs.raw_data[i]);
    }
    return same;
}

/// @brief Exactly compares two vectors.
template <class T, uint32_t Dim>
bool operator!=(const ShortArray<T, Dim>& lhs, const ShortArray<T, Dim>& rhs) {
    return !operator==(lhs, rhs);
}

/// @brief
template<class T, uint32_t Dim>
inline ShortArray <T, Dim> operator*(const ShortArray <T, Dim> &lhs, const ShortArray <T, Dim> &rhs) {
    ShortArray<T, Dim> result;
    for (uint32_t i{0}; i < Dim; ++i) {
        result[i] = lhs.raw_data[i] * rhs.raw_data[i];
    }
    return result;
}

/// @brief Elementwise vector division.
template<class T, uint32_t Dim>
inline ShortArray <T, Dim> operator/(const ShortArray <T, Dim> &lhs, const ShortArray <T, Dim> &rhs) {
    ShortArray<T, Dim> result;
    for (uint32_t i{0}; i < Dim; ++i) {
        result[i] = lhs.raw_data[i] / rhs.raw_data[i];
    }
    return result;
}

template<class T, uint32_t Dim>
inline ShortArray <T, Dim> operator+(const ShortArray <T, Dim> &lhs, const ShortArray <T, Dim> &rhs) {
    ShortArray<T, Dim> result;
    for (uint32_t i{0}; i < Dim; ++i) {
        result[i] = lhs.raw_data[i] + rhs.raw_data[i];
    }
    return result;
}

template<class T, uint32_t Dim>
inline ShortArray <T, Dim> operator-(const ShortArray <T, Dim> &lhs, const ShortArray <T, Dim> &rhs) {
    ShortArray<T, Dim> result;
    for (uint32_t i{0}; i < Dim; ++i) {
        result[i] = lhs.raw_data[i] - rhs.raw_data[i];
    }
    return result;
}

//------------------------------------------------------------------------------
// ShortArray assign arithmetic
//------------------------------------------------------------------------------

template<class T, uint32_t Dim>
inline ShortArray <T, Dim> &operator*=(ShortArray <T, Dim> &lhs, const ShortArray <T, Dim> &rhs) {
    for (uint32_t i{0}; i < Dim; ++i) {
        lhs.raw_data[i] *= rhs.raw_data[i];
    }
    return lhs;
}

template<class T, uint32_t Dim>
inline ShortArray <T, Dim> &operator/=(ShortArray <T, Dim> &lhs, const ShortArray <T, Dim> &rhs) {
    for (uint32_t i{0}; i < Dim; ++i) {
        lhs.raw_data[i] /= rhs.raw_data[i];
    }
    return lhs;
}

template<class T, uint32_t Dim>
inline ShortArray <T, Dim> &operator+=(ShortArray <T, Dim> &lhs, const ShortArray <T, Dim> &rhs) {
    for (uint32_t i{0}; i < Dim; ++i) {
        lhs.raw_data[i] += rhs.raw_data[i];
    }
    return lhs;
}

template<class T, uint32_t Dim>
inline ShortArray <T, Dim> &operator-=(ShortArray <T, Dim> &lhs, const ShortArray <T, Dim> &rhs) {
    for (uint32_t i{0}; i < Dim; ++i) {
        lhs.raw_data[i] -= rhs.raw_data[i];
    }
    return lhs;
}

//------------------------------------------------------------------------------
// Scalar assign arithmetic
//------------------------------------------------------------------------------

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> &operator*=(ShortArray <T, Dim> &lhs, U rhs) {
    for (uint32_t i{0}; i < Dim; ++i) {
        lhs.raw_data[i] *= rhs;
    }
    return lhs;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> &operator/=(ShortArray <T, Dim> &lhs, U rhs) {
    for (uint32_t i{0}; i < Dim; ++i) {
        lhs.raw_data[i] /= rhs;
    }
    return lhs;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> &operator+=(ShortArray <T, Dim> &lhs, U rhs) {
    for (uint32_t i{0}; i < Dim; ++i) {
        lhs.raw_data[i] += rhs;
    }
    return lhs;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> &operator-=(ShortArray <T, Dim> &lhs, U rhs) {
    for (uint32_t i{0}; i < Dim; ++i) {
        lhs.raw_data[i] -= rhs;
    }
    return lhs;
}


//------------------------------------------------------------------------------
// Scalar arithmetic
//------------------------------------------------------------------------------
template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> operator*(const ShortArray <T, Dim> &lhs, U rhs) {
    ShortArray<T, Dim> copy(lhs);
    copy *= rhs;
    return copy;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> operator/(const ShortArray <T, Dim> &lhs, U rhs) {
    ShortArray<T, Dim> copy(lhs);
    copy /= rhs;
    return copy;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> operator+(const ShortArray <T, Dim> &lhs, U rhs) {
    ShortArray<T, Dim> copy(lhs);
    copy += rhs;
    return copy;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> operator-(const ShortArray <T, Dim> &lhs, U rhs) {
    ShortArray<T, Dim> copy(lhs);
    copy -= rhs;
    return copy;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> operator*(U lhs, const ShortArray <T, Dim> &rhs) {
    return rhs * lhs;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> operator+(U lhs, const ShortArray <T, Dim> &rhs) {
    return rhs + lhs;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> operator-(U lhs, const ShortArray <T, Dim> &rhs) {
    return ShortArray<T, Dim>(lhs) - rhs;
}

template<class T, uint32_t Dim, class U, class = typename std::enable_if<std::is_convertible<U, T>::value>::type>
inline ShortArray <T, Dim> operator/(U lhs, const ShortArray <T, Dim> &rhs) {
    ShortArray<T, Dim> copy(lhs);
    copy /= rhs;
    return copy;
}


//------------------------------------------------------------------------------
// Extra
//------------------------------------------------------------------------------

template<class T, uint32_t Dim>
inline ShortArray <T, Dim>
MultiplyAdd(const ShortArray <T, Dim> &a, const ShortArray <T, Dim> &b, const ShortArray <T, Dim> &c) {
    return a * b + c;
}

template<class T, uint32_t Dim>
inline ShortArray <T, Dim> operator-(const ShortArray <T, Dim> &arg) {
    return arg * T(-1);
}

template<class T, uint32_t Dim>
inline ShortArray <T, Dim> operator+(const ShortArray <T, Dim> &arg) {
    return arg;
}

}  // namespace rally

#endif  // RALLY_CORE_CONTAINERS_DETAILS_SHORT_ARRAY_ARITHMETIC_H
