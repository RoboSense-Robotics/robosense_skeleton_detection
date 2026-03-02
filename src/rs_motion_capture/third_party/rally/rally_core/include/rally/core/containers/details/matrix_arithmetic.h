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
#ifndef RALLY_CORE_CONTAINERS_DETAILS_MATRIX_ARITHMETIC_H
#define RALLY_CORE_CONTAINERS_DETAILS_MATRIX_ARITHMETIC_H

namespace rally {

template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
inline std::ostream &operator<<(std::ostream &os, const Matrix <T, Rows, Columns, Order, Layout> &p) {
    os << p.infos();
    return os;
}

template<uint32_t Rows, uint32_t Columns, class T1, class T2, eMatrixOrder Order1,
eMatrixOrder Order2, eMatrixLayout Layout1, eMatrixLayout Layout2>
bool operator==(const Matrix <T1, Rows, Columns, Order1, Layout1> &lhs,
                const Matrix <T2, Rows, Columns, Order2, Layout2> &rhs) {
    bool equal = true;
    for (uint32_t i = 0; i < Rows; ++i) {
        for (uint32_t j = 0; j < Columns; ++j) {
            equal = equal && rally::isEqual(lhs(i, j), rhs(i, j));
        }
    }
    return equal;
}

template<uint32_t Rows, uint32_t Columns, class T1, class T2, eMatrixOrder Order1,
eMatrixOrder Order2, eMatrixLayout Layout1, eMatrixLayout Layout2>
bool operator!=(const Matrix <T1, Rows, Columns, Order1, Layout1> &lhs,
                const Matrix <T2, Rows, Columns, Order2, Layout2> &rhs) {
    return !(lhs == rhs);
}

//------------------------------------------------------------------------------
// Matrix-matrix multiplication
//------------------------------------------------------------------------------

template<class T, class U, uint32_t Rows1, uint32_t Match, uint32_t Columns2, eMatrixOrder Order>
inline auto operator*(const Matrix <T, Rows1, Match, Order, eMatrixLayout::ROW_MAJOR> &lhs,
                      const Matrix <U, Match, Columns2, Order, eMatrixLayout::ROW_MAJOR> &rhs) {
    using V = traits::MatMulElemT<T, U>;
    Matrix <V, Rows1, Columns2, Order, eMatrixLayout::ROW_MAJOR> result;
    for (uint32_t i = 0; i < Rows1; ++i) {
        result.stripes[i] = rhs.stripes[0] * lhs(i, 0);
    }
    for (uint32_t i = 0; i < Rows1; ++i) {
        for (uint32_t j = 1; j < Match; ++j) {
            result.stripes[i] += rhs.stripes[j] * lhs(i, j);
        }
    }
    return result;
}

template<class T, class U, uint32_t Rows1, uint32_t Match, uint32_t Columns2, eMatrixOrder Order>
inline auto operator*(const Matrix <T, Rows1, Match, Order, eMatrixLayout::ROW_MAJOR> &lhs,
                      const Matrix <U, Match, Columns2, Order, eMatrixLayout::COLUMN_MAJOR> &rhs) {
    using V = traits::MatMulElemT<T, U>;
    Matrix <V, Rows1, Columns2, Order, eMatrixLayout::ROW_MAJOR> result;

    for (uint32_t j = 0; j < Columns2; ++j) {
        for (uint32_t i = 0; i < Rows1; ++i) {
            result(i, j) = dot(lhs.stripes[i], rhs.stripes[j]);
        }
    }

    return result;
}

template<class T, class U, uint32_t Rows1, uint32_t Match, uint32_t Columns2, eMatrixOrder Order>
inline auto operator*(const Matrix <T, Rows1, Match, Order, eMatrixLayout::COLUMN_MAJOR> &lhs,
                      const Matrix <U, Match, Columns2, Order, eMatrixLayout::COLUMN_MAJOR> &rhs) {
    using V = traits::MatMulElemT<T, U>;
    Matrix <V, Rows1, Columns2, Order, eMatrixLayout::COLUMN_MAJOR> result;
    for (uint32_t j = 0; j < Columns2; ++j) {
        result.stripes[j] = lhs.stripes[0] * rhs(0, j);
    }
    for (uint32_t i = 1; i < Match; ++i) {
        for (uint32_t j = 0; j < Columns2; ++j) {
            result.stripes[j] += lhs.stripes[i] * rhs(i, j);
        }
    }
    return result;
}

template<class T, class U, uint32_t Rows1, uint32_t Match, uint32_t Columns2, eMatrixOrder Order>
inline auto operator*(const Matrix <T, Rows1, Match, Order, eMatrixLayout::COLUMN_MAJOR> &lhs,
                      const Matrix <U, Match, Columns2, Order, eMatrixLayout::ROW_MAJOR> &rhs) {
    // CC algorithm is completely fine for COL_MAJOR x ROW_MAJOR.
    // See that rhs is only indexed per-element, so its layout does not matter.
    using V = traits::MatMulElemT<T, U>;
    Matrix <V, Rows1, Columns2, Order, eMatrixLayout::COLUMN_MAJOR> result;
    for (uint32_t j = 0; j < Columns2; ++j) {
        result.stripes[j] = lhs.stripes[0] * rhs(0, j);
    }
    for (uint32_t i = 1; i < Match; ++i) {
        for (uint32_t j = 0; j < Columns2; ++j) {
            result.stripes[j] += lhs.stripes[i] * rhs(i, j);
        }
    }
    return result;
}


// Assign-multiply
template<class T1, class T2, uint32_t Dim, eMatrixOrder Order, eMatrixLayout Layout1, eMatrixLayout Layout2>
inline Matrix <T1, Dim, Dim, Order, Layout1> &operator*=(Matrix <T1, Dim, Dim, Order, Layout1> &lhs,
                                                         const Matrix <T2, Dim, Dim, Order, Layout2> &rhs) {
    lhs = lhs * rhs;
    return lhs;
}

//------------------------------------------------------------------------------
// Matrix-matrix addition & subtraction
//------------------------------------------------------------------------------

// Same layout
template<class T, class U, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout SameLayout>
inline auto operator+(const Matrix <T, Rows, Columns, Order, SameLayout> &lhs,
                      const Matrix <U, Rows, Columns, Order, SameLayout> &rhs) {
    using V = traits::MatMulElemT<T, U>;

    if (Rows * Columns == 4) {
        Matrix <V, Rows, Columns, Order, SameLayout> result;
        for (uint32_t i = 0; i < result.getRowCount(); ++i) {
            for (uint32_t j = 0; j < result.getColumnCount(); ++j) {
                result(i, j) = lhs(i, j) + rhs(i, j);
            }
        }
        return result;
    } else {
        Matrix <V, Rows, Columns, Order, SameLayout> result;
        for (uint32_t i = 0; i < result.StripeCount; ++i) {
            result.stripes[i] = lhs.stripes[i] + rhs.stripes[i];
        }
        return result;
    }
}

template<class T, class U, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout SameLayout>
inline auto operator-(const Matrix <T, Rows, Columns, Order, SameLayout> &lhs,
                      const Matrix <U, Rows, Columns, Order, SameLayout> &rhs) {
    using V = traits::MatMulElemT<T, U>;

    if (Rows * Columns == 4) {
        Matrix <V, Rows, Columns, Order, SameLayout> result;
        for (uint32_t i = 0; i < result.getRowCount(); ++i) {
            for (uint32_t j = 0; j < result.getColumnCount(); ++j) {
                result(i, j) = lhs(i, j) - rhs(i, j);
            }
        }
        return result;
    } else {
        Matrix <V, Rows, Columns, Order, SameLayout> result;
        for (uint32_t i = 0; i < result.StripeCount; ++i) {
            result.stripes[i] = lhs.stripes[i] - rhs.stripes[i];
        }
        return result;
    }
}


// Add & sub opposite layout
template<class T, class U, uint32_t Rows, uint32_t Columns, eMatrixOrder Order,
eMatrixLayout Layout1, eMatrixLayout Layout2,
class = typename std::enable_if<Layout1 != Layout2>::type>
inline auto operator+(const Matrix <T, Rows, Columns, Order, Layout1> &lhs,
                      const Matrix <U, Rows, Columns, Order, Layout2> &rhs) {
    using V = traits::MatMulElemT<T, U>;
    Matrix <V, Rows, Columns, Order, Layout1> result;
    for (uint32_t i = 0; i < result.getRowCount(); ++i) {
        for (uint32_t j = 0; j < result.getColumnCount(); ++j) {
            result(i, j) = lhs(i, j) + rhs(i, j);
        }
    }
    return result;
}

template<class T, class U, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout1,
eMatrixLayout Layout2, class = typename std::enable_if<Layout1 != Layout2>::type>
inline auto operator-(const Matrix <T, Rows, Columns, Order, Layout1> &lhs,
                      const Matrix <U, Rows, Columns, Order, Layout2> &rhs) {
    using V = traits::MatMulElemT<T, U>;
    Matrix <V, Rows, Columns, Order, Layout1> result;
    for (uint32_t i = 0; i < result.getRowCount(); ++i) {
        for (uint32_t j = 0; j < result.getColumnCount(); ++j) {
            result(i, j) = lhs(i, j) - rhs(i, j);
        }
    }
    return result;
}


/// @brief Performs matrix addition and stores result in this.
template<class T, class U, uint32_t Rows, uint32_t Columns, eMatrixOrder Order,
eMatrixLayout Layout1, eMatrixLayout Layout2>
inline Matrix <U, Rows, Columns, Order, Layout1> &operator+=(Matrix <T, Rows, Columns, Order, Layout1> &lhs,
                                                             const Matrix <U, Rows, Columns, Order, Layout2> &rhs) {
    lhs = lhs + rhs;
    return lhs;
}

/// @brief Performs matrix subtraction and stores result in this.
template<class T, class U, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout1,
eMatrixLayout Layout2>
inline Matrix <U, Rows, Columns, Order, Layout1> &operator-=(Matrix <T, Rows, Columns, Order, Layout1> &lhs,
                                                             const Matrix <U, Rows, Columns, Order, Layout2> &rhs) {
    lhs = lhs - rhs;
    return lhs;
}

//------------------------------------------------------------------------------
// Matrix-Scalar arithmetic
//------------------------------------------------------------------------------

/// @brief Multiplies all elements of the matrix by scalar.
template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout,
class U, std::enable_if_t<std::is_convertible<U, T>::value, uint32_t> = 0>
inline Matrix <T, Rows, Columns, Order, Layout> &operator*=(Matrix <T, Rows, Columns, Order, Layout> &mat, U s) {
    for (auto &stripe : mat.stripes) {
        stripe *= s;
    }
    return mat;
}

/// @brief Divides all elements of the matrix by scalar.
template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout,
class U, std::enable_if_t<std::is_convertible<U, T>::value, uint32_t> = 0>
inline Matrix <T, Rows, Columns, Order, Layout> &
operator/=(Matrix <T, Rows, Columns, Order, Layout> &mat, U s) {
    mat *= U(1) / s;
    return mat;
}

template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout,
class U, std::enable_if_t<std::is_convertible<U, T>::value, uint32_t> = 0>
Matrix <T, Rows, Columns, Order, Layout>
operator*(const Matrix <T, Rows, Columns, Order, Layout> &mat, U s) {
    Matrix<T, Rows, Columns, Order, Layout> copy(mat);
    copy *= s;
    return copy;
}

template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout,
class U, std::enable_if_t<std::is_convertible<U, T>::value, uint32_t> = 0>
Matrix <T, Rows, Columns, Order, Layout>
operator/(const Matrix <T, Rows, Columns, Order, Layout> &mat, U s) {
    Matrix<T, Rows, Columns, Order, Layout> copy(mat);
    copy /= s;
    return copy;
}

template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout,
class U, std::enable_if_t<std::is_convertible<U, T>::value, uint32_t> = 0>
Matrix <T, Rows, Columns, Order, Layout>
operator*(U s, const Matrix <T, Rows, Columns, Order, Layout> &mat) {
    return mat * s;
}

template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout,
class U, std::enable_if_t<std::is_convertible<U, T>::value, uint32_t> = 0>
Matrix <T, Rows, Columns, Order, Layout>
operator/(U s, const Matrix <T, Rows, Columns, Order, Layout> &mat) {
    Matrix<T, Rows, Columns, Order, Layout> result;
    for (uint32_t i = 0; i < Matrix<T, Rows, Columns, Order, Layout>::StripeCount; ++i) {
        result.stripes[i] = T(s) / mat.stripes[i];
    }
    return result;
}

//------------------------------------------------------------------------------
// Elementwise multiply and divide
//------------------------------------------------------------------------------
template<class T, class T2, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
auto mulElementwise(const Matrix <T, Rows, Columns, Order, Layout> &lhs,
                    const Matrix <T2, Rows, Columns, Order, Layout> &rhs) {
    Matrix <T, Rows, Columns, Order, Layout> result;
    for (uint32_t i = 0; i < result.StripeCount; ++i) {
        result.stripes[i] = lhs.stripes[i] * rhs.stripes[i];
    }
    return result;
}

template<class T, class T2, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
auto mulElementwise(const Matrix <T, Rows, Columns, Order, Layout> &lhs,
                    const Matrix <T2, Rows, Columns, Order, traits::OppositeLayout<Layout>::value> &rhs) {
    Matrix <T, Rows, Columns, Order, Layout> result;
    for (uint32_t i = 0; i < Rows; ++i) {
        for (uint32_t j = 0; j < Columns; ++j) {
            result(i, j) = lhs(i, j) * rhs(i, j);
        }
    }
    return result;
}

template<class T, class T2, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
auto divElementwise(const Matrix <T, Rows, Columns, Order, Layout> &lhs,
                    const Matrix <T2, Rows, Columns, Order, Layout> &rhs) {
    Matrix <T, Rows, Columns, Order, Layout> result;
    for (uint32_t i{0}; i < result.StripeCount; ++i) {
        result.stripes[i] = lhs.stripes[i] / rhs.stripes[i];
    }
    return result;
}

template<class T, class T2, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
auto divElementwise(const Matrix <T, Rows, Columns, Order, Layout> &lhs,
                    const Matrix <T2, Rows, Columns, Order, traits::OppositeLayout<Layout>::value> &rhs) {
    Matrix <T, Rows, Columns, Order, Layout> result;
    for (uint32_t i = 0; i < Rows; ++i) {
        for (uint32_t j = 0; j < Columns; ++j) {
            result(i, j) = lhs(i, j) / rhs(i, j);
        }
    }
    return result;
}


//------------------------------------------------------------------------------
// Unary signs
//------------------------------------------------------------------------------
template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
auto operator+(const Matrix <T, Rows, Columns, Order, Layout> &mat) {
    return Matrix<T, Rows, Columns, Order, Layout>(mat);
}

template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
auto operator-(const Matrix <T, Rows, Columns, Order, Layout> &mat) {
    return Matrix<T, Rows, Columns, Order, Layout>(mat) * static_cast<T>(-1);
}


/// @brief Returns the determinant of a 2x2 matrix.
template<class T, eMatrixOrder Order, eMatrixLayout Layout>
T determinant(const Matrix<T, 2, 2, Order, Layout> &m) {
    return m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1);
}

/// @brief Returns the determinant of a 3x3matrix.
template<class T, eMatrixOrder Order, eMatrixLayout Layout>
T determinant(const Matrix<T, 3, 3, Order, Layout> &m) {
    using Vec3 = ShortArray<T, 3>;

    const auto &r0 = m.stripes[0];
    const auto &r1 = m.stripes[1];
    const auto &r2 = m.stripes[2];
    Vec3 r0_zyx = {r0.z, r0.y, r0.x};
    Vec3 r1_xzy = {r1.x, r1.z, r1.y};
    Vec3 r1_yxz = {r1.y, r1.x, r1.z};
    Vec3 r2_yxz = {r2.y, r2.x, r2.z};
    Vec3 r2_xzy = {r2.x, r2.z, r2.y};

    T det = dot(r0_zyx, r1_xzy * r2_yxz - r1_yxz * r2_xzy);

    return det;
}

/// @brief Returns the determinant of a 4x4matrix.
template<class T, eMatrixOrder Order, eMatrixLayout Layout>
T determinant(const Matrix<T, 4, 4, Order, Layout> &m) {
    using Vec3 = ShortArray<T, 3>;
    using Vec4 = ShortArray<T, 4>;

    Vec4 evenPair = {1, -1, -1, 1};
    Vec4 oddPair = {-1, 1, 1, -1};

    const Vec4 &r0 = m.stripes[0];
    const Vec4 &r1 = m.stripes[1];
    const Vec4 &r2 = m.stripes[2];
    const Vec4 &r3 = m.stripes[3];

    Vec4 r2_zwzw = {r2.z, r2.w, r2.z, r2.w};
    Vec4 r0_yyxx = {r0.y, r0.y, r0.x, r0.x};
    Vec4 r1_wwxy = {r1.w, r1.w, r1.x, r1.y};
    Vec4 r2_xyzz = {r2.x, r2.y, r2.z, r2.z};
    Vec4 r3_wwww = {r3.w, r3.w, r3.w, r3.w};
    Vec4 r1_zzxy = {r1.z, r1.z, r1.x, r1.y};
    Vec4 r0_yxyx = {r0.y, r0.x, r0.y, r0.x};
    Vec4 r3_xxyy = {r3.x, r3.x, r3.y, r3.y};
    Vec4 r1_wzwz = {r1.w, r1.z, r1.w, r1.z};
    Vec4 r2_xyww = {r2.x, r2.y, r2.w, r2.w};
    Vec4 r3_zzzz = {r3.z, r3.z, r3.z, r3.z};

    Vec3 r2_yxz = {r2.y, r2.x, r2.z};
    Vec3 r3_xzy = {r3.x, r3.z, r3.y};
    Vec3 r2_xzy = {r2.x, r2.z, r2.y};
    Vec3 r3_yxz = {r3.y, r3.x, r3.z};
    Vec3 r2_yxw = {r2.y, r2.x, r2.w};
    Vec3 r1_zyx = {r1.z, r1.y, r1.x};
    Vec3 r3_yxw = {r3.y, r3.x, r3.w};
    Vec3 r2_xwy = {r2.x, r2.w, r2.y};
    Vec3 r3_xwy = {r3.x, r3.w, r3.y};
    Vec3 r1_wyx = {r1.w, r1.y, r1.x};
    T r0_w = r0.w;
    T r0_z = r0.z;

    T det = dot(evenPair, r0_yyxx * r1_wzwz * r2_zwzw * r3_xxyy)
            + dot(oddPair, r0_yxyx * r1_wwxy * r2_xyww * r3_zzzz)
            + dot(evenPair, r0_yxyx * r1_zzxy * r2_xyzz * r3_wwww)
            + (r0_w * dot(r1_zyx, r2_yxz * r3_xzy - r2_xzy * r3_yxz))
            + (r0_z * dot(r1_wyx, r2_xwy * r3_yxw - r2_yxw * r3_xwy));

    return det;
}

/// @brief Returns the inverse of a 2x2 matrix.
template<class T, eMatrixOrder Order, eMatrixLayout Layout>
inline Matrix<T, 2, 2, Order, Layout> inverse(const Matrix<T, 2, 2, Order, Layout> &m) {
    Matrix<T, 2, 2, Order, Layout> result;

    const auto &r0 = m.stripes[0];
    const auto &r1 = m.stripes[1];

    result.stripes[0] = {r1.y, -r0.y};
    result.stripes[1] = {-r1.x, r0.x};

    auto det = static_cast<T>(1) / (r0.x * r1.y - r0.y * r1.x);
    result *= det;

    return result;
}


/// @brief Returns the inverse of a 3x3 matrix.
template<class T, eMatrixOrder Order, eMatrixLayout Layout>
inline Matrix<T, 3, 3, Order, Layout> inverse(const Matrix<T, 3, 3, Order, Layout> &m) {
    Matrix<T, 3, 3, Order, Layout> result;

    using Vec3 = ShortArray<T, 3>;

    // This code below uses notation for row-major matrices' stripes.
    // It, however, "magically" works for column-major layout as well.
    const auto &r0 = m.stripes[0];
    const auto &r1 = m.stripes[1];
    const auto &r2 = m.stripes[2];
    Vec3 r0_zxy = Vec3(r0.z, r0.x, r0.y);
    Vec3 r0_yzx = Vec3(r0.y, r0.z, r0.x);
    Vec3 r1_zxy = Vec3(r1.z, r1.x, r1.y);
    Vec3 r1_yzx = Vec3(r1.y, r1.z, r1.x);
    Vec3 r2_zxy = Vec3(r2.z, r2.x, r2.y);
    Vec3 r2_yzx = Vec3(r2.y, r2.z, r2.x);

    Vec3 c0 = r1_yzx * r2_zxy - r1_zxy * r2_yzx;
    Vec3 c1 = r0_zxy * r2_yzx - r0_yzx * r2_zxy;
    Vec3 c2 = r0_yzx * r1_zxy - r0_zxy * r1_yzx;

    Vec3 r0_zyx = Vec3(r0.z, r0.y, r0.x);
    Vec3 r1_xzy = Vec3(r1.x, r1.z, r1.y);
    Vec3 r1_yxz = Vec3(r1.y, r1.x, r1.z);
    Vec3 r2_yxz = Vec3(r2.y, r2.x, r2.z);
    Vec3 r2_xzy = Vec3(r2.x, r2.z, r2.y);

    result.stripes[0] = {c0[0], c1[0], c2[0]};
    result.stripes[1] = {c0[1], c1[1], c2[1]};
    result.stripes[2] = {c0[2], c1[2], c2[2]};

    T det = static_cast<T>(1) / dot(r0_zyx, r1_xzy * r2_yxz - r1_yxz * r2_xzy);

    result.stripes[0] *= det;
    result.stripes[1] *= det;
    result.stripes[2] *= det;

    return result;
}


/// @brief Returns the inverse of a 4x4 matrix.
template<class T, eMatrixOrder Order, eMatrixLayout Layout>
inline Matrix<T, 4, 4, Order, Layout> inverse(const Matrix<T, 4, 4, Order, Layout> &m) {
    Matrix<T, 4, 4, Order, Layout> result;

    using Vec3 = ShortArray<T, 3>;
    using Vec4 = ShortArray<T, 4>;

    Vec4 even = {1, -1, 1, -1};
    Vec4 odd = {-1, 1, -1, 1};
    Vec4 evenPair = {1, -1, -1, 1};
    Vec4 oddPair = {-1, 1, 1, -1};

    const Vec4 &r0 = m.stripes[0];
    const Vec4 &r1 = m.stripes[1];
    const Vec4 &r2 = m.stripes[2];
    const Vec4 &r3 = m.stripes[3];

    Vec4 r0_wwwz = {r0.w, r0.w, r0.w, r0.z};
    Vec4 r0_yxxx = {r0.y, r0.x, r0.x, r0.x};
    Vec4 r0_zzyy = {r0.z, r0.z, r0.y, r0.y};
    Vec4 r1_wwwz = {r1.w, r1.w, r1.w, r1.z};
    Vec4 r1_yxxx = {r1.y, r1.x, r1.x, r1.x};
    Vec4 r1_zzyy = {r1.z, r1.z, r1.y, r1.y};
    Vec4 r2_wwwz = {r2.w, r2.w, r2.w, r2.z};
    Vec4 r2_yxxx = {r2.y, r2.x, r2.x, r2.x};
    Vec4 r2_zzyy = {r2.z, r2.z, r2.y, r2.y};
    Vec4 r3_wwwz = {r3.w, r3.w, r3.w, r3.z};
    Vec4 r3_yxxx = {r3.y, r3.x, r3.x, r3.x};
    Vec4 r3_zzyy = {r3.z, r3.z, r3.y, r3.y};

    Vec4 r0_wwwz_r1_yxxx = r0_wwwz * r1_yxxx;
    Vec4 r0_wwwz_r1_zzyy = r0_wwwz * r1_zzyy;
    Vec4 r0_yxxx_r1_wwwz = r0_yxxx * r1_wwwz;
    Vec4 r0_yxxx_r1_zzyy = r0_yxxx * r1_zzyy;
    Vec4 r0_zzyy_r1_wwwz = r0_zzyy * r1_wwwz;
    Vec4 r0_zzyy_r1_yxxx = r0_zzyy * r1_yxxx;
    Vec4 r2_wwwz_r3_yxxx = r2_wwwz * r3_yxxx;
    Vec4 r2_wwwz_r3_zzyy = r2_wwwz * r3_zzyy;
    Vec4 r2_yxxx_r3_wwwz = r2_yxxx * r3_wwwz;
    Vec4 r2_yxxx_r3_zzyy = r2_yxxx * r3_zzyy;
    Vec4 r2_zzyy_r3_wwwz = r2_zzyy * r3_wwwz;
    Vec4 r2_zzyy_r3_yxxx = r2_zzyy * r3_yxxx;

    Vec4 c0 = odd * (r1_wwwz * r2_zzyy_r3_yxxx - r1_zzyy * r2_wwwz_r3_yxxx - r1_wwwz * r2_yxxx_r3_zzyy +
                     r1_yxxx * r2_wwwz_r3_zzyy + r1_zzyy * r2_yxxx_r3_wwwz - r1_yxxx * r2_zzyy_r3_wwwz);
    Vec4 c1 = even * (r0_wwwz * r2_zzyy_r3_yxxx - r0_zzyy * r2_wwwz_r3_yxxx - r0_wwwz * r2_yxxx_r3_zzyy +
                      r0_yxxx * r2_wwwz_r3_zzyy + r0_zzyy * r2_yxxx_r3_wwwz - r0_yxxx * r2_zzyy_r3_wwwz);
    Vec4 c2 = odd * (r0_wwwz_r1_zzyy * r3_yxxx - r0_zzyy_r1_wwwz * r3_yxxx - r0_wwwz_r1_yxxx * r3_zzyy +
                     r0_yxxx_r1_wwwz * r3_zzyy + r0_zzyy_r1_yxxx * r3_wwwz - r0_yxxx_r1_zzyy * r3_wwwz);
    Vec4 c3 = even * (r0_wwwz_r1_zzyy * r2_yxxx - r0_zzyy_r1_wwwz * r2_yxxx - r0_wwwz_r1_yxxx * r2_zzyy +
                      r0_yxxx_r1_wwwz * r2_zzyy + r0_zzyy_r1_yxxx * r2_wwwz - r0_yxxx_r1_zzyy * r2_wwwz);

    result.stripes[0] = {c0[0], c1[0], c2[0], c3[0]};
    result.stripes[1] = {c0[1], c1[1], c2[1], c3[1]};
    result.stripes[2] = {c0[2], c1[2], c2[2], c3[2]};
    result.stripes[3] = {c0[3], c1[3], c2[3], c3[3]};

    Vec4 r2_zwzw = {r2.z, r2.w, r2.z, r2.w};
    Vec4 r0_yyxx = {r0.y, r0.y, r0.x, r0.x};
    Vec4 r1_wwxy = {r1.w, r1.w, r1.x, r1.y};
    Vec4 r2_xyzz = {r2.x, r2.y, r2.z, r2.z};
    Vec4 r3_wwww = {r3.w, r3.w, r3.w, r3.w};
    Vec4 r1_zzxy = {r1.z, r1.z, r1.x, r1.y};
    Vec4 r0_yxyx = {r0.y, r0.x, r0.y, r0.x};
    Vec4 r3_xxyy = {r3.x, r3.x, r3.y, r3.y};
    Vec4 r1_wzwz = {r1.w, r1.z, r1.w, r1.z};
    Vec4 r2_xyww = {r2.x, r2.y, r2.w, r2.w};
    Vec4 r3_zzzz = {r3.z, r3.z, r3.z, r3.z};

    Vec3 r2_yxz = {r2.y, r2.x, r2.z};
    Vec3 r3_xzy = {r3.x, r3.z, r3.y};
    Vec3 r2_xzy = {r2.x, r2.z, r2.y};
    Vec3 r3_yxz = {r3.y, r3.x, r3.z};
    Vec3 r2_yxw = {r2.y, r2.x, r2.w};
    Vec3 r1_zyx = {r1.z, r1.y, r1.x};
    Vec3 r3_yxw = {r3.y, r3.x, r3.w};
    Vec3 r2_xwy = {r2.x, r2.w, r2.y};
    Vec3 r3_xwy = {r3.x, r3.w, r3.y};
    Vec3 r1_wyx = {r1.w, r1.y, r1.x};
    T r0_w = r0.w;
    T r0_z = r0.z;

    T det = dot(evenPair, r0_yyxx * r1_wzwz * r2_zwzw * r3_xxyy)
            + dot(oddPair, r0_yxyx * r1_wwxy * r2_xyww * r3_zzzz)
            + dot(evenPair, r0_yxyx * r1_zzxy * r2_xyzz * r3_wwww)
            + (r0_w * dot(r1_zyx, r2_yxz * r3_xzy - r2_xzy * r3_yxz))
            + (r0_z * dot(r1_wyx, r2_xwy * r3_yxw - r2_yxw * r3_xwy));

    T invDet = 1 / det;

    result.stripes[0] *= invDet;
    result.stripes[1] *= invDet;
    result.stripes[2] *= invDet;
    result.stripes[3] *= invDet;

    return result;
}

/// @brief Calculates the square of the Frobenius norm of the matrix.
template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
inline T getNormSquared(const Matrix <T, Rows, Columns, Order, Layout> &m) {
    T sum = T(0);
    for (auto &stripe : m.stripes) {
        sum += stripe.dot();
    }
    return sum;
}

/// @brief Calculates the Frobenius norm of the matrix.
template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order, eMatrixLayout Layout>
T getNorm(const Matrix <T, Rows, Columns, Order, Layout> &m) {
    return std::sqrt(getNormSquared(m));
}

template<class Vt, class Mt, uint32_t Vd, uint32_t Mrow>
auto operator*(const Matrix <Mt, Mrow, Vd, eMatrixOrder::PRECEDE_VECTOR, eMatrixLayout::ROW_MAJOR> &mat,
               const ShortArray <Vt, Vd> &vec) {
    using Rt = traits::MatMulElemT<Vt, Mt>;
    ShortArray <Rt, Mrow> result;

    for (uint32_t i = 0; i < Mrow; ++i) {
        result(i) = dot(vec, mat.stripes[i]);
    }
    return result;
}

}  // namespace rally

#endif  // RALLY_CORE_CONTAINERS_DETAILS_MATRIX_ARITHMETIC_H
