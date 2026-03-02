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
#ifndef RALLY_CORE_CONTAINERS_MATRIX_H
#define RALLY_CORE_CONTAINERS_MATRIX_H

#include <cstdint>
#include "rally/core/containers/details/matrix_data.h"

namespace rally {

template <class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order = eMatrixOrder::PRECEDE_VECTOR,
eMatrixLayout Layout = eMatrixLayout::ROW_MAJOR>
class Matrix : public MatrixData<T, Rows, Columns, Order, Layout> {
    RALLY_STATIC_ASSERT(Columns >= 1 && Rows >= 1);
    static constexpr uint32_t VecDim = std::max(Rows, Columns);
    static constexpr bool VectorAssignable = std::min(Rows, Columns) == 1;
protected:
    using MatrixData<T, Rows, Columns, Order, Layout>::getElement;

public:
    using MatrixData<T, Rows, Columns, Order, Layout>::getRowCount;
    using MatrixData<T, Rows, Columns, Order, Layout>::getColumnCount;

    Matrix() = default;

    template <class T2, eMatrixLayout Layout2>
    Matrix(const Matrix<T2, Rows, Columns, Order, Layout2>& rhs) {
        for (uint32_t i = 0; i < getRowCount(); ++i) {
            for (uint32_t j = 0; j < getColumnCount(); ++j) {
                (*this)(i, j) = rhs(i, j);
            }
        }
    }

    template <class H, class... Args,
    typename std::enable_if<traits::All<traits::isScalar, H, Args...>::value, uint32_t>::type = 0,
    typename std::enable_if<1 + sizeof...(Args) == Rows * Columns, uint32_t>::type = 0>
    Matrix(H h, Args... args) {
        assign<0, 0>(h, args...);
    }

    /// @brief From vector if applicable (for 1*N and N*1 matrices)
    template <class T2, class = typename std::enable_if<VectorAssignable, T2>::type>
    Matrix(const ShortArray<T2, VecDim>& v) {
        for (uint32_t i = 0; i < v.getDimension(); ++i) {
            (*this)(i) = v(i);
        }
    }

    /// @brief General matrix indexing
    T& operator()(uint32_t row, uint32_t col) {
        return getElement(row, col);
    }
    T operator()(uint32_t row, uint32_t col) const {
        return getElement(row, col);
    }

    ///@brief Column and row vector simple indexing
    template <class Q = T>
    inline typename std::enable_if<(Columns == 1 && Rows > 1) || (Columns > 1 && Rows == 1), Q>::type& operator()(uint32_t idx) {
        return getElement(Rows == 1 ? 0 : idx, Columns == 1 ? 0 : idx);
    }
    template <class Q = T>
    inline typename std::enable_if<(Columns == 1 && Rows > 1) || (Columns > 1 && Rows == 1), Q>::type operator()(uint32_t idx) const {
        return getElement(Rows == 1 ? 0 : idx, Columns == 1 ? 0 : idx);
    }

    Matrix<T, Columns, Rows, Order, Layout> transpose() {
        Matrix<T, Columns, Rows, Order, Layout> result;
        for (uint32_t i = 0; i < Rows; ++i) {
            for (uint32_t j = 0; j < Columns; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    T sum() {
        T sum = static_cast<T>(0);
        for (uint32_t row = 0; row < getRowCount(); ++row) {
            for (uint32_t col = 0; col < getColumnCount(); ++col) {
                sum += (*this)(row, col);
            }
        }
        return sum;
    }

    T trace() {
        RALLY_STATIC_ASSERT(Rows == Columns);
        T sum = static_cast<T>(0);
        for (uint32_t i = 0; i < Rows; ++i) {
            sum += (*this)(i, i);
        }
        return sum;
    }

    static Matrix<T, Columns, Rows, Order, Layout> zero() {
        Matrix<T, Columns, Rows, Order, Layout> res;
        for (auto& stripe : res.stripes) {
            stripe.fill(static_cast<T>(0));
        }
        return res;
    };

    static Matrix<T, Columns, Rows, Order, Layout> identity() {
        Matrix<T, Columns, Rows, Order, Layout> res;
        res = Matrix<T, Columns, Rows, Order, Layout>::zero();
        for (uint32_t i = 0; i < std::min(res.getColumnCount(), res.getRowCount()); ++i) {
            res(i, i) = static_cast<T>(1);
        }
        return res;
    }

    std::string const name() const noexcept {
        return std::to_string(getRowCount()) + "-Row," + std::to_string(getRowCount()) + "Col Matrix";
    }

    std::string infos() const {
        std::stringstream ss;
        ss << name() << "(" << std::endl;
        for (uint32_t i = 0; i < getRowCount(); ++i) {
            for (uint32_t j = 0; j < getColumnCount(); ++j) {
                ss << (*this)(i, j) << ",";
            }
            if (i != getRowCount() - 1) {
                ss << std::endl;
            }
        }
        ss << ")";
        return ss.str();
    }

private:
    template <uint32_t i, uint32_t j, class Head, class... Args>
    void assign(Head head, Args... args) {
        (*this)(i, j) = static_cast<T>(head);
        assign<((j != Columns - 1) ? i : (i + 1)), ((j + 1) % Columns)>(args...);
    }

    template <uint32_t, uint32_t>
    void assign() {}
};

using Matrix2f = Matrix<float, 2, 2>;
using Matrix3f = Matrix<float, 3, 3>;
using Matrix4f = Matrix<float, 4, 4>;

}  // namespace rally

#include "rally/core/containers/details/matrix_arithmetic.h"

#endif  // RALLY_CORE_CONTAINERS_MATRIX_H
