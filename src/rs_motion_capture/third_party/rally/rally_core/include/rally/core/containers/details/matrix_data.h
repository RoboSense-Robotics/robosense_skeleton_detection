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
#ifndef RALLY_CORE_CONTAINERS_DETAILS_MATRIX_DATA_H
#define RALLY_CORE_CONTAINERS_DETAILS_MATRIX_DATA_H

#include <array>
#include "rally/core/containers/short_array.h"
#include "rally/core/containers/details/traits.h"

namespace rally {

template<class T, uint32_t Rows, uint32_t Columns, eMatrixOrder Order = eMatrixOrder::FOLLOW_VECTOR,
eMatrixLayout Layout = eMatrixLayout::ROW_MAJOR>
class MatrixData {
public:
    /// @brief Returns the number of columns of the matrix. </summary>
    constexpr uint32_t getColumnCount() const {
        return Columns;
    }

    /// @brief Returns the number of rows of the matrix.
    constexpr uint32_t getRowCount() const {
        return Rows;
    }

    /// @brief Returns the number of columns of the matrix.
    constexpr uint32_t getWidth() const {
        return Columns;
    }

    /// @brief Returns the number of rows of the matrix.
    constexpr uint32_t getHeight() const {
        return Rows;
    }

    // Rows equal height, Columns equal width, row-major has column-sized stripes
    static constexpr uint32_t StripeDim = Layout == eMatrixLayout::ROW_MAJOR ? Columns : Rows;
    static constexpr uint32_t StripeCount = Layout == eMatrixLayout::ROW_MAJOR ? Rows : Columns;

    using StripeVecT = ShortArray<T, StripeDim>;
    std::array<StripeVecT, StripeCount> stripes;

protected:
    // Get element
    inline T &getElement(uint32_t row, uint32_t col) {
        RENSURE(row < getRowCount());
        RENSURE(col < getColumnCount());
        if (Layout == eMatrixLayout::ROW_MAJOR) {
            return stripes[row][col];
        } else {
            return stripes[col][row];
        }
    }

    inline T getElement(uint32_t row, uint32_t col) const {
        RENSURE(row < getRowCount());
        RENSURE(col < getColumnCount());
        if (Layout == eMatrixLayout::ROW_MAJOR) {
            return stripes[row][col];
        } else {
            return stripes[col][row];
        }
    }
};


}  // namespace rally

#endif  // RALLY_CORE_CONTAINERS_DETAILS_MATRIX_DATA_H
