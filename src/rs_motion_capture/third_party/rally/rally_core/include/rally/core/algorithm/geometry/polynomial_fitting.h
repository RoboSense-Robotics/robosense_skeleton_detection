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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_POLYNOMIAL_FITTING_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_POLYNOMIAL_FITTING_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include "rally/core/containers/short_array.h"

namespace rally {

template<typename T>
inline std::vector<T> polyfit(const std::vector<rally::ShortArray<T, 2> > &points, uint32_t degree,
                              const std::vector<T> &weights = std::vector<T>(), bool useJacobi = true) {
    RALLY_STATIC_ASSERT(std::is_floating_point<T>::value);
    bool useWeights = weights.size() > 0 && weights.size() == points.size();

    uint32_t numCoefficients = degree + 1;
    size_t nCount = points.size();

    Eigen::MatrixXd x_values(nCount, numCoefficients);
    Eigen::MatrixXd y_values(nCount, 1);

    // fill Y matrix
    for (size_t i = 0; i < nCount; i++) {
        if (useWeights) {
            y_values(i, 0) = points[i].y * weights[i];
        } else {
            y_values(i, 0) = points[i].y;
        }
    }

    // fill X matrix (Vandermonde matrix)
    for (size_t nRow = 0; nRow < nCount; nRow++) {
        T nVal = 1.0f;
        for (uint32_t nCol = 0; nCol < numCoefficients; nCol++) {
            if (useWeights) {
                x_values(nRow, nCol) = nVal * weights[nRow];
            } else {
                x_values(nRow, nCol) = nVal;
            }

            nVal *= points[nRow].x;
        }
    }

    Eigen::VectorXd coefficients;
    if (useJacobi) {
        coefficients = x_values.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y_values);
    } else {
        coefficients = x_values.colPivHouseholderQr().solve(y_values);
    }

    return std::vector<T>(coefficients.data(), coefficients.data() + numCoefficients);
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_POLYNOMIAL_FITTING_H
