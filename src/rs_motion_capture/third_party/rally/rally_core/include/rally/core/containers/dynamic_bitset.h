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
#ifndef RALLY_CORE_CONTAINERS_DYNAMIC_BITSET_H
#define RALLY_CORE_CONTAINERS_DYNAMIC_BITSET_H

#include <limits.h>
#include <vector>
#include <algorithm>
#include <vector>

namespace rally {

class DynamicBitset {
public:
    DynamicBitset() noexcept : size_(0) {}

    /// @brief only constructor we use in our code
    /// @param the size of the bitset (in bits)
    DynamicBitset(size_t size) noexcept : size_(size) {
        resize(size);
        reset();
    }

    /// @brief checks if the bitset is empty
    /// @return true if the bitset is empty
    bool empty() const noexcept {
        return bitset_.empty();
    }

    /// @param set all the bits to 0
    void reset() noexcept {
        std::fill(bitset_.begin(), bitset_.end(), 0);
    }

    /// @brief set one bit to 0
    /// @param
    void reset(size_t index) noexcept {
        bitset_[index / cell_bit_size_] &= ~(size_t(1) << (index % cell_bit_size_));
    }

    /// @brief sets a specific bit to 0, and more bits too
    ///        This function is useful when resetting a given set of bits so that the
    ///        whole bitset ends up being 0: if that's the case, we don't care about setting
    ///        other bits to 0
    /// @param
    void reset_block(size_t index) noexcept {
        bitset_[index / cell_bit_size_] = 0;
    }

    /// @brief resize the bitset so that it contains at least size bits
    /// @param size
    void resize(size_t in_size) noexcept;

    /// @brief set a bit to true
    /// @param index the index of the bit to set to 1
    void set(size_t index) noexcept {
        bitset_[index / cell_bit_size_] |= size_t(1) << (index % cell_bit_size_);
    }

    /// @brief gives the number of contained bits
    size_t size() const noexcept {
        return size_;
    }

    /// @brief check if a bit is set
    /// @param index the index of the bit to check
    /// @return true if the bit is set
    bool test(size_t index) const noexcept {
        return (bitset_[index / cell_bit_size_] & (size_t(1) << (index % cell_bit_size_))) != 0;
    }

private:
    std::vector<size_t> bitset_;
    size_t size_;
    static const unsigned int cell_bit_size_ = CHAR_BIT * sizeof(size_t);
};

}  // namespace rally

#endif  // RALLY_CORE_CONTAINERS_DYNAMIC_BITSET_H
