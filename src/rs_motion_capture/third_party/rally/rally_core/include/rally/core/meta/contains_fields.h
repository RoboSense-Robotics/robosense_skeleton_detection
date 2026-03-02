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
#ifndef RALLY_CORE_META_CONTAINS_FIELDS_H
#define RALLY_CORE_META_CONTAINS_FIELDS_H

#include <type_traits>

namespace rally {
namespace details {

///@brief define x field check
template<typename T, typename V = bool>
struct has_x : std::false_type {
};

template<typename T>
struct has_x<T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().x),
void>::value, bool>::type>
: std::true_type {
};

///@brief define y field check
template<typename T, typename V = bool>
struct has_y : std::false_type {
};

template<typename T>
struct has_y<T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().y),
void>::value, bool>::type>
: std::true_type {
};

///@brief define z field check
template<typename T, typename V = bool>
struct has_z : std::false_type {
};

template<typename T>
struct has_z<T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().z),
void>::value, bool>::type>
: std::true_type {
};

}  // namespace details
}  // namespace rally

#endif  // RALLY_CORE_META_CONTAINS_FIELDS_H
