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
#ifndef RALLY_UTILS_DETAILS_MACROS_H
#define RALLY_UTILS_DETAILS_MACROS_H

namespace rally {

#define RALLY_DISALLOW_COPY_AND_ASSIGN(classname)                                \
  classname(const classname &) = delete;                                         \
  classname &operator=(const classname &) = delete;

#define RALLY_CACHELINE_SIZE 64

#define RALLY_DEPRECATED(message) __attribute__ ((deprecated(message)))

#define RALLY_STATIC_ASSERT(...) static_assert(__VA_ARGS__, #__VA_ARGS__)

/// @brief Meyers Singleton
template<typename T>
class Singleton {
public:
    static T &getInstance() {
        static T value;
        return value;
    }

private:
    Singleton() {}

    RALLY_DISALLOW_COPY_AND_ASSIGN(Singleton)

    ~Singleton() {}
};

}  // namespace rally

#endif  // RALLY_UTILS_DETAILS_MACROS_H
