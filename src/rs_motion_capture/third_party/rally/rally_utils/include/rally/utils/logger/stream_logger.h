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
#ifndef RALLY_UTILS_LOGGER_STREAM_LOGGER_H
#define RALLY_UTILS_LOGGER_STREAM_LOGGER_H

#include <sstream>
#include <mutex>
#include "rally/utils/logger/common.h"

namespace rally {
namespace log {

class StreamLogger {
public:
    StreamLogger();
    ~StreamLogger();

    std::ostringstream &trace();
    std::ostringstream &debug();
    std::ostringstream &info();
    std::ostringstream &warning();
    std::ostringstream &error();

private:
    std::ostringstream &getStream(Level level);
    void flush();

    Level m_activeLevel = Level::INFO;
    static std::mutex m_mutex;
    std::ostringstream m_oss;
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_STREAM_LOGGER_H
