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

#include "rally/utils/logger/stream_logger.h"
#include "rally/utils/logger/detail/registry.h"

namespace rally {
namespace log {

std::mutex StreamLogger::m_mutex;

StreamLogger::StreamLogger() {
    m_mutex.lock();
}

std::ostringstream &StreamLogger::trace() {
    return getStream(Level::TRACE);
}
std::ostringstream &StreamLogger::debug() {
    return getStream(Level::DEBUG);
}
std::ostringstream &StreamLogger::info() {
    return getStream(Level::INFO);
}
std::ostringstream &StreamLogger::warning() {
    return getStream(Level::WARN);
}
std::ostringstream &StreamLogger::error() {
    return getStream(Level::ERROR);
}

std::ostringstream &StreamLogger::getStream(Level level) {
    m_activeLevel = level;
    return m_oss;
}

void StreamLogger::flush() {
    m_oss << std::endl;
    Registry::getInstance().getDefaultRaw()->log(m_activeLevel, m_oss.str());
}

StreamLogger::~StreamLogger() {
    flush();
    m_mutex.unlock();
}

}  // namespace log
}  // namespace rally
