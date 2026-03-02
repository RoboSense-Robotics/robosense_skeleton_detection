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
#include "hyper_vision/common/log.h"

namespace robosense {
namespace inference {

#define TRACE(X)                                                   \
  std::cout << std::setiosflags(std::ios::fixed) << X << "\033[0m" \
            << "\r" << std::flush << std::endl

#define DEBUG(X)                                                      \
  std::cout << std::setiosflags(std::ios::fixed) << "\033[1;35m" << X \
            << "\033[0m"                                              \
            << "\r" << std::flush << std::endl

#define DEBUGSP(X)                                                    \
  std::cout << std::setiosflags(std::ios::fixed) << "\033[1;34m" << X \
            << "\033[0m"                                              \
            << "\r" << std::flush << std::endl

#define INFO(X)                                                       \
  std::cout << std::setiosflags(std::ios::fixed) << "\033[1;32m" << X \
            << "\033[0m"                                              \
            << "\r" << std::flush << std::endl

#define WARNING(X)                                                    \
  std::cout << std::setiosflags(std::ios::fixed) << "\033[1;33m" << X \
            << "\033[0m"                                              \
            << "\r" << std::flush << std::endl

#define ERROR(X)                                                      \
  std::cout << std::setiosflags(std::ios::fixed) << "\033[1;31m" << X \
            << "\033[0m"                                              \
            << "\r" << std::flush << std::endl

#define FATAL(X)                                                      \
  std::cout << std::setiosflags(std::ios::fixed) << "\033[1;31m" << X \
            << "\033[0m"                                              \
            << "\r" << std::flush << std::endl

bool RSLogger::m_echoMode = true;

RSLogger::Level RSLogger::m_level = RSLogger::Level::Info;

RSLogger::TimestampMode RSLogger::m_timestampMode =
    RSLogger::TimestampMode::None;

char RSLogger::m_timestampSeparator[] = ": ";

std::ofstream RSLogger::m_fout;

uint32_t RSLogger::m_file_length;

uint32_t RSLogger::m_max_log_size;

// Default level symbols
RSLogger::SymbolMap RSLogger::m_symbols = {
    {RSLogger::Level::Trace, "Trace:"},
    {RSLogger::Level::Debug, "Ddebug:"},
    {RSLogger::Level::DebugSP, "DdebugSP:"},
    {RSLogger::Level::Info, "Info:"},
    {RSLogger::Level::Warning, "Warning:"},
    {RSLogger::Level::Error, "Error:"},
    {RSLogger::Level::Fatal, "Fatal:"}};

std::mutex RSLogger::m_mutex;

RSLogger::RSLogger() { m_mutex.lock(); }

void RSLogger::init(const bool log, const uint32_t max_log_size, bool append) {
  if (log) {
    std::string filename = "/tmp/rs_sdk.log";
    m_fout.open(filename, append ? std::ofstream::out | std::ofstream::app
                                 : std::ofstream::out);
    if (!m_fout.is_open()) {
      throw std::runtime_error("ERROR!!: Couldn't open '" + filename +
                               "' for write.\n");
    }
    m_max_log_size = max_log_size;
    m_file_length = 0;
  }
}

void RSLogger::prefixTimestamp() {
  std::string timeStr;

  using std::chrono::duration_cast;
  using std::chrono::system_clock;

  switch (m_timestampMode) {
    case RSLogger::TimestampMode::None:
      break;
    case RSLogger::TimestampMode::DateTime: {
      time_t rawTime;
      static_cast<void>(time(&rawTime));
      char buf[26];
      timeStr = ctime_r(&rawTime, buf);
      timeStr.erase(timeStr.length() - 1);
    } break;
    case RSLogger::TimestampMode::EpochSeconds:
      timeStr = std::to_string(duration_cast<std::chrono::seconds>(
                                   system_clock::now().time_since_epoch())
                                   .count());
      break;
    case RSLogger::TimestampMode::EpochMilliseconds:
      timeStr = std::to_string(duration_cast<std::chrono::milliseconds>(
                                   system_clock::now().time_since_epoch())
                                   .count());
      break;
    case RSLogger::TimestampMode::EpochMicroseconds:
      timeStr = std::to_string(duration_cast<std::chrono::microseconds>(
                                   system_clock::now().time_since_epoch())
                                   .count());
      break;
    default:
      break;
  }

  if (!timeStr.empty()) {
    m_oss << timeStr << m_timestampSeparator;
  }
}

void RSLogger::enableEchoMode(bool enable) { m_echoMode = enable; }

void RSLogger::setLoggingLevel(Level level) { m_level = level; }

void RSLogger::setLevelSymbol(Level level, std::string symbol) {
  m_symbols[level] = symbol;
}

void RSLogger::setTimestampMode(TimestampMode timestampMode) {
  m_timestampMode = timestampMode;
}

void RSLogger::flush() {
  if (m_activeLevel < m_level) {
    return;
  }

  if (!m_oss.str().size()) {
    return;
  }

  if ((m_file_length >> 20) >= m_max_log_size && m_fout.is_open()) {
    m_fout.close();
  }
  if (m_fout.is_open()) {
    m_fout << m_oss.str() << std::endl;
    m_file_length += static_cast<uint32_t>(m_oss.str().size());
    m_fout.flush();
  }

  if (m_echoMode) {
    switch (m_activeLevel) {
      case RSLogger::Level::Trace:
        TRACE(m_oss.str());
        break;
      case RSLogger::Level::Debug:
        DEBUG(m_oss.str());
        break;
      case RSLogger::Level::DebugSP:
        DEBUGSP(m_oss.str());
        break;
      case RSLogger::Level::Info:
        INFO(m_oss.str());
        break;
      case RSLogger::Level::Warning:
        WARNING(m_oss.str());
        break;
      case RSLogger::Level::Error:
        ERROR(m_oss.str());
        break;
      case RSLogger::Level::Fatal:
        FATAL(m_oss.str());
        break;
      default:
        break;
    }
  }
}

std::ostringstream &RSLogger::getStream(RSLogger::Level level) {
  m_activeLevel = level;
  prefixTimestamp();
  return m_oss;
}

std::ostringstream &RSLogger::trace() {
  return getStream(RSLogger::Level::Trace);
}

std::ostringstream &RSLogger::debug() {
  return getStream(RSLogger::Level::Debug);
}

std::ostringstream &RSLogger::debugsp() {
  return getStream(RSLogger::Level::DebugSP);
}

std::ostringstream &RSLogger::info() {
  return getStream(RSLogger::Level::Info);
}

std::ostringstream &RSLogger::warning() {
  return getStream(RSLogger::Level::Warning);
}

std::ostringstream &RSLogger::error() {
  return getStream(RSLogger::Level::Error);
}

std::ostringstream &RSLogger::fatal() {
  return getStream(RSLogger::Level::Fatal);
}

RSLogger::~RSLogger() {
  flush();

  m_mutex.unlock();
}

}  //  namespace inference
}  //  namespace robosense
