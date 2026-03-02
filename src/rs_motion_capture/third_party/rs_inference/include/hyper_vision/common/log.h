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
#ifndef RS_COMMON_EXTERNAL_RS_LOGGER_H
#define RS_COMMON_EXTERNAL_RS_LOGGER_H

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
//#ifndef CMAKE_DEV
//#include "cyber/common/log.h"
//#endif
namespace robosense {
namespace inference {

class RSLogger {
 public:
  enum class Level {
    NONE = 0,
    Trace,
    Debug,
    DebugSP,
    Info,
    Warning,
    Error,
    Fatal
  };

  enum class TimestampMode {
    None = 0,
    DateTime,
    EpochSeconds,
    EpochMilliseconds,
    EpochMicroseconds
  };

  //! Constructor.
  RSLogger();

  //! Destructor.
  ~RSLogger();

  /*! Initialize the logger.
   *  \param filename Log to filename. Disabled if empty.
   *  \param append The existing log will be appended if true.
   *  Throws on error. */
  static void init(const bool log = false, const uint32_t max_log_size = 2,
                   bool append = false);

  //! Enable/disable echo mode.
  //! \param enable Echo everything if true. Default is false.
  static void enableEchoMode(bool enable);

  //! Set the logging level.
  //! \param level The minimum level. Default is Info.
  static void setLoggingLevel(Level level);

  //! Set custom symbol for the given logging level.
  //! \param level The level.
  //! \param symbol The symbol outputted for the messages of this level.
  static void setLevelSymbol(Level level, std::string symbol);

  //! Set/enable timestamp mode.
  //! \param timestampMode Timestamp mode enumeration.
  //! \param separator Separator string outputted after timestamp.
  static void setTimestampMode(TimestampMode timestampMode);

  //! Get stream to the trace log message.
  std::ostringstream &trace();

  //! Get stream to the debug log message.
  std::ostringstream &debug();

  //! Get stream to the debugsp log message.
  std::ostringstream &debugsp();

  //! Get stream to the info log message.
  std::ostringstream &info();

  //! Get stream to the warning log message.
  std::ostringstream &warning();

  //! Get stream to the error log message.
  std::ostringstream &error();

  //! Get stream to the fatal log message.
  std::ostringstream &fatal();

 private:
  std::ostringstream &getStream(Level level);

  void flush();

  void prefixTimestamp();

  static bool m_echoMode;

  static Level m_level;

  static TimestampMode m_timestampMode;

  static char m_timestampSeparator[];

  static std::ofstream m_fout;

  static uint32_t m_file_length;

  static uint32_t m_max_log_size;

  using SymbolMap = std::map<Level, std::string>;
  static SymbolMap m_symbols;

  static std::mutex m_mutex;

  Level m_activeLevel = Level::Info;

  std::ostringstream m_oss;
};

const static std::map<RSLogger::Level, std::string> kLogLevel2NameMap = {
    {RSLogger::Level::Trace, "trace"},     {RSLogger::Level::Debug, "debug"},
    {RSLogger::Level::DebugSP, "debugsp"}, {RSLogger::Level::Info, "info"},
    {RSLogger::Level::Warning, "warning"}, {RSLogger::Level::Error, "error"},
    {RSLogger::Level::Fatal, "fatal"},
};

const static std::map<std::string, RSLogger::Level> kLogLevelName2TypeMap = {
    {"trace", RSLogger::Level::Trace},     {"debug", RSLogger::Level::Debug},
    {"debugsp", RSLogger::Level::DebugSP}, {"info", RSLogger::Level::Info},
    {"warning", RSLogger::Level::Warning}, {"error", RSLogger::Level::Error},
    {"fatal", RSLogger::Level::Fatal},
};

#ifdef __GNUC__
#define FILENAME(x) (strrchr(x, '/') ? strrchr(x, '/') + 1 : x)
#elif _MSC_VER
#define FILENMAE(x) (strrchr(x, '\\') ? strrchr(x, '\\') + 1 : x)
#endif

//#ifndef CMAKE_DEV
//#define INFER_TRACE ADEBUG
//#define INFER_DEBUG AINFO
//#define INFER_DEBUGSP ADEBUG
//#define INFER_INFO AINFO
//#define INFER_WARN AWARN
//#define INFER_ERROR AERROR
//#else
#define INFER_TRACE (RSLogger().trace())
#define INFER_DEBUG (RSLogger().debug())
#define INFER_DEBUGSP (RSLogger().debugsp())
#define INFER_INFO (RSLogger().info())
#define INFER_WARN (RSLogger().warning())
#define INFER_ERROR (RSLogger().error())
#define INFER_FATAL (RSLogger().fatal())
//#endif
}  //  namespace inference
}  //  namespace robosense
#endif  // RS_COMMON_EXTERNAL_RS_LOGGER_H
