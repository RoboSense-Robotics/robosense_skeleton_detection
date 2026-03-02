//
// Created by sti on 2025/6/16.
//

#ifndef MOTION_CAPTURE_LOGGER_H
#define MOTION_CAPTURE_LOGGER_H

#include "rally/utils/logger/sinks/file_sink.h"
#include "rally/utils/utils.h"

namespace robosense {
namespace motion_capture {

inline void setLogger(const std::string &log_to_dir, const std::string &log_level, const std::string &log_mode) {
  std::vector<rally::log::BaseSink::Ptr> sinks;
  rally::log::BaseSink::Ptr terminator_ptr(new rally::log::TerminalSink(stdout));
  sinks.push_back(terminator_ptr);

  std::string log_file = "/tmp/rs_motion_capture.txt";
  if (rally::directoryExists(log_to_dir)) {
    log_file = log_to_dir + "/rs_motion_capture.txt";
  } else {
    RWARN << "log_dir: " << log_to_dir << " not exist! write log to default dir: /tmp";
  }
  rally::log::FileSink::Ptr file_sink_ptr(new rally::log::FileSink(log_file));
  sinks.push_back(file_sink_ptr);
  const auto &dump_catcher_logger = rally::log::getLoggerByName("dump_catcher");
  dump_catcher_logger->getSinks().push_back(file_sink_ptr);

  rally::log::Logger::Ptr logger;
  if ("sync" == log_mode) {
    logger = rally::log::createSyncLogger("sync", sinks);
  } else {
    logger = rally::log::createAsyncLogger("async", sinks);

    std::vector<std::thread *> tps;
    rally::log::Registry::getInstance().getTp()->get_threads(tps);
  }

  if (rally::log::kLevelName2Type.find(log_level) != rally::log::kLevelName2Type.end()) {
    logger->setLevel(rally::log::kLevelName2Type.at(log_level));
  } else {
    logger->setLevel(rally::log::Level::INFO);
  }

  rally::log::setDefaultLogger(logger);
}

}
}

#endif //MOTION_CAPTURE_LOGGER_H
