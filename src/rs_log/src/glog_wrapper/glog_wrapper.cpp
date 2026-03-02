#include "rs_log/glog_wrapper/glog_wrapper.h"
#include <cstdarg>
#include <cstdio>
#include <glog/logging.h>
#include <memory>
#include <vector>

void log_message(int level, const char *format, va_list args) {
  // We might need a larger buffer, but this is a start
  std::vector<char> buffer(1024);
  // vsnprintf will return the number of characters needed
  int needed = vsnprintf(buffer.data(), buffer.size(), format, args);
  if (needed >= 0 && needed < buffer.size()) {
    // We have enough space
    switch (level) {
    case 0:
      LOG(INFO) << buffer.data();
      break;
    case 1:
      LOG(WARNING) << buffer.data();
      break;
    case 2:
      LOG(ERROR) << buffer.data();
      break;
    case 3:
      LOG(FATAL) << buffer.data();
      break;
    }
  } else {
    // We need more space, allocate it and print again
    buffer.resize(needed + 1);
    needed = vsnprintf(buffer.data(), buffer.size(), format, args);
    switch (level) {
    case 0:
      LOG(INFO) << buffer.data();
      break;
    case 1:
      LOG(WARNING) << buffer.data();
      break;
    case 2:
      LOG(ERROR) << buffer.data();
      break;
    case 3:
      LOG(FATAL) << buffer.data();
      break;
    }
  }
}

void log_info(const char *format, ...) {
  va_list args;
  va_start(args, format);
  log_message(0, format, args);
  va_end(args);
}

void log_warn(const char *format, ...) {
  va_list args;
  va_start(args, format);
  log_message(1, format, args);
  va_end(args);
}

void log_error(const char *format, ...) {
  va_list args;
  va_start(args, format);
  log_message(2, format, args);
  va_end(args);
}

void log_fatal(const char *format, ...) {
  va_list args;
  va_start(args, format);
  log_message(3, format, args);
  va_end(args);
}
