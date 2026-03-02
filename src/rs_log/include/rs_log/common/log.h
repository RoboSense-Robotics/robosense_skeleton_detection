/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @log
 */

#ifndef RSLOG_COMMON_LOG_H_
#define RSLOG_COMMON_LOG_H_

#include <cstdarg>
#include <string>

#include "glog/logging.h"
#include "glog/raw_logging.h"

// #include "rs_log/binary.h"

// RS macro define module string name
const char* const RS_COMMON = "RS_COMMON";
const char* const RS_COMMON_MSGS = "RS_COMMON_MSGS";
const char* const RS_CONTROL = "RS_CONTROL";
const char* const RS_LOCALIZATION = "RS_LOCALIZATION";
const char* const RS_MAP = "RS_MAP";
const char* const RS_PERCEPTION = "RS_PERCEPTION";
const char* const RS_PLANNING = "RS_PLANNING";
const char* const RS_PREDICTION = "RS_PREDICTION";
const char* const RS_ROUTING = "RS_ROUTING";
const char* const RS_TOOLS = "RS_TOOLS";
const char* const RS_CAMERA = "RS_CAMERA";

#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"

#ifndef MODULE_NAME
#define MODULE_NAME "UNKOWN"
#endif

// RS macro define BEGIN
#define RSINFO(sub_module) RSLOG_MODULE(sub_module, MODULE_NAME, INFO)
#define RSWARN(sub_module) RSLOG_MODULE(sub_module, MODULE_NAME, WARN)
#define RSERROR(sub_module) RSLOG_MODULE(sub_module, MODULE_NAME, ERROR)
#define RSFATAL(sub_module) RSLOG_MODULE(sub_module, MODULE_NAME, FATAL)

#define RSINFO_EVERY(freq, sub_module)                                    \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET \
                          << LEFT_BRACKET << sub_module << RIGHT_BRACKET
#define RSWARN_EVERY(freq, sub_module)                                       \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET \
                             << LEFT_BRACKET << sub_module << RIGHT_BRACKET
#define RSERROR_EVERY(freq, sub_module)                                    \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET \
                           << LEFT_BRACKET << sub_module << RIGHT_BRACKET
#define RSFATAL_EVERY(freq, sub_module)                                    \
  LOG_EVERY_N(FATAL, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET \
                           << LEFT_BRACKET << sub_module << RIGHT_BRACKET

const char* const BLACK_LIST_LOG = "-BLACK_LIST_LOG";
#define RSERROR_BLL(sub_module)                                        \
  RSLOG_MODULE(                                                        \
      (std::string(sub_module) + std::string(BLACK_LIST_LOG)).c_str(), \
      MODULE_NAME, ERROR)
#define RSWARN_BLL(sub_module)                                         \
  RSLOG_MODULE(                                                        \
      (std::string(sub_module) + std::string(BLACK_LIST_LOG)).c_str(), \
      MODULE_NAME, WARN)
#define RSWARN_BLL_EVERY(freq, sub_module) \
  RSWARN_EVERY(                             \
      freq, (std::string(sub_module) + std::string(BLACK_LIST_LOG)).c_str())
#define RSERROR_BLL_EVERY(freq, sub_module) \
  RSERROR_EVERY(                            \
      freq, (std::string(sub_module) + std::string(BLACK_LIST_LOG)).c_str())

#ifndef RSLOG_MODULE_STREAM
#define RSLOG_MODULE_STREAM(log_severity) RSLOG_MODULE_STREAM_##log_severity
#endif

#ifndef RSLOG_MODULE
#define RSLOG_MODULE(sub_module, module, log_severity) \
  RSLOG_MODULE_STREAM(log_severity)(sub_module, module)
#endif

#define RSLOG_MODULE_STREAM_INFO(sub_module, module)                           \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream()                \
      << LEFT_BRACKET << module << RIGHT_BRACKET << LEFT_BRACKET << sub_module \
      << RIGHT_BRACKET

#define RSLOG_MODULE_STREAM_WARN(sub_module, module)                           \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream()             \
      << LEFT_BRACKET << module << RIGHT_BRACKET << LEFT_BRACKET << sub_module \
      << RIGHT_BRACKET

#define RSLOG_MODULE_STREAM_ERROR(sub_module, module)                          \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream()               \
      << LEFT_BRACKET << module << RIGHT_BRACKET << LEFT_BRACKET << sub_module \
      << RIGHT_BRACKET

#define RSLOG_MODULE_STREAM_FATAL(sub_module, module)                          \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream()               \
      << LEFT_BRACKET << module << RIGHT_BRACKET << LEFT_BRACKET << sub_module \
      << RIGHT_BRACKET
// RS macro define END

#define ADEBUG_MODULE(module) \
  VLOG(4) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG] "
#define ADEBUG ADEBUG_MODULE(MODULE_NAME)
#define AINFO RSLOG_MODULE("UNKNOWN", MODULE_NAME, INFO)
#define AWARN RSLOG_MODULE("UNKNOWN", MODULE_NAME, WARN)
#define AERROR RSLOG_MODULE("UNKNOWN", MODULE_NAME, ERROR)
#define AFATAL RSLOG_MODULE("UNKNOWN", MODULE_NAME, FATAL)

#ifndef ALOG_MODULE_STREAM
#define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#define ALOG_MODULE(module, log_severity) \
  ALOG_MODULE_STREAM(log_severity)(module)
#endif

#define ALOG_MODULE_STREAM_INFO(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_WARN(module)                            \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_ERROR(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_FATAL(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define AINFO_IF(cond) ALOG_IF(INFO, cond, MODULE_NAME)
#define AWARN_IF(cond) ALOG_IF(WARN, cond, MODULE_NAME)
#define AERROR_IF(cond) ALOG_IF(ERROR, cond, MODULE_NAME)
#define AFATAL_IF(cond) ALOG_IF(FATAL, cond, MODULE_NAME)
#define ALOG_IF(severity, cond, module) \
  !(cond) ? (void)0                     \
          : google::LogMessageVoidify() & ALOG_MODULE(module, severity)

#define ACHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define AINFO_EVERY(freq) \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AWARN_EVERY(freq) \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AERROR_EVERY(freq) \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return;                          \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)           \
  if (condition) {                     \
    AWARN << #condition << " is met."; \
    return;                            \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)  \
  if (condition) {                     \
    AWARN << #condition << " is met."; \
    return val;                        \
  }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val) \
  if (ptr == nullptr) {               \
    return (val);                     \
  }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val) \
  if (condition) {                     \
    return (val);                      \
  }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition) \
  if (condition) {            \
    return;                   \
  }
#endif

#endif  // RSLOG_COMMON_LOG_H_
