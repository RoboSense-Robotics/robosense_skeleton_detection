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

#include <libgen.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

#include <csignal>
#include <cstdio>
#include <ctime>
#include <memory>
#include <string>
#include <optional>

#include <filesystem>

#include "rs_log/init.h"
#include "rs_log/logger/async_logger.h"

namespace robosense {
namespace log {

namespace {

bool g_atexit_registered = false;
std::mutex g_mutex;

// 之前注册的信号处理函数, 一般为 rclcpp 提供的 signal handler
inline static struct sigaction g_prev_sigint_action {};

logger::AsyncLogger *async_logger = nullptr;

void SignalhHandle(const char *data, int size) {
  AERROR << std::string(data, size);
  if (async_logger) {
    async_logger->Flush();
  }
}

inline std::optional<std::string> GetEnv(char const *name) {
  if (char const *v = std::getenv(name)) {
    return std::string(v);
  }
  return std::nullopt;
}

void InitLogger(const char *binary_name) {
  // setcap会导致无法产生coredump，通过该调用使能
  prctl(PR_SET_DUMPABLE, 1);

  // Init glog
  google::InitGoogleLogging(binary_name);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter(&SignalhHandle);
  google::SetLogDestination(google::ERROR, "");
  google::SetLogDestination(google::WARNING, "");
  google::SetLogDestination(google::FATAL, "");

  // Init async logger
  async_logger = new ::robosense::log::logger::AsyncLogger(
      google::base::GetLogger(FLAGS_minloglevel));
  google::base::SetLogger(FLAGS_minloglevel, async_logger);
  async_logger->Start();
}

void StopLogger() { delete async_logger; }

bool EnsureDirectory(const std::string &path) {
  std::filesystem::path dir(path);
  std::error_code ec;
  // 如果路径不存在，则创建目录（包括所有父目录）
  if (!std::filesystem::exists(dir, ec)) {
    if (!std::filesystem::create_directories(dir, ec)) {
      return false;
    }
  } else if (!std::filesystem::is_directory(dir, ec)) {
    return false;
  }
  return true;
}

} // namespace

void OnShutdown(int sig) {
  (void)sig;

  if (GetState() != STATE_SHUTDOWN) {
    SetState(STATE_SHUTTING_DOWN);
  }

  // 恢复信号处理函数, 再抛出一次调用, 触发之前的信号处理函数
  ::sigaction(sig, &g_prev_sigint_action, nullptr);
  ::kill(getpid(), sig);
}

void ExitHandle() { Clear(); }

bool Init(const char *module_name, const std::string &log_dir,
          const std::string &log_level) {
  // 确保路径存在
  EnsureDirectory(log_dir);

  // 设置日志相关内容
  FLAGS_log_dir = log_dir;
  // 可选：控制日志输出到 stderr（默认 0，即不输出到终端）
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1; // 设为 1 则同时输出到终端和文件
  FLAGS_colorlogtostderr = 1; // 设置log stderr的颜色
  FLAGS_minloglevel = 0;     // 0:INFO, 1:WARNING, 2:ERROR, 3:FATAL
  if (log_level == "INFO") {
    FLAGS_minloglevel = 0; // 0:INFO, 1:WARNING, 2:ERROR, 3:FATAL
  } else if (log_level == "WARN") {
    FLAGS_minloglevel = 1; // 0:INFO, 1:WARNING, 2:ERROR, 3:FATAL
  } else if (log_level == "ERROR") {
    FLAGS_minloglevel = 2; // 0:INFO, 1:WARNING, 2:ERROR, 3:FATAL
  }

  std::lock_guard<std::mutex> lg(g_mutex);
  if (GetState() != STATE_UNINITIALIZED) {
    return false;
  }

  InitLogger(module_name);

  // 保存之前的信号处理函数 + 注册新的信号处理函数
  struct sigaction act{};
  act.sa_handler = OnShutdown;
  sigemptyset(&act.sa_mask);
  act.sa_flags = 0;
  ::sigaction(SIGINT, &act, &g_prev_sigint_action);

  // Register exit handlers
  if (!g_atexit_registered) {
    if (std::atexit(ExitHandle) != 0) {
      //      AERROR << "Register exit handle failed";
      return false;
    }
    //    AINFO << "Register exit handle succ.";
    g_atexit_registered = true;
  }
  SetState(STATE_INITIALIZED);

  return true;
}

bool Init(const char *module_name) {
  auto log_dir = GetEnv("RS_LOG_DIR");
  auto log_level = GetEnv("RS_LOG_LEVEL");

  std::string log_dir_str = "/mnt/ssd/RDCS_LOG_ROOT";
  std::string log_level_str = "INFO";

  if (log_dir.has_value()) {
    log_dir_str = log_dir.value();
  }
  if (log_level.has_value()) {
    log_level_str = log_level.value();
  }

  return Init(module_name, log_dir_str, log_level_str);
}

void Clear() {
  std::lock_guard<std::mutex> lg(g_mutex);
  if (GetState() == STATE_SHUTDOWN || GetState() == STATE_UNINITIALIZED) {
    return;
  }
  StopLogger();
  SetState(STATE_SHUTDOWN);
}

} // namespace log
} // namespace robosense
