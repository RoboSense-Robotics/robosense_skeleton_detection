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

#ifndef ROBOSENSE_INIT_H_
#define ROBOSENSE_INIT_H_

#include "rs_log/common/log.h"
#include "rs_log/state.h"

namespace robosense {
namespace log {

// 固定日志路径: "/mnt/ssd/RDCS_LOG_ROOT"
// 固定日志等级: "INFO"
bool Init(const char *module_name); 

bool Init(const char *module_name, const std::string &log_dir,
          const std::string &log_level = "INFO");

void Clear();

} // namespace log
} // namespace robosense

#endif // ROBOSENSE_INIT_H_
