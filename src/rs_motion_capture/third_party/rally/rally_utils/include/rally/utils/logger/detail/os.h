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
#ifndef RALLY_UTILS_LOGGER_DETAIL_OS_H
#define RALLY_UTILS_LOGGER_DETAIL_OS_H

#include <ctime>
#include "rally/utils/logger/common.h"

namespace rally {
namespace log {
namespace os {

log_clock::time_point now() noexcept;

std::tm localtime(const std::time_t &time_tt) noexcept;

std::tm localtime() noexcept;

std::tm gmtime(const std::time_t &time_tt) noexcept;

std::tm gmtime() noexcept;

bool fopen_s(FILE **fp, const std::string &filename, const std::string &mode);

int remove(const std::string &filename) noexcept;

int rename(const std::string &filename1, const std::string &filename2) noexcept;

static const std::string::value_type folder_seps_filename[] = "/";

bool path_exists(const std::string &filename) noexcept;

size_t filesize(FILE *f);

size_t _thread_id() noexcept;

size_t thread_id() noexcept;

void sleep_for_millis(unsigned int milliseconds) noexcept;

constexpr static const char *default_eol = "\n";

std::string dir_name(std::string path);

bool create_dir(std::string path);

}  // namespace os
}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_DETAIL_OS_H
