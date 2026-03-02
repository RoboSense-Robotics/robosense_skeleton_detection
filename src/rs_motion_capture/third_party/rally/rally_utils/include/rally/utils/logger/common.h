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
#ifndef RALLY_UTILS_LOGGER_COMMON_H
#define RALLY_UTILS_LOGGER_COMMON_H

#include <map>
#include <atomic>
#include <chrono>
#include <initializer_list>
#include <memory>
#include <exception>
#include <string>
#include <type_traits>
#include <functional>

namespace rally {
namespace log {

// Formatting codes
const static char* reset = "\033[m";
const static char* bold = "\033[1m";
const static char* dark = "\033[2m";
const static char* underline = "\033[4m";
const static char* blink = "\033[5m";
const static char* reverse = "\033[7m";
const static char* concealed = "\033[8m";
const static char* clear_line = "\033[K";

// Foreground colors
const static char* black = "\033[30m";
const static char* red = "\033[31m";
const static char* green = "\033[32m";
const static char* yellow = "\033[33m";
const static char* blue = "\033[34m";
const static char* magenta = "\033[35m";
const static char* cyan = "\033[36m";
const static char* white = "\033[37m";

/// Background colors
const static char* on_black = "\033[40m";
const static char* on_red = "\033[41m";
const static char* on_green = "\033[42m";
const static char* on_yellow = "\033[43m";
const static char* on_blue = "\033[44m";
const static char* on_magenta = "\033[45m";
const static char* on_cyan = "\033[46m";
const static char* on_white = "\033[47m";

/// Bold colors
const static char* yellow_bold = "\033[33m\033[1m";
const static char* red_bold = "\033[31m\033[1m";
const static char* green_bold = "\033[32m\033[1m";
const static char* bold_on_red = "\033[1m\033[41m";

using log_clock = std::chrono::system_clock;

using level_t = std::atomic<int>;

enum class Level {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    OFF = 5,
};

const static std::map<std::string, Level> kLevelName2Type = {
{"TRACE", Level::TRACE},
{"DEBUG", Level::DEBUG},
{"INFO", Level::INFO},
{"WARN", Level::WARN},
{"ERROR", Level::ERROR},
{"OFF", Level::OFF},
};

const static std::map<Level, std::string> kLevelType2Name = {
{Level::TRACE, "TRACE"},
{Level::DEBUG, "DEBUG"},
{Level::INFO, "INFO"},
{Level::WARN, "WARN"},
{Level::ERROR, "ERROR"},
{Level::OFF, "OFF"},
};

class LogEx : public std::exception {
public:
    explicit LogEx(std::string msg);
    LogEx(const std::string &msg, int last_errno);
    const char *what() const noexcept override;

private:
    std::string msg_;
};

void throw_log_ex(const std::string &msg, int last_errno);
void throw_log_ex(std::string msg);

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_COMMON_H
