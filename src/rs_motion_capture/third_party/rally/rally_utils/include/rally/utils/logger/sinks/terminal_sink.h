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
#ifndef RALLY_UTILS_LOGGER_SINKS_TERMINAL_SINK_H
#define RALLY_UTILS_LOGGER_SINKS_TERMINAL_SINK_H

#include <memory>
#include <mutex>
#include <string>
#include "rally/utils/logger/common.h"
#include "rally/utils/logger/sinks/base_sink.h"

namespace rally {
namespace log {

class TerminalSink final : public BaseSink {
public:
    explicit TerminalSink(FILE *target_file);
    ~TerminalSink() = default;

private:
    RALLY_DISALLOW_COPY_AND_ASSIGN(TerminalSink)

    FILE *target_file_;
    const std::map<Level, std::string> colors_ = {
    {Level::TRACE, white},
    {Level::DEBUG, cyan},
    {Level::INFO, green_bold},
    {Level::WARN, yellow_bold},
    {Level::ERROR, red_bold},
    {Level::OFF, reset},
    };

    void sink_(const LogMsg &msg) override;
    void flush_() override;

    void print_ccode_(const std::string &color_code);
    void print_range_(const std::string &formatted, size_t start, size_t end);
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_SINKS_TERMINAL_SINK_H
