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

#include "rally/utils/logger/sinks/terminal_sink.h"

namespace rally {
namespace log {

TerminalSink::TerminalSink(FILE *target_file) : target_file_(target_file) {}

void TerminalSink::sink_(const LogMsg &msg) {
    // Wrap the originally formatted message in color codes.
    // If color is not supported in the terminal, log as is instead.
    std::string formatted;
    formatter_.format(msg, formatted);

    print_ccode_(colors_.at(msg.level_));
    print_range_(formatted, 0, formatted.size());
    print_ccode_(reset);

    fflush(target_file_);
}
void TerminalSink::flush_() {
    fflush(target_file_);
}

void TerminalSink::print_ccode_(const std::string &color_code) {
    fwrite(color_code.data(), sizeof(char), color_code.size(), target_file_);
}

void TerminalSink::print_range_(const std::string &formatted, size_t start, size_t end) {
    fwrite(formatted.data() + start, sizeof(char), end - start, target_file_);
}

}  // namespace log
}  // namespace rally
