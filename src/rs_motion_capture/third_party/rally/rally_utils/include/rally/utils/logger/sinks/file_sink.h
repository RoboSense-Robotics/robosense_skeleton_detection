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
#ifndef RALLY_UTILS_LOGGER_SINKS_FILE_SINK_H
#define RALLY_UTILS_LOGGER_SINKS_FILE_SINK_H

#include <mutex>
#include <string>
#include "rally/utils/logger/sinks/base_sink.h"
#include "rally/utils/logger/detail/file_helper.h"

namespace rally {
namespace log {

class FileSink : public BaseSink {
public:
    explicit FileSink(const std::string &base_filename, std::size_t max_size = 1024 * 1024 * 10,
                      std::size_t max_files = 100, bool rotate_on_open = false);
    const std::string &filename() const;

private:
    void rotate();
    bool renameFile(const std::string & src, const std::string& target) {
        (void)os::remove(target);
        return os::rename(src, target) == 0;
    }
    std::string calc_filename(const std::string& filename, std::size_t index);

    void sink_(const LogMsg &msg) override;
    void flush_() override;

    std::string base_file_name_;
    std::size_t max_size_;
    std::size_t max_files_;
    std::size_t current_size_;
    FileHelper file_helper_;
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_SINKS_FILE_SINK_H
