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

#include "rally/utils/logger/sinks/file_sink.h"

namespace rally {
namespace log {

FileSink::FileSink(const std::string &base_filename, std::size_t max_size,
                   std::size_t max_files, bool rotate_on_open)
: base_file_name_(base_filename), max_size_(max_size), max_files_(max_files) {
    file_helper_.open(calc_filename(base_file_name_, 0));
    current_size_ = file_helper_.size();  // expensive. called only once
    if (rotate_on_open && current_size_ > 0) {
        rotate();
    }
}

const std::string &FileSink::filename() const {
    return file_helper_.filename();
}

void FileSink::sink_(const LogMsg &msg) {
    std::string formatted;
    formatter_.format(msg, formatted);
    current_size_ += formatted.size();
    if (current_size_ > max_size_) {
        rotate();
        current_size_ = formatted.size();
    }
    file_helper_.write(formatted);
}

void FileSink::flush_() {
    file_helper_.flush();
}

void FileSink::rotate() {
    file_helper_.close();
    for (auto i = max_files_; i > 0; --i) {
        std::string src = calc_filename(base_file_name_, i - 1);
        if (!os::path_exists(src)) {
            continue;
        }
        std::string target = calc_filename(base_file_name_, i);

        if (!renameFile(src, target)) {
            // if failed try again after a small delay.
            // this is a workaround to a windows issue, where very high rotation
            // rates can cause the rename to fail with permission denied (because of antivirus?).
            os::sleep_for_millis(100);
            if (!renameFile(src, target)) {
                file_helper_.reopen(true);  // truncate the log file anyway to prevent it to grow beyond its limit!
                current_size_ = 0;
                throw_log_ex("rotating_file_sink: failed renaming " + src + " to " + target, errno);
            }
        }
    }
    file_helper_.reopen(true);
}

std::string FileSink::calc_filename(const std::string &filename, std::size_t index) {
    if (index == 0u) {
        return filename;
    }
    std::string base_name, ext;
    std::tie(base_name, ext) = file_helper_.split_by_extension(filename);
    return base_name + std::to_string(index) + ext;
}

}  // namespace log
}  // namespace rally
