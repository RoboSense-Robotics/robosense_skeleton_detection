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

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <tuple>
#include "rally/utils/logger/detail/file_helper.h"
#include "rally/utils/logger/detail/os.h"
#include "rally/utils/logger/common.h"

namespace rally {
namespace log {

FileHelper::~FileHelper() {
    close();
}

void FileHelper::open(const std::string &fname, bool truncate) {
    close();
    filename_ = fname;

    auto *mode = "ab";
    auto *trunc_mode = "wb";

    for (int tries = 0; tries < open_tries_; ++tries) {
        // create containing folder if not exists already.
        os::create_dir(os::dir_name(fname));
        if (truncate) {
            // Truncate by opening-and-closing a tmp file in "wb" mode, always
            // opening the actual log-we-write-to in "ab" mode, since that
            // interacts more politely with eternal processes that might
            // rotate/truncate the file underneath us.
            std::FILE *tmp;
            if (os::fopen_s(&tmp, fname, trunc_mode)) {
                continue;
            }
            std::fclose(tmp);
        }
        if (!os::fopen_s(&fd_, fname, mode)) {
            return;
        }

        os::sleep_for_millis(open_interval_);
    }

    throw_log_ex("Failed opening file " + filename_ + " for writing", errno);
}

void FileHelper::reopen(bool truncate) {
    if (filename_.empty()) {
        throw_log_ex("Failed re opening file - was not opened before");
    }
    this->open(filename_, truncate);
}

void FileHelper::flush() {
    std::fflush(fd_);
}

void FileHelper::close() {
    if (fd_ != nullptr) {
        std::fclose(fd_);
        fd_ = nullptr;
    }
}

void FileHelper::write(const std::string &buf) {
    size_t msg_size = buf.size();
    auto data = buf.data();
    if (std::fwrite(data, 1, msg_size, fd_) != msg_size) {
        throw_log_ex("Failed writing to file " + filename_, errno);
    }
}

size_t FileHelper::size() const {
    if (fd_ == nullptr) {
        throw_log_ex("Cannot use size() on closed file " + filename_);
    }
    return os::filesize(fd_);
}

const std::string &FileHelper::filename() const {
    return filename_;
}

//
// return file path and its extension:
//
// "mylog.txt" => ("mylog", ".txt")
// "mylog" => ("mylog", "")
// "mylog." => ("mylog.", "")
// "/dir1/dir2/mylog.txt" => ("/dir1/dir2/mylog", ".txt")
//
// the starting dot in filenames is ignored (hidden files):
//
// ".mylog" => (".mylog". "")
// "my_folder/.mylog" => ("my_folder/.mylog", "")
// "my_folder/.mylog.txt" => ("my_folder/.mylog", ".txt")
std::tuple<std::string, std::string> FileHelper::split_by_extension(const std::string &fname) {
    auto ext_index = fname.rfind('.');

    // no valid extension found - return whole path and empty string as
    // extension
    if (ext_index == std::string::npos || ext_index == 0 || ext_index == fname.size() - 1) {
        return std::make_tuple(fname, std::string());
    }

    // treat cases like "/etc/rc.d/somelogfile or "/abc/.hiddenfile"
    auto folder_index = fname.find_last_of(os::folder_seps_filename);
    if (folder_index != std::string::npos && folder_index >= ext_index - 1) {
        return std::make_tuple(fname, std::string());
    }

    // finally - return a valid base and extension tuple
    return std::make_tuple(fname.substr(0, ext_index), fname.substr(ext_index));
};

}  // namespace log
}  // namespace rally
