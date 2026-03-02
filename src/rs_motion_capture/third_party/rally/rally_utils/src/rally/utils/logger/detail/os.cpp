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

#include <sys/stat.h>

#ifdef __QNX__
#include <process.h>
#else
#include <sys/syscall.h>
#endif

#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include "rally/utils/logger/detail/os.h"

namespace rally {
namespace log {
namespace os {

log_clock::time_point now() noexcept {
    return log_clock::now();
}

std::tm localtime(const std::time_t &time_tt) noexcept {
    std::tm tm;
    ::localtime_r(&time_tt, &tm);
    return tm;
}

std::tm localtime() noexcept {
    std::time_t now_t = ::time(nullptr);
    return localtime(now_t);
}

std::tm gmtime(const std::time_t &time_tt) noexcept {
    std::tm tm;
    ::gmtime_r(&time_tt, &tm);
    return tm;
}

std::tm gmtime() noexcept {
    std::time_t now_t = ::time(nullptr);
    return gmtime(now_t);
}

bool fopen_s(FILE **fp, const std::string &filename, const std::string &mode) {
    *fp = ::fopen((filename.c_str()), mode.c_str());
    return *fp == nullptr;
}

int remove(const std::string &filename) noexcept {
    return std::remove(filename.c_str());
}

int rename(const std::string &filename1, const std::string &filename2) noexcept {
    return std::rename(filename1.c_str(), filename2.c_str());
}

bool path_exists(const std::string &filename) noexcept {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

size_t filesize(FILE *f) {
    if (f == nullptr) {
        throw_log_ex("Failed getting file size. fd is null");
    }
    int fd = ::fileno(f);
    struct stat64 st;
    if (::fstat64(fd, &st) == 0) {
        return static_cast<size_t>(st.st_size);
    }
    throw_log_ex("Failed getting file size from fd", errno);
    return 0;  // will not be reached.
}

size_t _thread_id() noexcept {
#ifdef __QNX__
    return static_cast<size_t>(gettid());
#else
    return static_cast<size_t>(::syscall(SYS_gettid));
#endif
}

size_t thread_id() noexcept {
    static thread_local const size_t tid = _thread_id();
    return tid;
}

void sleep_for_millis(unsigned int milliseconds) noexcept {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

static bool mkdir_(const std::string &path) {
    return ::mkdir(path.c_str(), mode_t(0755)) == 0;
}

std::string dir_name(std::string path) {
    auto pos = path.find_last_of(folder_seps_filename);
    return pos != std::string::npos ? path.substr(0, pos) : std::string{};
}

bool create_dir(std::string path) {
    if (path_exists(path)) {
        return true;
    }

    if (path.empty()) {
        return false;
    }

    size_t search_offset = 0;
    do {
        auto token_pos = path.find_first_of(folder_seps_filename, search_offset);
        // treat the entire path as a folder if no folder separator not found
        if (token_pos == std::string::npos) {
            token_pos = path.size();
        }

        auto subdir = path.substr(0, token_pos);

        if (!subdir.empty() && !path_exists(subdir) && !mkdir_(subdir)) {
            return false;  // return error if failed creating dir
        }
        search_offset = token_pos + 1;
    } while (search_offset < path.size());

    return true;
}

}  // namespace os
}  // namespace log
}  // namespace rally
