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
#ifndef RALLY_UTILS_LOGGER_DETAIL_FILE_HELPER_H
#define RALLY_UTILS_LOGGER_DETAIL_FILE_HELPER_H

#include <tuple>
#include "rally/utils/logger/common.h"

namespace rally {
namespace log {

class FileHelper {
public:
    explicit FileHelper() = default;

    FileHelper(const FileHelper &) = delete;
    FileHelper &operator=(const FileHelper &) = delete;
    ~FileHelper();

    void open(const std::string &fname, bool truncate = false);
    void reopen(bool truncate);
    void flush();
    void close();
    void write(const std::string &buf);
    size_t size() const;
    const std::string &filename() const;

    static std::tuple<std::string, std::string> split_by_extension(const std::string &fname);

private:
    const int open_tries_ = 5;
    const unsigned int open_interval_ = 10;
    std::FILE *fd_{nullptr};
    std::string filename_;
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_DETAIL_FILE_HELPER_H
