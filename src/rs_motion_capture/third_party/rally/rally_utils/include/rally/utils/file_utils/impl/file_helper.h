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
#ifndef RALLY_UTILS_DETAILS_IMPL_FILE_HELPER_H
#define RALLY_UTILS_DETAILS_IMPL_FILE_HELPER_H

#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>

namespace rally {

inline std::string getAbsolutePath(const std::string &prefix, const std::string &relative_path) {
    if (relative_path.empty()) {
        return prefix;
    }
    // If prefix is empty or relative_path is already absolute.
    if (prefix.empty() || relative_path.front() == '/') {
        return relative_path;
    }

    if (prefix.back() == '/') {
        return prefix + relative_path;
    }
    return prefix + "/" + relative_path;
}

inline bool directoryExists(const std::string &directory_path) {
    struct stat info;
    return stat(directory_path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

inline bool ensureDirectory(const std::string &directory_path) {
    std::string path = directory_path;
    for (size_t i = 1; i < directory_path.size(); ++i) {
        if (directory_path[i] == '/') {
            // Whenever a '/' is encountered, create a temporary view from
            // the start of the path to the character right before this.
            path[i] = 0;

            if (mkdir(path.c_str(), 0777) != 0) {
                if (errno != EEXIST) {
                    return false;
                }
            }

            // Revert the temporary view back to the original.
            path[i] = '/';
        }
    }

    // Make the final (full) directory.
    if (mkdir(path.c_str(), 0777) != 0) {
        if (errno != EEXIST) {
            return false;
        }
    }

    return true;
}

inline std::string getCurrentPath() {
    char tmp[PATH_MAX];
    return getcwd(tmp, sizeof(tmp)) ? std::string(tmp) : std::string("");
}

inline bool pathExists(const std::string &path) {
    struct stat info;
    return stat(path.c_str(), &info) == 0;
}

}  // namespace rally

#endif  // RALLY_UTILS_DETAILS_IMPL_FILE_HELPER_H
