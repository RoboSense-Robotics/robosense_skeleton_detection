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
#ifndef RALLY_UTILS_FILE_UTILS_FILE_UTILS_H
#define RALLY_UTILS_FILE_UTILS_FILE_UTILS_H

#include <dirent.h>
#include <cstring>
#include "rally/utils/file_utils/file_helper.h"
#include "rally/utils/file_utils/file_reader.h"
#include "rally/utils/file_utils/file_writer.h"

namespace rally {

inline std::vector<std::string> FindFiles(const std::string& data_dir, const std::string& target_extension) {
  std::vector<std::string> files;
  DIR* dir = opendir(data_dir.c_str());
  if (dir != nullptr) {
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
        std::string entry_path = data_dir + "/" + entry->d_name;
        struct stat statbuf;
        if (lstat(entry_path.c_str(), &statbuf) != -1) {
          if (S_ISDIR(statbuf.st_mode)) {
            auto res = FindFiles(entry_path, target_extension);
            for (size_t i = 0; i < res.size(); ++i) {
              files.emplace_back(res[i]);
            }
          } else if (S_ISREG(statbuf.st_mode)) {
            std::string file_name = entry->d_name;
            if (file_name.size() > target_extension.size() &&
                file_name.substr(file_name.size() - target_extension.size()) == target_extension) {
              files.emplace_back(entry_path);
            }
          }
        }
      }
    }
    closedir(dir);
  } else {
    RWARN << "can not open " << data_dir;
  }
  return files;
}

}  // namespace rally

#endif  // RALLY_UTILS_FILE_UTILS_FILE_UTILS_H
