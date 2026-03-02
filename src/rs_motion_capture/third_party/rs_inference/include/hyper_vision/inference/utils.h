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

#ifndef HYPER_VISION_INFERENCE_UTILS_H
#define HYPER_VISION_INFERENCE_UTILS_H

#include <dirent.h>
#include <sys/stat.h>

#include <fstream>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "hyper_vision/common/md5.h"
#include "rs_perception/rs_cryptor.h"

// using namespace robosense::perception;

namespace robosense {
namespace inference {

inline bool pathExists(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

inline bool dirExists(const std::string& dir_path) {
  struct stat info;
  return stat(dir_path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

inline std::vector<std::string> getFileInDIR(const std::string& dir_path,
                                             const std::string& extension) {
  std::vector<std::string> res;
  if (pathExists(dir_path)) {
    DIR* directory = opendir(dir_path.c_str());
    if (directory == nullptr) {
      INFER_ERROR << "Cannot open directory " << dir_path;
      exit(kFAILURE);
    }
    struct dirent* entry;
    while ((entry = readdir(directory)) != nullptr) {
      if (strstr(entry->d_name, extension.c_str())) {
        res.emplace_back(std::string(entry->d_name));
      }
    }
    closedir(directory);
  } else {
    INFER_ERROR << dir_path << " not existed!";
    exit(kFAILURE);
  }
  return res;
}

// md5 function use the parameter to construct a MD5 object
// and return the MD5 value
inline std::string md5(const std::string str) { return RsMD5(str).hexdigest(); }

class CryptorParser {
 public:
  inline bool readFileToBuffer(const std::string& in_file,
                               std::vector<char>& buffer) {
    std::ifstream file_stream(in_file.c_str(), std::ios::binary);
    if (file_stream.good()) {
      file_stream.seekg(0, file_stream.end);
      int size = static_cast<int>(file_stream.tellg());
      buffer.clear();
      buffer.resize(size);
      file_stream.seekg(0);
      file_stream.read(buffer.data(), size);
      file_stream.close();
      return true;
    }
    return false;
  }

  inline std::string decrypt(const std::vector<char>& buffer) {
    std::vector<char> encrypt_buffer;
    perception::RsCryptor().decrypt(buffer, encrypt_buffer);
    std::string decrypt_data =
        std::string(encrypt_buffer.begin(), encrypt_buffer.end());
    return decrypt_data;
  }

  inline std::string encrypt(const std::vector<char>& buffer) {
    std::vector<char> decrypt_buffer;
    perception::RsCryptor().encrypt(buffer, decrypt_buffer);
    std::string encrypt_data =
        std::string(decrypt_buffer.begin(), decrypt_buffer.end());
    return encrypt_data;
  }

  inline void parse(const std::string& decrypt_data,
                    std::map<std::string, std::string>& infos_vec,
                    std::string& model_buffer) {
    infos_vec.clear();
    auto pos = decrypt_data.find("|||", 0);
    if (pos != decrypt_data.npos) {
      std::string extra_infos = decrypt_data.substr(0, pos);
      model_buffer =
          decrypt_data.substr(pos + 3, decrypt_data.size() - pos - 3);
      infos_vec = parseInfos(extra_infos);
    } else {
      model_buffer = decrypt_data;
    }
  }

 private:
  inline std::vector<std::string> split(const std::string& str,
                                        const std::string& pattern) {
    std::vector<std::string> res;
    if (str == "") {
      return res;
    }
    std::string strs = str;
    size_t pos = strs.find(pattern);

    while (pos != strs.npos) {
      std::string temp = strs.substr(0, pos);
      res.push_back(temp);
      strs = strs.substr(pos + 1, strs.size());
      pos = strs.find(pattern);
    }

    return res;
  }

  inline std::map<std::string, std::string> parseInfos(
      const std::string& infos) {
    std::vector<std::string> str = split(infos, ">");
    std::map<std::string, std::string> res_map;
    for (size_t i = 0; i < str.size(); ++i) {
      const auto& t_str = str[i];
      auto pos_pre = t_str.find("<", 0);
      auto pos_dot = t_str.find(",", 0);

      std::string key = t_str.substr(pos_pre + 1, pos_dot - pos_pre - 1);
      std::string val = t_str.substr(pos_dot + 1, t_str.size() - pos_dot - 1);
      res_map[key] = val;
    }
    return res_map;
  }
};

}  // namespace inference
}  // namespace robosense

#endif
