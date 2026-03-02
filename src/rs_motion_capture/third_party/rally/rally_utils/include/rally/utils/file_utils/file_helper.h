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
#ifndef RALLY_UTILS_FILE_UTILS_FILE_HELPER_H
#define RALLY_UTILS_FILE_UTILS_FILE_HELPER_H

#include <string>
#include <fstream>
#include <sstream>

namespace rally {

/// @brief Get absolute path by concatenating prefix and relative_path.
/// @return The absolute path.
std::string getAbsolutePath(const std::string &prefix, const std::string &relative_path);

/// @brief Check if the directory specified by directory_path exists and is indeed a directory.
/// @param[in] directory_path Directory path.
/// @return If the directory specified by directory_path exists and is indeed a directory.
bool directoryExists(const std::string &directory_path);

/// @brief Check if a specified directory specified by directory_path exists.
///        If not, recursively create the directory (and its parents).
/// @param[in] directory_path Directory path.
/// @return If the directory does exist or its creation is successful.
bool ensureDirectory(const std::string &directory_path);

/// @brief Get current absolute directory path
/// @return retuen the path
std::string getCurrentPath();

/// @brief Check if the path exists.
/// @param[in] path a file name, such as /a/b/c.txt
/// @return If the path exists.
bool pathExists(const std::string &path);

}  // namespace rally

#include "rally/utils/file_utils/impl/file_helper.h"

#endif  // RALLY_UTILS_FILE_UTILS_FILE_HELPER_H
