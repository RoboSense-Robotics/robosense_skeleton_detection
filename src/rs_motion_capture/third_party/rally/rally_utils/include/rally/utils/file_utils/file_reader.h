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
#ifndef RALLY_UTILS_FILE_UTILS_FILE_READER_H
#define RALLY_UTILS_FILE_UTILS_FILE_READER_H

#include <memory>
#include "rally/utils/file_utils/file_helper.h"
#include "rally/utils/details/macros.h"
#include "rally/utils/details/throw.h"

namespace rally {

class FileReader {
public:
    using Ptr = std::shared_ptr<FileReader>;

    FileReader(const std::string &f_fileName);

    ~FileReader() { m_fileStream.close(); }

    std::pair<bool, std::string> getLine();

    std::pair<bool, std::istringstream> getLineAsStream();

private:
    RALLY_DISALLOW_COPY_AND_ASSIGN(FileReader)

    std::ifstream m_fileStream;
};

}  // namespace rally

#endif  // RALLY_UTILS_FILE_UTILS_FILE_READER_H
