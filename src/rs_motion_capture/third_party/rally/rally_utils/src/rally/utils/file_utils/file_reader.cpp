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

#include "rally/utils/file_utils/file_reader.h"

namespace rally {

FileReader::FileReader(const std::string &f_fileName) {
    m_fileStream.open(f_fileName);
    RENSURE(m_fileStream.is_open());
}

std::pair<bool, std::string> FileReader::getLine() {
    std::pair<bool, std::string> res;
    res.first = static_cast<bool>(std::getline(m_fileStream, res.second));
    return res;
};

std::pair<bool, std::istringstream> FileReader::getLineAsStream() {
    std::pair<bool, std::istringstream> res;
    std::string tmp_str;
    res.first = static_cast<bool>(std::getline(m_fileStream, tmp_str));
    res.second = std::istringstream(tmp_str);
    return res;
};

}  // namespace rally
