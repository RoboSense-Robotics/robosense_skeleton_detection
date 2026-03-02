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
#ifndef RALLY_UTILS_DETAILS_THROW_H
#define RALLY_UTILS_DETAILS_THROW_H

#include <stdexcept>
#include <sstream>
#include "rally/utils/logger/log.h"

#define RALLY_THROW_EXCEPTION(ExceptionName, message)                       \
{                                                                           \
  std::ostringstream s;                                                     \
  s << message;                                                             \
  throw ExceptionName(s.str(), __FILE__, __LINE__);                         \
}

namespace rally {

/// @brief A base class for all pcl exceptions which inherits from std::runtime_error
class RallyException : public std::runtime_error {
public:

    RallyException(const std::string &error_description,
                   const char *file_name = nullptr,
                   unsigned line_number = 0)
    : std::runtime_error(createDetailedMessage(error_description,
                                               file_name,
                                               line_number)), file_name_(file_name), line_number_(line_number) {}

    const char *getFileName() const throw() {
        return (file_name_);
    }

    unsigned getLineNumber() const throw() {
        return (line_number_);
    }

    const char *detailedMessage() const throw() {
        return (what());
    }


protected:

    static std::string
    createDetailedMessage(const std::string &error_description,
                          const char *file_name,
                          unsigned line_number) {
        std::ostringstream sstream;

        if (file_name != nullptr) {
            sstream << "in " << file_name << " ";
            if (line_number != 0)
                sstream << "@ " << line_number << " ";
        }
        sstream << ": " << error_description;

        return (sstream.str());
    }

    const char *file_name_;
    unsigned line_number_;
};

namespace internal {

inline void require(const bool condition, const char *function, const char *conditionString) {
    if (!condition) {
        std::stringstream ss;
        ss << "Condition: " << conditionString << " in " << function << " is violated!";
        throw std::runtime_error(ss.str());
    }
}

}  // namespace internal

}  // namespace rally

#define RTHROW(message) RALLY_THROW_EXCEPTION(rally::RallyException, message)

#define RENSURE(condition) rally::internal::require(condition, __PRETTY_FUNCTION__, #condition)

#endif  // RALLY_UTILS_DETAILS_THROW_H
