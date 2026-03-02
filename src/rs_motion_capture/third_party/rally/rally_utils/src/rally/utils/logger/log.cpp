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

#include "rally/utils/logger/log.h"

namespace rally {
namespace log {

AsyncLogger::Ptr createAsyncLogger(const std::string& name, const std::vector<BaseSink::Ptr> &sinks,
                                   AsyncOverflowPolicy overflow_policy) {
    auto &mutex = Registry::getInstance().getTpMutex();

    std::lock_guard<std::recursive_mutex> tp_lock(mutex);
    auto tp = Registry::getInstance().getTp();
    if (tp == nullptr) {
        tp.reset(new ThreadPool(8192, 1U));
        Registry::getInstance().setTp(tp);
    }

    AsyncLogger::Ptr new_logger_ptr(new AsyncLogger(name, sinks, tp, overflow_policy));
    new_logger_ptr->setLevel(Level::INFO);

    return new_logger_ptr;
}

Logger::Ptr createSyncLogger(const std::string &name, const std::vector<BaseSink::Ptr> &sinks) {
    Logger::Ptr new_logger_ptr(new Logger(name, sinks));
    new_logger_ptr->setLevel(Level::INFO);

    return new_logger_ptr;
}

}  // namespace log
}  // namespace rally
