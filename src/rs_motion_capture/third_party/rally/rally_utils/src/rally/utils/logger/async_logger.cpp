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

#include <memory>
#include <string>
#include "rally/utils/logger/async_logger.h"

namespace rally {
namespace log {

// send the log message to the thread pool
void AsyncLogger::sink_it_(const LogMsg &msg) {
    if (auto pool_ptr = thread_pool_.lock()) {
        pool_ptr->post_log(shared_from_this(), msg, overflow_policy_);
    } else {
        throw_log_ex("AsyncLogger: ThreadPool doesn't exist anymore");
    }
}

// send flush request to the thread pool
void AsyncLogger::flush_() {
    if (auto pool_ptr = thread_pool_.lock()) {
        pool_ptr->post_flush(shared_from_this(), overflow_policy_);
    } else {
        throw_log_ex("AsyncLogger::flush: ThreadPool doesn't exist anymore!");
    }
}

//
// backend functions - called from the thread pool to do the actual job
//
void AsyncLogger::backend_sink_it_(const LogMsg &msg) {
    for (auto &sink : sinks_) {
        sink->log(msg);
    }
    backend_flush_();
}

void AsyncLogger::backend_flush_() {
    for (auto &sink : sinks_) {
        sink->flush();
    }
}

}  // namespace log
}  // namespace rally
