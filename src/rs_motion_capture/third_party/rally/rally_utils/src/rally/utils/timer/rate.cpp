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

#include "rally/utils/timer/rate.h"
#include "rally/utils/logger/log.h"

namespace rally {

Rate::Rate(double frequency)
: start_(Time::getNow()),
  expected_cycle_time_(1.0 / frequency),
  actual_cycle_time_(0.0) {}

Rate::Rate(uint64_t nano_seconds)
: start_(Time::getNow()),
  expected_cycle_time_(static_cast<int64_t>(nano_seconds)),
  actual_cycle_time_(0.0) {}

Rate::Rate(const Duration& d)
: start_(Time::getNow()), expected_cycle_time_(d), actual_cycle_time_(0.0) {}

void Rate::sleep() {
    Time expected_end = start_ + expected_cycle_time_;

    Time actual_end = Time::getNow();

    // detect backward jumps in time
    if (actual_end < start_) {
        RWARN << "Detect backward jumps in time";
        expected_end = actual_end + expected_cycle_time_;
    }

    // calculate the time we'll sleep for
    Duration sleep_time = expected_end - actual_end;

    // set the actual amount of time the loop took in case the user wants to kNow
    actual_cycle_time_ = actual_end - start_;

    // make sure to reset our start time
    start_ = expected_end;

    // if we've taken too much time we won't sleep
    if (sleep_time < Duration(0.0)) {
        RWARN << "Detect forward jumps in time";
        // if we've jumped forward in time, or the loop has taken more than a full
        // extra cycle, reset our cycle
        if (actual_end > expected_end + expected_cycle_time_) {
            start_ = actual_end;
        }
        // return false to show that the desired rate was not met
        return;
    }

    Time::sleepUntil(expected_end);
}

void Rate::reset() { start_ = Time::getNow(); }

Duration Rate::getCycleTime() const { return actual_cycle_time_; }

}  // namespace rally
