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
#ifndef RALLY_UTILS_YAML_YAML_H
#define RALLY_UTILS_YAML_YAML_H

#include <sstream>
#include <yaml-cpp/yaml.h>
#include "rally/utils/logger/log.h"

namespace rally {

template<typename T>
inline bool yamlRead(const YAML::Node &node, const std::string &key, T &val) {
    try {
        val = node[key].as<T>();
    } catch (std::exception &e) {
        return false;
    }
    return true;
}

inline bool yamlSubNode(const YAML::Node &node, const std::string &key, YAML::Node &ret) {
    try {
        ret = node[key];
    } catch (std::exception &e) {
        return false;
    }
    return true;
}

inline bool loadFile(const std::string &yaml_file, YAML::Node &node) {
    try {
        node = YAML::LoadFile(yaml_file);
    } catch (std::exception &e) {
        std::string error_msg(e.what());
        if (error_msg == "bad file") {
            RWARN << "yaml file do not exist! " << yaml_file;
        } else {
            RERROR << "-- YAML Load Error: ";
            RERROR << "-- In:\n\t" << yaml_file;
            RERROR << "-- What:\n\t" << error_msg;
        }
        return false;
    }

    return true;
}

}  // namespace rally

#endif  // RALLY_UTILS_YAML_YAML_H
