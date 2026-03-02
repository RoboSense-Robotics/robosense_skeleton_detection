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

#include "rally/utils/init.h"
#include "rally/utils/file_utils/file_utils.h"
#include "rally/utils/global/configure_manager.h"
#include "rally/utils/logger/sinks/file_sink.h"
#include "rally/utils/logger/sinks/terminal_sink.h"

namespace rally {

void init(const std::string &config_file) {
    if (config_file.size() <= 4) {
        RWARN << "rally_utils: init failed with config_file: " << config_file;
        return;
    }

    std::string::size_type format_begin = config_file.find_last_of('.') + 1;
    std::string file_format = config_file.substr(format_begin, config_file.length() - format_begin);
    if (file_format != "yaml") {
        RWARN << "rally_utild: init with only yaml format file! init failed! " << config_file;
        return;
    }

    ConfigureManager::getInstance().setConfigFile(config_file);

    YAML::Node cfg_node = ConfigureManager::getInstance().getCfgNode();
    YAML::Node general_node;
    yamlSubNode(cfg_node, "general", general_node);

    {  // logger
        YAML::Node logger_node;
        yamlSubNode(general_node, "logger", logger_node);

        std::string log_file = "/tmp/rally.txt";
        std::string log_level = "INFO";
        yamlRead(logger_node, "log_file", log_file);
        yamlRead(logger_node, "level", log_level);
        setLogFile(log_file);
        if (rally::log::kLevelName2Type.find(log_level) != rally::log::kLevelName2Type.end()) {
            rally::log::getLoggerByName("default")->setLevel(rally::log::kLevelName2Type.at(log_level));
        } else {
            RWARN << "rally: not valid log level and set log level as default level info! " << log_level;
        }
    }
}

void setLogFile(const std::string &log_file) {
    rally::log::FileSink::Ptr file_sink_ptr(new rally::log::FileSink(log_file));
    rally::log::getLoggerByName("dump_catcher")->getSinks()[1] = file_sink_ptr;
    rally::log::getLoggerByName("default")->getSinks()[1] = file_sink_ptr;
}

}  // namespace rally
