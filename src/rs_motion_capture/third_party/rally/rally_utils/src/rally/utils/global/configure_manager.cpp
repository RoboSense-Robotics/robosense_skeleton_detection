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

#include "rally/utils/global/configure_manager.h"
#include "rally/utils/file_utils/file_utils.h"

namespace rally {

void ConfigureManager::setConfigFile(const std::string& config_file) {
    std::string::size_type format_begin = config_file.find_last_of('/');
    std::string root_dir = config_file.substr(0, format_begin);

    cfg_node_ = YAML::Null;
    RENSURE(loadFile(config_file, cfg_node_));
    cfg_path_ = root_dir;
    cfg_node_["root_dir"] = cfg_path_;
    catYAML(cfg_node_);
}

bool ConfigureManager::catYAML(YAML::Node &node) {
    if (node.Type() != YAML::NodeType::Map && node.Type() != YAML::NodeType::Sequence) {
        return true;
    }
    if (node.Type() == YAML::NodeType::Map) {
        std::string include_str;
        if (yamlRead(node, "include", include_str)) {
            auto path = getAbsolutePath(cfg_path_, include_str);
            YAML::Node sub_node;
            if (loadFile(path, sub_node)) {
                if (!catYAML(sub_node)) {
                    return false;
                }
                node = sub_node;
            }
        } else {
            for (auto it = node.begin(); it != node.end(); ++it) {
                YAML::Node &sub_node = it->second;
                if (!catYAML(sub_node)) {
                    return false;
                }
            }
        }
    } else {  // process sequence
        for (auto it = node.begin(); it != node.end(); ++it) {
            YAML::Node sub_node = *it;
            catYAML(sub_node);
        }
    }
    return true;
}

}  // namespace rally
