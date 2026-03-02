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

#include "rally/utils/register.h"

BaseClassMap &GlobalFactoryMap() noexcept {
    static BaseClassMap factory_map;
    return factory_map;
}

bool getRegisteredClasses(std::string const &base_class_name,
                          std::vector<std::string> *const registered_derived_classes_names) noexcept {
    if (registered_derived_classes_names == nullptr) {
        RERROR << "registered_derived_classes_names is not available!";
        return false;
    }
    BaseClassMap &map = GlobalFactoryMap();
    const auto &iter = map.find(base_class_name);
    if (iter == map.end()) {
        RERROR << "class[" << base_class_name << "] not registered!";
        return false;
    }
    for (const auto &pair : iter->second) {
        registered_derived_classes_names->push_back(pair.first);
    }
    return true;
}