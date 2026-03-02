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
#ifndef RALLY_UTILS_REGISTER_H
#define RALLY_UTILS_REGISTER_H

#include <map>
#include <string>
#include <vector>
#include <memory>
#include "rally/utils/any.h"
#include "rally/utils/logger/log.h"

class ObjectFactory {
public:
    using Ptr = std::shared_ptr<ObjectFactory>;

    ObjectFactory() {}

    virtual ~ObjectFactory() {}

    virtual rally::Any NewInstance() { return rally::Any(); }

private:
    RALLY_DISALLOW_COPY_AND_ASSIGN(ObjectFactory)
};

using FactoryMap = std::map<std::string, ObjectFactory::Ptr>;
using BaseClassMap = std::map<std::string, FactoryMap>;

BaseClassMap &GlobalFactoryMap() noexcept;

bool getRegisteredClasses(
std::string const &base_class_name,
std::vector<std::string> *const registered_derived_classes_names) noexcept;

#define RALLY_REGISTER_REGISTER(base_class)                                         \
class base_class##Register {                                                        \
public:                                                                             \
    static base_class *getInstanceByName(const ::std::string &name) {               \
        FactoryMap &map = GlobalFactoryMap()[#base_class];                          \
        FactoryMap::iterator iter = map.find(name);                                 \
        if (iter == map.end()) {                                                    \
            for (auto c : map) {                                                    \
                RDEBUG << "getInstanceByName: Instance: " << c.first;               \
            }                                                                       \
            RDEBUG << "getInstanceByName: Get instance " << name <<" failed!";      \
            return nullptr;                                                         \
        }                                                                           \
        rally::Any object = iter->second->NewInstance();                            \
        return *(object.AnyCast<base_class *>());                                   \
    }                                                                               \
    static std::vector<base_class *> getAllInstances() {                            \
        std::vector<base_class *> instances;                                        \
        FactoryMap &map = GlobalFactoryMap()[#base_class];                          \
        instances.reserve(map.size());                                              \
        for (auto item : map) {                                                     \
            rally::Any object = item.second->NewInstance();                         \
            instances.push_back(*(object.AnyCast<base_class *>()));                 \
        }                                                                           \
        return instances;                                                           \
    }                                                                               \
    static const ::std::string getUniqInstanceName() {                              \
        FactoryMap &map = GlobalFactoryMap()[#base_class];                          \
        if (map.size() != 1) {                                                      \
            RTHROW("getUniqInstanceName: map size is not 1, is " + map.size());     \
        }                                                                           \
        return map.begin()->first;                                                  \
    }                                                                               \
    static base_class *getUniqInstance() {                                          \
        FactoryMap &map = GlobalFactoryMap()[#base_class];                          \
        if (map.size() != 1) {                                                      \
            RTHROW("getUniqInstance: get map size is not 1, is " + map.size());     \
        }                                                                           \
        rally::Any object = map.begin()->second->NewInstance();                     \
        return *(object.AnyCast<base_class *>());                                   \
    }                                                                               \
    static bool isValid(const ::std::string &name) {                                \
        FactoryMap &map = GlobalFactoryMap()[#base_class];                          \
        return map.find(name) != map.end();                                         \
    }                                                                               \
};                                                                                  \

#define RALLY_REGISTER_CLASS(clazz, name)                                           \
namespace {                                                                         \
class ObjectFactory##name : public ObjectFactory {                                  \
public:                                                                             \
    virtual ~ObjectFactory##name() {}                                               \
    virtual rally::Any NewInstance() {                                              \
        return rally::Any(new name());                                              \
    }                                                                               \
};                                                                                  \
__attribute__((constructor)) void RegisterFactory##name() {                         \
    FactoryMap &map =                                                               \
    GlobalFactoryMap()[#clazz];                                                     \
    if (map.find(#name) == map.end()) {                                             \
        ObjectFactory::Ptr tmp_ptr(new ObjectFactory##name());                      \
        map[#name] = tmp_ptr;                                                       \
    }                                                                               \
}                                                                                   \
}                                                                                   \

#endif  // RALLY_UTILS_REGISTER_H
