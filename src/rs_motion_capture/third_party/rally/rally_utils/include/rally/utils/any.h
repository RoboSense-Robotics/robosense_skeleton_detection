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
#ifndef RALLY_UTILS_ANY_H
#define RALLY_UTILS_ANY_H

#include <memory>
#include <typeinfo>

namespace rally {

class Any {
public:
    using Ptr = std::shared_ptr<Any>;

    Any() : content_(nullptr) {}

    template<typename ValueType>
    explicit Any(const ValueType &value)
    : content_(new Holder<ValueType>(value)) {}

    Any(const Any &other)
    : content_(other.content_ ? other.content_->clone() : nullptr) {}

    ~Any() { delete content_; }

    const std::type_info &type_info() const { return content_ ? content_->type_info() : typeid(void); }

    template<typename ValueType>
    ValueType *AnyCast() {
        return content_ ? &(static_cast<Holder <ValueType> *>(content_)->held_)
                        : nullptr;
    }

private:

    /// @brief interface of the true content
    class PlaceHolder {
    public:
        virtual ~PlaceHolder() {}

        /// @brief interface for getting the type_info
        virtual const std::type_info &type_info() const = 0;

        virtual PlaceHolder *clone() const = 0;
    };

    template<typename ValueType>
    class Holder : public PlaceHolder {
    public:
        explicit Holder(const ValueType &value) : held_(value) {}

        virtual const std::type_info &type_info() const { return typeid(ValueType); }

        virtual ~Holder() {}

        /// @brief Prototype Pattern of design pattern
        virtual PlaceHolder *clone() const { return new Holder(held_); }

        ValueType held_;
    };

    PlaceHolder *content_;
};


}  // namespace rally

#endif  // RALLY_UTILS_ANY_H
