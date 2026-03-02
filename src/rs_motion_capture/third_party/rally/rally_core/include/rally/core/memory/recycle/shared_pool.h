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
#ifndef RALLY_CORE_MEMORY_RECYCLE_SHARED_POOL_H
#define RALLY_CORE_MEMORY_RECYCLE_SHARED_POOL_H

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <type_traits>
#include <utility>
#include <mutex>
#include "rally/utils/details/macros.h"

namespace rally {

template<typename T>
class SharedPool {
public:
    using ValuePtr = std::shared_ptr<T>;
    using AllocateFunction = std::function<ValuePtr()>;

    SharedPool() :
    m_pool(std::make_shared<impl>(
    AllocateFunction(std::make_shared<T>))) {
    }

    ValuePtr allocate() {
        assert(m_pool);
        return m_pool->allocate();
    }

private:
    class impl : public std::enable_shared_from_this<impl> {
    public:
        impl(AllocateFunction allocate) : m_allocate(std::move(allocate)) {
            assert(m_allocate);
        }

        impl(const impl &other) :
        std::enable_shared_from_this<impl>(other),
        m_allocate(other.m_allocate) {
            std::size_t size = other.unused_resources();
            for (std::size_t i = 0; i < size; ++i) {
                m_free_list.push_back(m_allocate());
            }
        }

        impl(impl &&other) :
        std::enable_shared_from_this<impl>(other),
        m_allocate(std::move(other.m_allocate)),
        m_free_list(std::move(other.m_free_list)) {
        }

        impl &operator=(const impl &other) {
            impl tmp(other);
            std::swap(*this, tmp);
            return *this;
        }

        impl &operator=(impl &&other) {
            m_allocate = std::move(other.m_allocate);
            m_free_list = std::move(other.m_free_list);
            return *this;
        }

        ValuePtr allocate() {
            ValuePtr resource;
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (m_free_list.size() > 0) {
                    resource = m_free_list.back();
                    m_free_list.pop_back();
                }
            }

            if (!resource) {
                assert(m_allocate);
                resource = m_allocate();
            }

            auto pool = impl::shared_from_this();

            return ValuePtr(resource.get(), deleter(pool, resource));
        }

        void recycle(const ValuePtr &resource) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_free_list.push_back(resource);
        }

        std::size_t unused_resources() const {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_free_list.size();
        }

    private:
        AllocateFunction m_allocate;
        std::list<ValuePtr> m_free_list;
        mutable std::mutex m_mutex;
    };

    class deleter {
    public:
        deleter(const std::weak_ptr<impl> &pool, const ValuePtr &resource) :
        m_pool(pool), m_resource(resource) {
            assert(!m_pool.expired());
            assert(m_resource);
        }

        void operator()(T *) {
            auto pool = m_pool.lock();

            if (pool) {
                pool->recycle(m_resource);
            }
            m_resource.reset();
        }

    private:
        std::weak_ptr<impl> m_pool;
        ValuePtr m_resource;
    };
    RALLY_DISALLOW_COPY_AND_ASSIGN(SharedPool)

    std::shared_ptr<impl> m_pool;
};

}  // namespace rally

#endif  // RALLY_CORE_MEMORY_RECYCLE_SHARED_POOL_H
