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
#ifndef RALLY_CORE_CONCURRENCY_ATOMIC_HASH_MAP_H
#define RALLY_CORE_CONCURRENCY_ATOMIC_HASH_MAP_H

#include <atomic>
#include <cstdint>
#include <type_traits>
#include <utility>
#include "rally/utils/details/macros.h"

namespace rally {

/// @brief A implementation of lock-free fixed size hash map
/// @tparam K Type of key, must be integral
/// @tparam V Type of value
/// @tparam TableSize 28 Size of hash table
/// @tparam 0 Type traits, use for checking types of key & value
template<typename K, typename V, std::size_t TableSize = 128,
typename std::enable_if<std::is_integral<K>::value &&
                        (TableSize & (TableSize - 1)) == 0,
int>::type = 0>
class AtomicHashMap {
public:
    AtomicHashMap() : capacity_(TableSize), mode_num_(capacity_ - 1) {}

    bool has(K key) {
        uint64_t index = key & mode_num_;
        return table_[index].has(key);
    }

    bool get(K key, V **value) {
        uint64_t index = key & mode_num_;
        return table_[index].get(key, value);
    }

    bool get(K key, V *value) {
        uint64_t index = key & mode_num_;
        V *val = nullptr;
        bool res = table_[index].get(key, &val);
        if (res) {
            *value = *val;
        }
        return res;
    }

    void set(K key) {
        uint64_t index = key & mode_num_;
        table_[index].insert(key);
    }

    void set(K key, const V &value) {
        uint64_t index = key & mode_num_;
        table_[index].insert(key, value);
    }

    void set(K key, V &&value) {
        uint64_t index = key & mode_num_;
        table_[index].insert(key, std::forward<V>(value));
    }

private:
    struct Entry {
        Entry() {}

        explicit Entry(K key) : key(key) {
            value_ptr.store(new V(), std::memory_order_release);
        }

        Entry(K key, const V &value) : key(key) {
            value_ptr.store(new V(value), std::memory_order_release);
        }

        Entry(K key, V &&value) : key(key) {
            value_ptr.store(new V(std::forward<V>(value)), std::memory_order_release);
        }

        ~Entry() { delete value_ptr.load(std::memory_order_acquire); }

        K key = 0;
        std::atomic<V *> value_ptr = {nullptr};
        std::atomic<Entry *> next = {nullptr};
    };

    class Bucket {
    public:
        Bucket() : head_(new Entry()) {}

        ~Bucket() {
            Entry *ite = head_;
            while (ite) {
                auto tmp = ite->next.load(std::memory_order_acquire);
                delete ite;
                ite = tmp;
            }
        }

        bool has(K key) {
            Entry *m_target = head_->next.load(std::memory_order_acquire);
            while (Entry *target = m_target) {
                if (target->key < key) {
                    m_target = target->next.load(std::memory_order_acquire);
                    continue;
                } else {
                    return target->key == key;
                }
            }
            return false;
        }

        bool find(K key, Entry **prev_ptr, Entry **target_ptr) {
            Entry *prev = head_;
            Entry *m_target = head_->next.load(std::memory_order_acquire);
            while (Entry *target = m_target) {
                if (target->key == key) {
                    *prev_ptr = prev;
                    *target_ptr = target;
                    return true;
                } else if (target->key > key) {
                    *prev_ptr = prev;
                    *target_ptr = target;
                    return false;
                } else {
                    prev = target;
                    m_target = target->next.load(std::memory_order_acquire);
                }
            }
            *prev_ptr = prev;
            *target_ptr = nullptr;
            return false;
        }

        void insert(K key, const V &value) {
            Entry *prev = nullptr;
            Entry *target = nullptr;
            Entry *new_entry = nullptr;
            V *new_value = nullptr;
            while (true) {
                if (find(key, &prev, &target)) {
                    // key exists, update value
                    if (!new_value) {
                        new_value = new V(value);
                    }
                    auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
                    if (target->value_ptr.compare_exchange_strong(
                    old_val_ptr, new_value, std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                        delete old_val_ptr;
                        if (new_entry) {
                            delete new_entry;
                            new_entry = nullptr;
                        }
                        return;
                    }
                    continue;
                } else {
                    if (!new_entry) {
                        new_entry = new Entry(key, value);
                    }
                    new_entry->next.store(target, std::memory_order_release);
                    if (prev->next.compare_exchange_strong(target, new_entry,
                                                           std::memory_order_acq_rel,
                                                           std::memory_order_relaxed)) {
                        // Insert success
                        if (new_value) {
                            delete new_value;
                            new_value = nullptr;
                        }
                        return;
                    }
                    // another entry has been inserted, retry
                }
            }
        }

        void insert(K key, V &&value) {
            Entry *prev = nullptr;
            Entry *target = nullptr;
            Entry *new_entry = nullptr;
            V *new_value = nullptr;
            while (true) {
                if (find(key, &prev, &target)) {
                    // key exists, update value
                    if (!new_value) {
                        new_value = new V(std::forward<V>(value));
                    }
                    auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
                    if (target->value_ptr.compare_exchange_strong(
                    old_val_ptr, new_value, std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                        delete old_val_ptr;
                        if (new_entry) {
                            delete new_entry;
                            new_entry = nullptr;
                        }
                        return;
                    }
                    continue;
                } else {
                    if (!new_entry) {
                        new_entry = new Entry(key, value);
                    }
                    new_entry->next.store(target, std::memory_order_release);
                    if (prev->next.compare_exchange_strong(target, new_entry,
                                                           std::memory_order_acq_rel,
                                                           std::memory_order_relaxed)) {
                        // Insert success
                        if (new_value) {
                            delete new_value;
                            new_value = nullptr;
                        }
                        return;
                    }
                    // another entry has been inserted, retry
                }
            }
        }

        void insert(K key) {
            Entry *prev = nullptr;
            Entry *target = nullptr;
            Entry *new_entry = nullptr;
            V *new_value = nullptr;
            while (true) {
                if (find(key, &prev, &target)) {
                    // key exists, update value
                    if (!new_value) {
                        new_value = new V();
                    }
                    auto old_val_ptr = target->value_ptr.load(std::memory_order_acquire);
                    if (target->value_ptr.compare_exchange_strong(
                    old_val_ptr, new_value, std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                        delete old_val_ptr;
                        if (new_entry) {
                            delete new_entry;
                            new_entry = nullptr;
                        }
                        return;
                    }
                    continue;
                } else {
                    if (!new_entry) {
                        new_entry = new Entry(key);
                    }
                    new_entry->next.store(target, std::memory_order_release);
                    if (prev->next.compare_exchange_strong(target, new_entry,
                                                           std::memory_order_acq_rel,
                                                           std::memory_order_relaxed)) {
                        // Insert success
                        if (new_value) {
                            delete new_value;
                            new_value = nullptr;
                        }
                        return;
                    }
                    // another entry has been inserted, retry
                }
            }
        }

        bool get(K key, V **value) {
            Entry *prev = nullptr;
            Entry *target = nullptr;
            if (find(key, &prev, &target)) {
                *value = target->value_ptr.load(std::memory_order_acquire);
                return true;
            }
            return false;
        }

        Entry *head_;
    };

    RALLY_DISALLOW_COPY_AND_ASSIGN(AtomicHashMap)

    Bucket table_[TableSize];
    uint64_t capacity_;
    uint64_t mode_num_;
};

}  // namespace rally

#endif  // RALLY_CORE_CONCURRENCY_ATOMIC_HASH_MAP_H
