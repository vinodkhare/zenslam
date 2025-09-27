#pragma once

#include <mutex>
#include <utility>

namespace zenslam
{
    /**
     * @brief Simple thread-safe wrapper for any type T
     *
     * Protects access to an enclosed value with a mutex.
     * Allows implicit conversions from and to T for easy use.
     */
    template<typename T>
    class thread_safe
    {
    public:
        // Default constructor
        thread_safe() = default;

        // Constructor from T (allows implicit conversion)
        // ReSharper disable once CppNonExplicitConvertingConstructor
        explicit thread_safe(T value) :
            _value(std::move(value))
        {
        }

        // Copy constructor
        thread_safe(const thread_safe &other)
        {
            std::lock_guard<std::mutex> lock(other._mutex);
            _value = other._value;
        }

        // Move constructor
        thread_safe(thread_safe &&other) noexcept
        {
            std::lock_guard<std::mutex> lock(other._mutex);
            _value = std::move(other._value);
        }

        // Copy assignment
        thread_safe &operator=(const thread_safe &other)
        {
            if (this != &other)
            {
                // Lock both mutexes (avoid deadlock by acquiring locks in consistent order)
                std::lock(_mutex, other._mutex);
                std::lock_guard<std::mutex> lock1(_mutex, std::adopt_lock);
                std::lock_guard<std::mutex> lock2(other._mutex, std::adopt_lock);
                _value = other._value;
            }
            return *this;
        }

        // Move assignment
        thread_safe &operator=(thread_safe &&other) noexcept
        {
            if (this != &other)
            {
                std::lock(_mutex, other._mutex);
                std::lock_guard<std::mutex> lock1(_mutex, std::adopt_lock);
                std::lock_guard<std::mutex> lock2(other._mutex, std::adopt_lock);
                _value = std::move(other._value);
            }
            return *this;
        }

        // Assignment from T (allows implicit conversion)
        thread_safe &operator=(T value)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _value = std::move(value);
            return *this;
        }

        // Implicit conversion to T (allows reading the value directly)
        operator T() const
        {
            std::lock_guard<std::mutex> lock(_mutex);
            return _value;
        }
        
        // Access to a reference of the protected value
        // Use with caution as the reference is temporary
        T& operator*()
        {
            std::lock_guard<std::mutex> lock(_mutex);
            return _value;
        }
        
        // Const access to a reference of the protected value
        const T& operator*() const
        {
            std::lock_guard<std::mutex> lock(_mutex);
            return _value;
        }
        
        // Access to members of the protected value (thread-safe)
        // Usage: counter->member instead of counter.member
        T* operator->()
        {
            std::lock_guard<std::mutex> lock(_mutex);
            return &_value;
        }
        
        // Const access to members of the protected value (thread-safe)
        const T* operator->() const
        {
            std::lock_guard<std::mutex> lock(_mutex);
            return &_value;
        }

    private:
        T                  _value { };
        mutable std::mutex _mutex { }; // mutable so we can lock in const methods
    };
} // namespace zenslam
