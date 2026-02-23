#pragma once

#include <functional>
#include <shared_mutex>
#include <vector>

namespace zenslam
{
    template <typename... Args>
    class event
    {
    public:
        using function_type = std::function<void(Args...)>;

        struct connection
        {
            std::size_t index { };
            event*      event { };

            void disconnect() const
            {
                if (event)
                    event->disconnect(index);
            }
        };

        connection operator+=(function_type f)
        {
            std::unique_lock lock(_mutex);

            auto index = _next_index++;

            _slots.push_back({ index, std::move(f) });

            return connection { index, this };
        }

        void disconnect(std::size_t index)
        {
            std::unique_lock lock(_mutex);

            for (auto& slot : _slots)
            {
                if (slot.id == index)
                {
                    slot.id   = _invalid;
                    slot.fn   = nullptr;
                    _is_dirty = true;
                    break;
                }
            }
        }

        void operator()(Args... args)
        {
            std::shared_lock lock(_mutex);

            if (_is_dirty)
            {
                lock.unlock();
                {
                    std::unique_lock ulock(_mutex);

                    _slots.erase
                    (
                        std::remove_if
                        (
                            _slots.begin(),
                            _slots.end(),
                            [](auto& slot)
                            {
                                return slot.index == _invalid;
                            }
                        ),
                        _slots.end()
                    );

                    _is_dirty = false;
                }
                lock.lock();
            }

            for (auto& slot : _slots)
                if (slot.function)
                    slot.function(args...);
        }

    private:
        static constexpr auto _invalid = static_cast<std::size_t>(-1);

        struct slot
        {
            std::size_t   index;
            function_type function;
        };

        std::vector<slot>         _slots { };
        std::size_t               _next_index { };
        bool                      _is_dirty { };
        mutable std::shared_mutex _mutex { };
    };
}
