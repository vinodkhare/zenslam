#pragma once

#include <string>
#include <utility>

namespace zenslam
{
    /**
     * @brief A wrapper class that models an option value with metadata
     * @tparam T The type of the wrapped value
     *
     * This class wraps a value of type T and allows it to be used naturally
     * through implicit conversions and assignment operators. It tracks the
     * default value set at construction and provides metadata (name and description).
     */
    template <typename T>
    class option
    {
    public:
        /**
         * @brief Construct an option with a default value
         * @param default_value The default value for this option
         * @param name The name of this option
         * @param description A description of this option
         */
        explicit option(T default_value, std::string name = "", std::string description = "") :
            _value(std::move(default_value)),
            _default_value(_value),
            _name(std::move(name)),
            _description(std::move(description))
        {
        }

        /**
         * @brief Default constructor (requires T to be default-constructible)
         */
        option() :
            _value { },
            _default_value { }
        {
        }

        // Implicit conversion to T (allows using option<T> as T)
        operator T&() { return _value; }
        operator const T&() const { return _value; }

        // Assignment from T (allows assigning T to option<T>)
        option& operator=(const T& new_value)
        {
            _value = new_value;
            return *this;
        }

        option& operator=(T&& new_value)
        {
            _value = std::move(new_value);
            return *this;
        }

        // Copy and move constructors/assignments
        option(const option&)            = default;
        option(option&&)                 = default;
        option& operator=(const option&) = default;
        option& operator=(option&&)      = default;

        /**
         * @brief Get the current value
         * @return Reference to the current value
         */
        T&       value() { return _value; }
        const T& value() const { return _value; }

        /**
         * @brief Get the default value
         * @return Reference to the default value
         */
        const T& default_value() const { return _default_value; }

        /**
         * @brief Get the name of this option
         * @return The name string
         */
        const std::string& name() const { return _name; }

        /**
         * @brief Get the description of this option
         * @return The description string
         */
        const std::string& description() const { return _description; }

        /**
         * @brief Reset the value to its default
         */
        void reset() { _value = _default_value; }

        /**
         * @brief Check if the current value differs from the default
         * @return true if the value has been modified from the default
         */
        bool is_modified() const { return _value != _default_value; }

    private:
        T           _value         = { };
        T           _default_value = { };
        std::string _name          = { };
        std::string _description   = { };
    };
} // namespace zenslam
