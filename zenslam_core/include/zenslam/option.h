#pragma once

#include <string>
#include <utility>

#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/tuple/elem.hpp>

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
        option(T default_value, std::string name = "", std::string description = "") :
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
        [[nodiscard]] const std::string& name() const { return _name; }

        /**
         * @brief Get the description of this option
         * @return The description string
         */
        [[nodiscard]] const std::string& description() const { return _description; }

        /**
         * @brief Reset the value to its default
         */
        void reset() { _value = _default_value; }

        /**
         * @brief Check if the current value differs from the default
         * @return true if the value has been modified from the default
         */
        [[nodiscard]] bool is_modified() const { return _value != _default_value; }

    private:
        T           _value         = { };
        T           _default_value = { };
        std::string _name          = { };
        std::string _description   = { };
    };
} // namespace zenslam

// ============================================================================
// Macro-based field auto-registration system
// ============================================================================
// This system allows automatic registration of option fields and generation
// of helper methods for iteration without manual maintenance.
//
// Usage:
//   ZENSLAM_DEFINE_OPTIONS(
//       ((type1, name1, default1, "description1"))
//       ((type2, name2, default2, "description2"))
//       ...
//   )
//
// This generates:
//   1. All option field declarations
//   2. all_options() methods that return std::tie() of all fields
//
// The parse_yaml() and print() methods can then use all_options() to iterate.
// ============================================================================

// Extract elements from the tuple format: (type, name, default_val, "description")
#define ZENSLAM_OPTION_TYPE(elem)        BOOST_PP_TUPLE_ELEM(0, elem)
#define ZENSLAM_OPTION_NAME(elem)        BOOST_PP_TUPLE_ELEM(1, elem)
#define ZENSLAM_OPTION_DEFAULT(elem)     BOOST_PP_TUPLE_ELEM(2, elem)
#define ZENSLAM_OPTION_DESC(elem)        BOOST_PP_TUPLE_ELEM(3, elem)

// Declare a single option field
#define ZENSLAM_OPTION_FIELD_DECL(r, data, elem) \
    zenslam::option<ZENSLAM_OPTION_TYPE(elem)> ZENSLAM_OPTION_NAME(elem) = \
    { \
        ZENSLAM_OPTION_DEFAULT(elem), \
        BOOST_PP_STRINGIZE(ZENSLAM_OPTION_NAME(elem)), \
        ZENSLAM_OPTION_DESC(elem) \
    };

// Get the field name for std::tie
#define ZENSLAM_OPTION_TIE_NAME(r, data, elem) \
    ZENSLAM_OPTION_NAME(elem)

// Main macro to define options and generate the all_options() helper
#define ZENSLAM_DEFINE_OPTIONS(seq) \
    public: \
        BOOST_PP_SEQ_FOR_EACH(ZENSLAM_OPTION_FIELD_DECL, _, seq) \
    \
    public: \
        auto all_options() -> decltype(auto) \
        { \
            return std::tie(BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TRANSFORM(ZENSLAM_OPTION_TIE_NAME, _, seq))); \
        } \
        auto all_options() const -> decltype(auto) \
        { \
            return std::tie(BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TRANSFORM(ZENSLAM_OPTION_TIE_NAME, _, seq))); \
        }

// Legacy macro for backward compatibility (for classes not yet converted)
#define ZENSLAM_OPTION(type, name, default_val, description) \
    zenslam::option<type> name = { default_val, #name, description }
