#include <catch2/catch_all.hpp>

#include <zenslam/option.h>
#include <zenslam/options.h>

TEST_CASE("Options validation", "[options]")
{
    using zenslam::options;

    SECTION("Defaults pass validation")
    {
        auto opts = options{};
        REQUIRE_NOTHROW(opts.folder.validate());
        REQUIRE_NOTHROW(opts.slam.validate());
    }

    SECTION("Invalid thickness throws")
    {
        auto opts = options{};
        opts.slam.keyline_thickness = 0; // invalid
        REQUIRE_THROWS_AS(opts.slam.validate(), std::invalid_argument);
    }

    SECTION("Invalid epipolar threshold throws")
    {
        auto opts = options{};
        opts.slam.epipolar_threshold = -0.1; // invalid
        REQUIRE_THROWS_AS(opts.slam.validate(), std::invalid_argument);
    }

    SECTION("Invalid depth range throws")
    {
        auto opts = options{};
        opts.slam.triangulation_min_depth = 5.0;
        opts.slam.triangulation_max_depth = 2.0;
        REQUIRE_THROWS_AS(opts.slam.validate(), std::invalid_argument);
    }

    SECTION("Invalid KLT params throw")
    {
        auto opts = options{};
        opts.slam.klt_window_size = {0, 31};
        REQUIRE_THROWS_AS(opts.slam.validate(), std::invalid_argument);

        opts.slam.klt_window_size = {31, 0};
        REQUIRE_THROWS_AS(opts.slam.validate(), std::invalid_argument);

        opts.slam.klt_window_size = {31, 31};
        opts.slam.klt_max_level = -1;
        REQUIRE_THROWS_AS(opts.slam.validate(), std::invalid_argument);
    }
}

TEST_CASE("option class basic functionality", "[option]")
{
    using zenslam::option;

    SECTION("Construction with default value")
    {
        option<int> opt(42, "test_int", "A test integer option");
        
        REQUIRE(opt.value() == 42);
        REQUIRE(opt.default_value() == 42);
        REQUIRE(opt.name() == "test_int");
        REQUIRE(opt.description() == "A test integer option");
        REQUIRE_FALSE(opt.is_modified());
    }

    SECTION("Default constructor")
    {
        option<int> opt;
        
        REQUIRE(opt.value() == 0);
        REQUIRE(opt.default_value() == 0);
        REQUIRE(opt.name() == "");
        REQUIRE(opt.description() == "");
    }

    SECTION("Implicit conversion to T")
    {
        option<int> opt(100);
        
        int value = opt; // implicit conversion
        REQUIRE(value == 100);
        
        // Use in arithmetic
        int result = opt + 50;
        REQUIRE(result == 150);
    }

    SECTION("Assignment from T")
    {
        option<int> opt(10);
        
        opt = 20; // assignment from int
        REQUIRE(opt.value() == 20);
        REQUIRE(opt.default_value() == 10);
        REQUIRE(opt.is_modified());
    }

    SECTION("Can be used like a value")
    {
        option<double> opt(3.14);
        
        // Assignment
        opt = 2.71;
        REQUIRE(opt.value() == 2.71);
        
        // Reading
        double x = opt;
        REQUIRE(x == 2.71);
        
        // In expressions
        double y = opt * 2.0;
        REQUIRE(y == Catch::Approx(5.42));
    }

    SECTION("Reset to default")
    {
        option<int> opt(100);
        opt = 200;
        
        REQUIRE(opt.is_modified());
        
        opt.reset();
        REQUIRE(opt.value() == 100);
        REQUIRE_FALSE(opt.is_modified());
    }

    SECTION("Metadata can be set after construction")
    {
        option<std::string> opt("default");
        
        REQUIRE(opt.name() == "my_option");
        REQUIRE(opt.description() == "This is my option");
    }
}

TEST_CASE("option class with different types", "[option]")
{
    using zenslam::option;

    SECTION("String option")
    {
        option<std::string> opt("hello", "greeting", "A greeting message");
        
        REQUIRE(opt.value() == "hello");
        
        opt = "world";
        REQUIRE(opt.value() == "world");
        REQUIRE(opt.is_modified());
        
        std::string str = opt;
        REQUIRE(str == "world");
    }

    SECTION("Bool option")
    {
        option<bool> opt(false, "flag", "A boolean flag");
        
        REQUIRE(opt.value() == false);
        
        opt = true;
        REQUIRE(opt.value() == true);
        
        if (opt) { // implicit conversion in boolean context
            REQUIRE(true);
        } else {
            REQUIRE(false);
        }
    }

    SECTION("Double option")
    {
        option<double> opt(1.5, "threshold", "A threshold value");
        
        REQUIRE(opt.value() == 1.5);
        
        opt = 2.5;
        REQUIRE(opt.value() == Catch::Approx(2.5));
        
        double val = opt + 1.0;
        REQUIRE(val == Catch::Approx(3.5));
    }
}

TEST_CASE("option class copy and move semantics", "[option]")
{
    using zenslam::option;

    SECTION("Copy construction")
    {
        option<int> opt1(42, "test", "description");
        opt1 = 100;
        
        option<int> opt2 = opt1;
        
        REQUIRE(opt2.value() == 100);
        REQUIRE(opt2.default_value() == 42);
        REQUIRE(opt2.name() == "test");
        REQUIRE(opt2.description() == "description");
        REQUIRE(opt2.is_modified());
    }

    SECTION("Move construction")
    {
        option<std::string> opt1("hello", "test", "description");
        opt1 = "world";
        
        option<std::string> opt2 = std::move(opt1);
        
        REQUIRE(opt2.value() == "world");
        REQUIRE(opt2.name() == "test");
    }

    SECTION("Copy assignment")
    {
        option<int> opt1(10, "first", "First option");
        option<int> opt2(20, "second", "Second option");
        
        opt1 = 30;
        opt2 = opt1;
        
        REQUIRE(opt2.value() == 30);
        REQUIRE(opt2.default_value() == 10);
        REQUIRE(opt2.name() == "first");
    }
}

TEST_CASE("option class modification tracking", "[option]")
{
    using zenslam::option;

    SECTION("Initially not modified")
    {
        option<int> opt(42);
        REQUIRE_FALSE(opt.is_modified());
    }

    SECTION("Modified after assignment")
    {
        option<int> opt(42);
        opt = 100;
        REQUIRE(opt.is_modified());
    }

    SECTION("Not modified if assigned same value")
    {
        option<int> opt(42);
        opt = 42;
        REQUIRE_FALSE(opt.is_modified());
    }

    SECTION("Reset clears modification flag")
    {
        option<int> opt(42);
        opt = 100;
        opt.reset();
        REQUIRE_FALSE(opt.is_modified());
    }
}

