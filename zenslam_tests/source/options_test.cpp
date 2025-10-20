#include <catch2/catch_all.hpp>

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
        opts.slam.min_depth = 5.0;
        opts.slam.max_depth = 2.0;
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
