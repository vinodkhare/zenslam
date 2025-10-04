#define CATCH_CONFIG_MAIN

#include <catch2/catch_all.hpp>

TEST_CASE("Hello World Test")
{
    REQUIRE(1 + 1 == 2);
    REQUIRE(std::string("Hello") + " World" == "Hello World");
}
