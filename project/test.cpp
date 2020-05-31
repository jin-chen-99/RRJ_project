//
// Created by Lilo on 2020/5/28.
//

#include<algorithm>
#define CATCH_CONFIG_RUNNER
#include "lib/catch.hpp"
TEST_CASE("Test with zero", "[classic]")
{
REQUIRE((0) == "0");
}