#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/DistanceRangeAvailability.hpp"
#include <limits>

TEST_CASE("Research range acquisition obeys the frozen850 and1000 metre anchors") {
    gf::DistanceRangeAvailability model;model.enabled=true;model.link_seed=131211;
    CHECK(model.acquisitionProbability(0)==1);
    CHECK(model.acquisitionProbability(849)==1);
    CHECK(model.acquisitionProbability(850)==1);
    CHECK(model.acquisitionProbability(1000)==doctest::Approx(.05).epsilon(1e-12));
    CHECK(model.acquisitionProbability(1150)==doctest::Approx(.0025).epsilon(1e-12));
    CHECK(model.acquired(850,.999999));
    CHECK(model.acquired(1000,.049));
    CHECK_FALSE(model.acquired(1000,.051));
}

TEST_CASE("The distance model is default off and rejects nonphysical inputs") {
    gf::DistanceRangeAvailability model;
    CHECK_FALSE(model.enabled);CHECK(model.acquisitionProbability(5000)==1);
    model.enabled=true;
    CHECK(model.acquisitionProbability(925)==doctest::Approx(.22360679774997896));
    double previous=1;
    for(int d=0;d<=5000;d+=5) {
        const auto p=model.acquisitionProbability(d);CHECK(p<=previous);CHECK(p>=0);previous=p;
    }
    CHECK_THROWS(model.acquisitionProbability(-1));
    CHECK_THROWS(model.acquisitionProbability(std::numeric_limits<double>::quiet_NaN()));
    CHECK_THROWS(model.acquisitionProbability(std::numeric_limits<double>::infinity()));
    CHECK_THROWS(model.acquired(1000,1));CHECK_THROWS(model.acquired(1000,-.1));
}
