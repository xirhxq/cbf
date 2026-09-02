#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task17GridTelemetry.hpp"

TEST_CASE("Task 17 GridWorld deltas reconstruct certified and truth masks exactly") {
    GridWorld certified({0.0,30.0},3,{0.0,20.0},2);
    GridWorld truth=certified;
    certified.setValue(0,0,true);
    truth.setValue(0,0,true);
    truth.setValue(1,0,true);
    const auto initial=gf::task17GridSnapshot(certified,truth);
    CHECK(initial.certified_count==1);
    CHECK(initial.truth_count==2);
    CHECK(initial.certified_hash!=initial.truth_hash);

    GridWorld next_certified=certified;
    GridWorld next_truth=truth;
    next_certified.setValue(1,0,true);
    next_certified.setValue(2,1,true);
    next_truth.setValue(2,1,true);
    const auto delta=gf::task17GridDelta(
        certified,truth,next_certified,next_truth);
    CHECK(delta.certified_new_ids==
        std::vector<std::string>{"1:0","2:1"});
    CHECK(delta.truth_new_ids==std::vector<std::string>{"2:1"});
    CHECK(delta.certified_count==3);
    CHECK(delta.truth_count==3);

    const auto rebuilt=gf::task17ApplyGridDelta(initial,delta);
    const auto expected=gf::task17GridSnapshot(next_certified,next_truth);
    CHECK(rebuilt.certified_count==expected.certified_count);
    CHECK(rebuilt.truth_count==expected.truth_count);
    CHECK(rebuilt.certified_hash==expected.certified_hash);
    CHECK(rebuilt.truth_hash==expected.truth_hash);
    CHECK(rebuilt.certified_bits_hex==expected.certified_bits_hex);
    CHECK(rebuilt.truth_bits_hex==expected.truth_bits_hex);
}
