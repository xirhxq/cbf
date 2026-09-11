#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32ReducedBatchAdmission.hpp"
TEST_CASE("Only preregistered evaluation fields and A C mechanisms enter reduced batch") {
    for(int arm:{0,2}) {
        CHECK(gf::task32RegisteredReducedBatch(arm,0.,2027,154001));
        CHECK(gf::task32RegisteredReducedBatch(arm,.5,153011,154011));
        CHECK(gf::task32RegisteredReducedBatch(arm,.5,153029,154029));
        CHECK(gf::task32RegisteredReducedBatch(arm,.5,153047,154047));
        CHECK_FALSE(gf::task32RegisteredReducedBatch(arm,.5,153011,154029));
        CHECK_FALSE(gf::task32RegisteredReducedBatch(arm,.5,141101,142101));
        CHECK_FALSE(gf::task32RegisteredReducedBatch(arm,0.,2027,134001));
        CHECK_FALSE(gf::task32RegisteredReducedBatch(arm,.6,153011,154011));
    }
    CHECK_FALSE(gf::task32RegisteredReducedBatch(1,.5,153011,154011));
    CHECK_FALSE(gf::task32RegisteredReducedBatch(-1,0.,2027,154001));
}
