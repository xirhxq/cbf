#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/HybridSupervisor.hpp"

namespace {

struct FakeClock {
    double now = 0.0;
    void advance(double seconds) { now += seconds; }
};

gf::TransitionCertificate validCertificate() {
    gf::TransitionCertificate certificate;
    certificate.valid = true;
    certificate.forward_valid = true;
    certificate.reverse_valid = true;
    certificate.topology_version = 7;
    certificate.estimator_version = 13;
    certificate.old_edges = {{10, 1}, {11, 1}, {10, 2}, {1, 2}};
    certificate.new_edge = gf::DirectedEdge{11, 2};
    certificate.old_edge = gf::DirectedEdge{10, 2};
    certificate.union_edges = {
        {10, 1}, {11, 1}, {10, 2}, {1, 2}, {11, 2}};
    certificate.successor_edges = {
        {10, 1}, {11, 1}, {1, 2}, {11, 2}};
    return certificate;
}

gf::TransitionCertificate refreshedCertificate() {
    auto certificate = validCertificate();
    certificate.topology_version = 8;
    certificate.estimator_version = 14;
    return certificate;
}

}  // namespace

TEST_CASE("Dwell and gamma hysteresis prevent supervisor chatter") {
    gf::HybridSupervisor supervisor({1.0, 0.2, 0.5});
    FakeClock clock;

    clock.advance(0.5);
    CHECK(supervisor.observeGamma(clock.now, 0.1, true, true)
          == gf::SupervisorMode::Search);
    clock.advance(0.5);
    CHECK(supervisor.observeGamma(clock.now, 0.1, true, true)
          == gf::SupervisorMode::Reform);
    clock.advance(0.5);
    CHECK(supervisor.observeGamma(clock.now, 0.6, true, true)
          == gf::SupervisorMode::Reform);
    clock.advance(0.5);
    CHECK(supervisor.observeGamma(clock.now, 0.6, true, true)
          == gf::SupervisorMode::Search);
}

TEST_CASE("No candidate enters certified retreat or fail-closed hold") {
    FakeClock clock;
    clock.now = 1.0;
    gf::HybridSupervisor retreat({1.0, 0.2, 0.5});
    CHECK(retreat.observeGamma(clock.now, 0.1, false, true)
          == gf::SupervisorMode::Retreat);

    gf::HybridSupervisor hold({1.0, 0.2, 0.5});
    CHECK(hold.observeGamma(clock.now, 0.1, false, false)
          == gf::SupervisorMode::Hold);
}

TEST_CASE("An explicit retreat request respects positive dwell") {
    gf::HybridSupervisor supervisor({1.0, 0.2, 0.5});
    CHECK(supervisor.requestRetreat(0.5, true)
          == gf::SupervisorMode::Search);
    CHECK(supervisor.requestRetreat(1.0, true)
          == gf::SupervisorMode::Retreat);
}

TEST_CASE("A material topology request can trigger reform independently of gamma warning") {
    gf::HybridSupervisor supervisor({0.0, 0.2, 0.5});
    CHECK(supervisor.requestReformation(0.0, true, true)
          == gf::SupervisorMode::Reform);
}

TEST_CASE("Make rejects version races and never installs the union graph") {
    gf::HybridSupervisor supervisor({0.0, 0.2, 0.5});
    supervisor.observeGamma(0.0, 0.1, true, true);
    const auto certificate = validCertificate();

    CHECK_FALSE(supervisor.beginMakeBeforeBreak(certificate, 8, 13, 0.1));
    CHECK(supervisor.mode() == gf::SupervisorMode::Hold);
    CHECK(supervisor.topology().empty());
}

TEST_CASE("Make rejects a certificate for a different current graph") {
    gf::HybridSupervisor supervisor({0.0, 0.2, 0.5});
    supervisor.initializeTopology(
        {{10, 1}, {12, 1}, {10, 2}, {1, 2}}, 7);
    supervisor.observeGamma(0.0, 0.1, true, true);

    CHECK_FALSE(supervisor.beginMakeBeforeBreak(
        validCertificate(), 7, 13, 0.1));
    CHECK(supervisor.mode() == gf::SupervisorMode::Hold);
    CHECK(supervisor.topology() ==
          std::vector<gf::DirectedEdge>{
              {10, 1}, {12, 1}, {10, 2}, {1, 2}});
}

TEST_CASE("A version-consistent transition installs union before successor") {
    gf::HybridSupervisor supervisor({0.0, 0.2, 0.5});
    supervisor.initializeTopology(validCertificate().old_edges, 7);
    supervisor.observeGamma(0.0, 0.1, true, true);
    const auto certificate = validCertificate();

    REQUIRE(supervisor.beginMakeBeforeBreak(certificate, 7, 13, 0.1));
    CHECK(supervisor.mode() == gf::SupervisorMode::Union);
    CHECK(supervisor.topology() == certificate.union_edges);
    CHECK(supervisor.topologyVersion() == 8);

    REQUIRE(supervisor.finishMakeBeforeBreak(
        refreshedCertificate(), 8, 14, 0.2));
    CHECK(supervisor.mode() == gf::SupervisorMode::Search);
    CHECK(supervisor.topology() == certificate.successor_edges);
    CHECK(supervisor.topologyVersion() == 9);
}

TEST_CASE("A race between make and break holds the certified union") {
    gf::HybridSupervisor supervisor({0.0, 0.2, 0.5});
    supervisor.initializeTopology(validCertificate().old_edges, 7);
    supervisor.observeGamma(0.0, 0.1, true, true);
    const auto certificate = validCertificate();
    REQUIRE(supervisor.beginMakeBeforeBreak(certificate, 7, 13, 0.1));

    CHECK_FALSE(supervisor.finishMakeBeforeBreak(
        validCertificate(), 8, 14, 0.2));
    CHECK(supervisor.mode() == gf::SupervisorMode::Hold);
    CHECK(supervisor.topology() == certificate.union_edges);
}

TEST_CASE("Break rejects a fresh certificate for a different replacement") {
    gf::HybridSupervisor supervisor({0.0, 0.2, 0.5});
    supervisor.initializeTopology(validCertificate().old_edges, 7);
    supervisor.observeGamma(0.0, 0.1, true, true);
    REQUIRE(supervisor.beginMakeBeforeBreak(
        validCertificate(), 7, 13, 0.1));

    auto foreign = refreshedCertificate();
    foreign.old_edge = gf::DirectedEdge{10, 1};
    CHECK_FALSE(supervisor.finishMakeBeforeBreak(foreign, 8, 14, 0.2));
    CHECK(supervisor.mode() == gf::SupervisorMode::Hold);
    CHECK(supervisor.topology() == validCertificate().union_edges);
}
