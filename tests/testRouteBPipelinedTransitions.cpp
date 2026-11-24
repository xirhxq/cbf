#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/RouteBClosedLoopAudit.hpp"
#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <vector>

TEST_CASE("Pinball P6 and DL3 dual-graph planning uses h_b=100m berths and inverted-triangle crossings") {
    const auto p6 = gf::buildPinballTriangulation(/*order_w=*/3, /*span_s=*/400.0, 100.0, 140.0);
    CHECK(p6.mobile_ids.size() == 6);
    CHECK(p6.beacon_ids.size() == 4);
    // P6 has 6 upward faces + 3 inverted faces = 9 faces
    CHECK(p6.faces.size() == 9);

    const std::vector<double> b_xs = {0.0, 400.0, 800.0, 1200.0};
    const auto dl3 = gf::buildStaggeredTriangulation(
        b_xs, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    CHECK(dl3.mobile_ids.size() == 6);

    const auto fwd_moves = gf::planLabelledTransfer(p6, dl3);
    CHECK(fwd_moves.size() == 12);
    // Node 6's withdrawal from P6 starts at face (4, 5, 6) and must traverse inverted face (4, 2, 5)
    bool node6_crossed_inverted_aux2 = false;
    for (const auto& mv : fwd_moves) {
        if (mv.node == 6 && mv.legs.front().kind == "tip") {
            if (mv.legs.front().auxiliary == std::optional<gf::NodeId>(2)) {
                node6_crossed_inverted_aux2 = true;
            }
        }
        for (std::size_t idx = 0; idx + 1 < mv.legs.size(); ++idx) {
            const auto& cur = mv.legs[idx];
            const auto& nxt = mv.legs[idx + 1];
            const bool p1_in = cur.parents.first == nxt.parents.first ||
                               cur.parents.first == nxt.parents.second;
            const bool p2_in = cur.parents.second == nxt.parents.first ||
                               cur.parents.second == nxt.parents.second;
            // At most one parent changes across any single leg handover
            CHECK((p1_in || p2_in));
        }
    }
    CHECK(node6_crossed_inverted_aux2);

    const auto rev_moves = gf::planLabelledTransfer(dl3, p6);
    CHECK(rev_moves.size() == 12);
}

TEST_CASE("Closed-loop Pinball P6 <-> DL3 round-trip executes under 4D D-EKF, degraded UWB, and HOCBF") {
    const std::vector<double> b_xs = {0.0, 400.0, 800.0, 1200.0};
    const auto p6 = gf::buildPinballTriangulation(3, 400.0, 100.0, 140.0);
    const auto dl3 = gf::buildStaggeredTriangulation(
        b_xs, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto fwd_moves = gf::planLabelledTransfer(p6, dl3);
    const auto rev_moves = gf::planLabelledTransfer(dl3, p6);

    gf::RouteBPipelinedConfig exec_cfg;
    exec_cfg.serial_mode = false;
    exec_cfg.v_cruise_mps = 14.0;
    exec_cfg.accel_mps2 = 2.0;
    exec_cfg.d_pipe_m = 60.0;
    exec_cfg.d_clear_m = 32.0;
    exec_cfg.t_settle_s = 1.0;
    exec_cfg.uwb_d_r_m = 600.0;
    exec_cfg.uwb_sigma0_m = 1.0;
    exec_cfg.uwb_d_max_m = 1200.0;

    const auto fwd_res = gf::runClosedLoopReconfiguration(p6, dl3, fwd_moves, exec_cfg);
    CHECK(fwd_res.completed);
    CHECK(fwd_res.static_parent_violations == 0);
    CHECK(fwd_res.dag_violations == 0);
    CHECK(fwd_res.audit.collision_violations == 0);
    CHECK(fwd_res.audit.reference_violations == 0);
    CHECK(fwd_res.audit.axis_speed_violations == 0);
    CHECK(fwd_res.audit.max_axis_speed_mps <= 25.0);
    CHECK(fwd_res.final_position_error_m < 4.0);
    CHECK(fwd_res.final_topology_matched);

    const auto rev_res = gf::runClosedLoopReconfiguration(dl3, p6, rev_moves, exec_cfg);
    CHECK(rev_res.completed);
    CHECK(rev_res.static_parent_violations == 0);
    CHECK(rev_res.dag_violations == 0);
    CHECK(rev_res.audit.collision_violations == 0);
    CHECK(rev_res.audit.reference_violations == 0);
    CHECK(rev_res.audit.axis_speed_violations == 0);
    CHECK(rev_res.final_position_error_m < 4.0);
    CHECK(rev_res.final_topology_matched);
}

TEST_CASE("Closed-loop DL3 <-> L6 pipelined execution accelerates serial baseline with zero static-parent violations") {
    const std::vector<double> b_xs = {0.0, 400.0, 800.0};
    const auto dl3 = gf::buildStaggeredTriangulation(
        b_xs, {{1, 2, 3}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto l6 = gf::buildStaggeredTriangulation(
        b_xs, {{}, {6, 5, 4, 3, 2, 1}}, 380.0, 90.0, 100.0, 140.0);
    const auto moves = gf::planLabelledTransfer(dl3, l6);

    gf::RouteBPipelinedConfig base_cfg;
    base_cfg.v_cruise_mps = 14.0;
    base_cfg.accel_mps2 = 2.0;
    base_cfg.d_pipe_m = 60.0;
    base_cfg.d_clear_m = 32.0;
    base_cfg.t_settle_s = 1.0;
    base_cfg.uwb_d_r_m = 850.0;
    base_cfg.uwb_sigma0_m = 0.5;
    base_cfg.uwb_d_max_m = 1200.0;

    auto serial_cfg = base_cfg;
    serial_cfg.serial_mode = true;
    const auto serial_res = gf::runClosedLoopReconfiguration(dl3, l6, moves, serial_cfg);
    CHECK(serial_res.completed);
    CHECK(serial_res.static_parent_violations == 0);
    CHECK(serial_res.audit.collision_violations == 0);
    CHECK(serial_res.audit.reference_violations == 0);

    auto pipe_cfg = base_cfg;
    pipe_cfg.serial_mode = false;
    const auto pipe_res = gf::runClosedLoopReconfiguration(dl3, l6, moves, pipe_cfg);
    CHECK(pipe_res.completed);
    CHECK(pipe_res.static_parent_violations == 0);
    CHECK(pipe_res.audit.collision_violations == 0);
    CHECK(pipe_res.audit.reference_violations == 0);
    CHECK(pipe_res.makespan_s < serial_res.makespan_s);
}

TEST_CASE("Multiscale closed-loop N=6 (M=3) and N=14 (M=4) direct-matched pipelined merge runs cleanly") {
    for (const auto& [beacon_xs, src_units, dst_units] :
         std::vector<std::tuple<std::vector<double>,
                                std::vector<std::vector<gf::NodeId>>,
                                std::vector<std::vector<gf::NodeId>>>>{
             {{0.0, 400.0, 800.0},
              {{1, 2, 3, 4, 5, 6}, {}},
              {{}, {6, 5, 4, 3, 2, 1}}},
             {{0.0, 400.0, 800.0, 1200.0},
              {{1, 2, 3, 4, 5, 6, 7}, {}, {8, 9, 10, 11, 12, 13, 14}},
              {{}, {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1}, {}}},
         }) {
        const auto src = gf::buildStaggeredTriangulation(
            beacon_xs, src_units, 380.0, 90.0, 100.0, 140.0);
        const auto dst = gf::buildStaggeredTriangulation(
            beacon_xs, dst_units, 380.0, 90.0, 100.0, 140.0);
        const auto [moves, matched_dst] = gf::planDirectMatchedTransfer(src, dst);

        gf::RouteBPipelinedConfig pipe_cfg;
        pipe_cfg.serial_mode = false;
        pipe_cfg.v_cruise_mps = 15.0;
        pipe_cfg.accel_mps2 = 2.0;
        pipe_cfg.d_pipe_m = 60.0;
        pipe_cfg.d_clear_m = 32.0;
        pipe_cfg.t_settle_s = 1.0;
        pipe_cfg.max_sim_time_s = 1400.0;
        pipe_cfg.uwb_d_r_m = 850.0;
        pipe_cfg.uwb_sigma0_m = 0.5;
        pipe_cfg.uwb_d_max_m = 1200.0;

        const auto res = gf::runClosedLoopReconfiguration(src, matched_dst, moves, pipe_cfg);
        std::printf(
            "[TDD-5 Multiscale] N=%zu completed=%d reason=%s makespan=%.1f wall=%.2f mbb=%zu retries=%zu col=%zu ref=%zu spd=%zu\n",
            src.mobile_ids.size(), res.completed ? 1 : 0, res.reason.c_str(),
            res.makespan_s, res.wall_clock_s, res.total_mbb_swaps, res.mbb_retries,
            res.audit.collision_violations, res.audit.reference_violations,
            res.audit.axis_speed_violations);
        CHECK(res.completed);
        CHECK(res.static_parent_violations == 0);
        CHECK(res.dag_violations == 0);
        CHECK(res.audit.collision_violations == 0);
        CHECK(res.audit.reference_violations == 0);
        CHECK(res.audit.axis_speed_violations == 0);
        CHECK(res.final_topology_matched);
    }
}
