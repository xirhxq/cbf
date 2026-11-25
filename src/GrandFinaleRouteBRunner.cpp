#include "grand_finale/RouteBClosedLoopAudit.hpp"
#include "grand_finale/RouteBMissionRunner.hpp"
#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"
#include "grand_finale/RouteBPolarEntry.hpp"
#include "grand_finale/RouteBWave9Validations.hpp"
#include "json.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

namespace {

using json = nlohmann::json;

std::string trimWhitespace(std::string s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\t')) {
        s.pop_back();
    }
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {
        ++i;
    }
    return s.substr(i);
}

std::string runShellCapture(const std::string& cmd) {
    std::array<char, 512> buf{};
    std::string out;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return "";
    while (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) {
        out += buf.data();
    }
    pclose(pipe);
    return trimWhitespace(out);
}

std::string resolveExecutablePath() {
#ifdef __APPLE__
    uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    if (size > 0) {
        std::string buf(size, '\0');
        if (_NSGetExecutablePath(buf.data(), &size) == 0) {
            std::error_code ec;
            auto canon = std::filesystem::canonical(std::filesystem::path(buf.c_str()), ec);
            if (!ec) return canon.string();
            return std::string(buf.c_str());
        }
    }
#endif
    return "";
}

json auditToJson(const gf::RouteBClosedLoopAuditSummary& a) {
    json per_pos_rmse = json::object();
    json per_vel_rmse = json::object();
    json per_pos_cov = json::object();
    json per_vel_cov = json::object();
    json per_peak_scale = json::object();
    for (const auto& [id, v] : a.per_agent_position_rmse_m) per_pos_rmse[std::to_string(id)] = v;
    for (const auto& [id, v] : a.per_agent_velocity_rmse_mps) per_vel_rmse[std::to_string(id)] = v;
    for (const auto& [id, v] : a.per_agent_envelope_coverage) per_pos_cov[std::to_string(id)] = v;
    for (const auto& [id, v] : a.per_agent_velocity_envelope_coverage) per_vel_cov[std::to_string(id)] = v;
    for (const auto& [id, v] : a.per_agent_peak_position_scale_radius_m) per_peak_scale[std::to_string(id)] = v;

    return {
        {"observation_count", a.observation_count},
        {"total_agent_samples", a.total_agent_samples},
        {"collision_violations", a.collision_violations},
        {"reference_violations", a.reference_violations},
        {"axis_speed_violations", a.axis_speed_violations},
        {"min_inter_node_distance_m", std::isfinite(a.min_inter_node_distance_m) ? json(a.min_inter_node_distance_m) : json(nullptr)},
        {"max_active_reference_distance_m", a.max_active_reference_distance_m},
        {"max_axis_speed_mps", a.max_axis_speed_mps},
        {"max_position_error_m", a.max_position_error_m},
        {"max_velocity_error_mps", a.max_velocity_error_mps},
        {"position_rmse_m", a.position_rmse_m},
        {"velocity_rmse_mps", a.velocity_rmse_mps},
        {"peak_position_scale_radius_m", a.peak_position_scale_radius_m},
        {"mean_position_scale_radius_m", a.mean_position_scale_radius_m},
        {"peak_position_covariance_trace_m2", a.peak_position_covariance_trace_m2},
        {"overall_envelope_coverage", a.overall_envelope_coverage},
        {"overall_velocity_envelope_coverage", a.overall_velocity_envelope_coverage},
        {"mean_cumulative_abs_yaw_rad", a.mean_cumulative_abs_yaw_rad},
        {"max_cumulative_abs_yaw_rad", a.max_cumulative_abs_yaw_rad},
        {"mean_abs_yaw_rate_radps", a.mean_abs_yaw_rate_radps},
        {"max_abs_yaw_rate_radps", a.max_abs_yaw_rate_radps},
        {"per_agent_position_rmse_m", per_pos_rmse},
        {"per_agent_velocity_rmse_mps", per_vel_rmse},
        {"per_agent_envelope_coverage", per_pos_cov},
        {"per_agent_velocity_envelope_coverage", per_vel_cov},
        {"per_agent_peak_position_scale_radius_m", per_peak_scale},
    };
}

json trajectorySamplesToJson(const std::vector<gf::RouteBTrajectorySample>& samples) {
    json arr = json::array();
    for (const auto& s : samples) {
        json pos = json::object();
        for (const auto& [id, p] : s.positions) {
            pos[std::to_string(id)] = json::array({p.x(), p.y()});
        }
        arr.push_back({
            {"time_s", s.time_s},
            {"positions", pos},
        });
    }
    return arr;
}

json reconfigResultToJson(const gf::RouteBPipelinedRunResult& r) {
    json gantt = json::array();
    for (const auto& g : r.gantt) {
        gantt.push_back({
            {"node", g.node},
            {"start_s", g.start_s},
            {"finish_s", g.finish_s},
            {"duration_s", g.finish_s - g.start_s},
        });
    }
    json out = {
        {"completed", r.completed},
        {"reason", r.reason},
        {"makespan_s", r.makespan_s},
        {"wall_clock_s", r.wall_clock_s},
        {"ticks", r.ticks},
        {"static_parent_violations", r.static_parent_violations},
        {"dag_violations", r.dag_violations},
        {"max_concurrent_moving", r.max_concurrent_moving},
        {"total_mbb_swaps", r.total_mbb_swaps},
        {"mbb_retries", r.mbb_retries},
        {"min_inter_moving_distance_m", std::isfinite(r.min_inter_moving_distance_m) ? json(r.min_inter_moving_distance_m) : json(nullptr)},
        {"final_position_error_m", r.final_position_error_m},
        {"final_topology_matched", r.final_topology_matched},
        {"audit", auditToJson(r.audit)},
        {"gantt", gantt},
    };
    if (!r.scale_radius_series.empty()) {
        json series = json::array();
        for (const auto& pt : r.scale_radius_series) {
            series.push_back({
                {"time_s", pt.time_s},
                {"max_scale_radius_m", pt.max_scale_radius_m},
                {"mean_scale_radius_m", pt.mean_scale_radius_m},
                {"position_rmse_m", pt.position_rmse_m},
            });
        }
        out["scale_radius_series"] = std::move(series);
    }
    if (!r.trajectory_samples.empty()) {
        out["trajectory_samples"] = trajectorySamplesToJson(r.trajectory_samples);
    }
    return out;
}

json missionArmToJson(const gf::RouteBMissionArmResult& arm) {
    json curve = json::array();
    for (const auto& s : arm.coverage_curve) {
        curve.push_back({
            {"time_s", s.time_s},
            {"truth_coverage", s.truth_coverage},
            {"certified_coverage", s.certified_coverage},
        });
    }
    json out = {
        {"arm_name", arm.arm_name},
        {"reconfig_completed", arm.reconfig_completed},
        {"status_reason", arm.status_reason},
        {"reconfig_entry_s", arm.reconfig_entry_s},
        {"reconfig_complete_s", arm.reconfig_complete_s},
        {"final_time_s", arm.final_time_s},
        {"wall_clock_s", arm.wall_clock_s},
        {"reached_t95", arm.reached_t95},
        {"reached_t100", arm.reached_t100},
        {"t95_certified_s", arm.t95_certified_s.has_value() ? json(*arm.t95_certified_s) : json(nullptr)},
        {"t100_certified_s", arm.t100_certified_s.has_value() ? json(*arm.t100_certified_s) : json(nullptr)},
        {"final_truth_coverage", arm.final_truth_coverage},
        {"final_certified_coverage", arm.final_certified_coverage},
        {"false_certified_count", arm.false_certified_count},
        {"certified_subset_of_truth", arm.certified_subset_of_truth},
        {"static_parent_violations", arm.static_parent_violations},
        {"dag_violations", arm.dag_violations},
        {"total_mbb_swaps", arm.total_mbb_swaps},
        {"audit", auditToJson(arm.audit)},
        {"coverage_curve", curve},
    };
    if (!arm.trajectory_samples.empty()) {
        out["trajectory_samples"] = trajectorySamplesToJson(arm.trajectory_samples);
    }
    return out;
}

json provenanceHeader(
    const std::string& protocol = "route-b-wave2-20261123",
    const json& seeds_json = json(20261123)) {
    const std::filesystem::path cbf_root = PROJECT_ROOT;
    const std::filesystem::path parent_root = cbf_root.parent_path();
    const std::string exe_path = resolveExecutablePath();
    const std::string exe_hash = exe_path.empty()
        ? ""
        : runShellCapture("shasum -a 256 \"" + exe_path + "\" | awk '{print $1}'");
    const std::string cbf_commit = runShellCapture("git -C \"" + cbf_root.string() + "\" rev-parse HEAD");
    const std::string cbf_tree = runShellCapture("git -C \"" + cbf_root.string() + "\" rev-parse HEAD^{tree}");
    const std::string cbf_branch = runShellCapture("git -C \"" + cbf_root.string() + "\" rev-parse --abbrev-ref HEAD");
    const std::string cbf_dirty_str = runShellCapture("git -C \"" + cbf_root.string() + "\" status --porcelain");
    const std::string parent_commit = runShellCapture("git -C \"" + parent_root.string() + "\" rev-parse HEAD");
    const std::string gitlink = runShellCapture(
        "git -C \"" + parent_root.string() + "\" ls-tree HEAD cbf | awk '{print $3}'");
    const std::string src_hash = runShellCapture(
        "(cd \"" + cbf_root.string() +
        "\" && find include/grand_finale src -maxdepth 3 -type f \\( -name '*.hpp' -o -name '*.cpp' \\) | LC_ALL=C sort | xargs shasum -a 256 | shasum -a 256 | awk '{print $1}')");

#ifdef NDEBUG
    const std::string build_type = "Release";
#else
    const std::string build_type = "Debug";
#endif

    return {
        {"superproject_repo", "GrandFinale2027-ast-exploration"},
        {"superproject_commit", parent_commit},
        {"superproject_cbf_gitlink", gitlink},
        {"repo", "GrandFinale2027-ast-exploration/cbf"},
        {"branch", cbf_branch.empty() ? std::string("codex/gf-ast-flight-20261120") : cbf_branch},
        {"cbf_commit", cbf_commit},
        {"cbf_tree", cbf_tree},
        {"cbf_working_tree_dirty", !cbf_dirty_str.empty()},
        {"cbf_working_sources_sha256", src_hash},
        {"binary_path", exe_path},
        {"binary_sha256", exe_hash},
        {"build_type", build_type},
#ifdef ENABLE_PIQP
        {"solver_profile", "open_source (PIQP 0.6.1)"},
#else
        {"solver_profile", "open_source (OSQP)"},
#endif
        {"protocol", protocol},
        {"seeds", seeds_json},
        {"dt_s", 0.1},
        {"d_dep_m", 850.0},
        {"d_safe_m", 10.0},
        {"d_max_m", 1200.0},
        {"axis_velocity_limit_mps", 25.0},
        {"acceleration_half_box_mps2", 4.0},
        {"max_yaw_rate_radps", 1.0},
        {"uncertainty_scale", "3*sqrt(tr(P))"},
        {"estimator", "4D Interim-Master D-EKF"},
        {"minimal_runtime_gating", true},
        {"uwb_channel_groups", json::array({
            {{"name", "nominal"}, {"d_r_m", 850.0}, {"sigma0_m", 0.5}, {"d_max_m", 1200.0}},
            {{"name", "degraded"}, {"d_r_m", 600.0}, {"sigma0_m", 1.0}, {"d_max_m", 1200.0}},
        })},
    };
}

void writeJsonFile(const std::filesystem::path& path, const json& doc) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    out << std::setprecision(6) << doc.dump(2) << "\n";
}

json runTdd5PinballAndDl3() {
    json payload;
    payload["provenance"] = provenanceHeader();

    const std::vector<std::tuple<std::string, double, double>> uwb_groups = {
        {"nominal", 850.0, 0.5},
        {"degraded", 600.0, 1.0},
    };

    const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};
    const auto p6 = gf::buildPinballTriangulation(3, 400.0, 100.0, 140.0);
    const auto dl3_b4 = gf::buildStaggeredTriangulation(
        b4, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto p6_to_dl3_moves = gf::planLabelledTransfer(p6, dl3_b4);
    const auto dl3_to_p6_moves = gf::planLabelledTransfer(dl3_b4, p6);

    json p6_runs = json::array();
    for (const auto& [ch_name, d_r, sig0] : uwb_groups) {
        gf::RouteBPipelinedConfig cfg;
        cfg.serial_mode = false;
        cfg.v_cruise_mps = 14.0;
        cfg.accel_mps2 = 2.0;
        cfg.d_pipe_m = 60.0;
        cfg.d_clear_m = 32.0;
        cfg.t_settle_s = 1.0;
        cfg.uwb_d_r_m = d_r;
        cfg.uwb_sigma0_m = sig0;
        cfg.uwb_d_max_m = 1200.0;

        const auto fwd = gf::runClosedLoopReconfiguration(p6, dl3_b4, p6_to_dl3_moves, cfg);
        const auto rev = gf::runClosedLoopReconfiguration(dl3_b4, p6, dl3_to_p6_moves, cfg);
        std::cout << "[Pinball P6<->DL3 " << ch_name << "] fwd=" << fwd.makespan_s
                  << "s rev=" << rev.makespan_s << "s" << std::endl;
        p6_runs.push_back({
            {"channel_group", ch_name},
            {"uwb_d_r_m", d_r},
            {"uwb_sigma0_m", sig0},
            {"p6_to_dl3", reconfigResultToJson(fwd)},
            {"dl3_to_p6", reconfigResultToJson(rev)},
        });
    }
    payload["pinball_p6_roundtrip"] = p6_runs;

    const std::vector<double> b3 = {0.0, 400.0, 800.0};
    const auto dl3 = gf::buildStaggeredTriangulation(
        b3, {{1, 2, 3}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto l6 = gf::buildStaggeredTriangulation(
        b3, {{}, {6, 5, 4, 3, 2, 1}}, 380.0, 90.0, 100.0, 140.0);
    const auto dl3_to_l6_moves = gf::planLabelledTransfer(dl3, l6);

    json dl3_runs = json::array();
    for (const auto& [ch_name, d_r, sig0] : uwb_groups) {
        gf::RouteBPipelinedConfig cfg;
        cfg.v_cruise_mps = 14.0;
        cfg.accel_mps2 = 2.0;
        cfg.d_pipe_m = 60.0;
        cfg.d_clear_m = 32.0;
        cfg.t_settle_s = 1.0;
        cfg.uwb_d_r_m = d_r;
        cfg.uwb_sigma0_m = sig0;
        cfg.uwb_d_max_m = 1200.0;

        cfg.serial_mode = true;
        const auto ser = gf::runClosedLoopReconfiguration(dl3, l6, dl3_to_l6_moves, cfg);
        cfg.serial_mode = false;
        const auto pip = gf::runClosedLoopReconfiguration(dl3, l6, dl3_to_l6_moves, cfg);
        const double speedup = (pip.completed && ser.completed && pip.makespan_s > 0.0)
            ? ser.makespan_s / pip.makespan_s
            : 0.0;
        std::cout << "[DL3->L6 " << ch_name << "] serial=" << ser.makespan_s
                  << "s pipelined=" << pip.makespan_s << "s speedup=" << speedup << "x" << std::endl;
        dl3_runs.push_back({
            {"channel_group", ch_name},
            {"uwb_d_r_m", d_r},
            {"uwb_sigma0_m", sig0},
            {"serial", reconfigResultToJson(ser)},
            {"pipelined", reconfigResultToJson(pip)},
            {"speedup_vs_serial", speedup},
        });
    }
    payload["dl3_to_l6_serial_vs_pipelined"] = dl3_runs;
    return payload;
}

json runTdd5Multiscale() {
    json payload;
    payload["provenance"] = provenanceHeader();

    struct ScaleSpec {
        int n;
        int m;
        std::vector<double> beacon_xs;
        std::vector<std::vector<gf::NodeId>> src_units;
        std::vector<std::vector<gf::NodeId>> dst_units;
        double pipe_cap_s;
        double serial_cap_s;
        bool run_serial;
    };

    const std::vector<ScaleSpec> specs = {
        {6, 3, {0.0, 400.0, 800.0},
         {{1, 2, 3, 4, 5, 6}, {}},
         {{}, {6, 5, 4, 3, 2, 1}},
         1000.0, 1500.0, true},
        {14, 4, {0.0, 400.0, 800.0, 1200.0},
         {{1, 2, 3, 4, 5, 6, 7}, {}, {8, 9, 10, 11, 12, 13, 14}},
         {{}, {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1}, {}},
         1600.0, 3200.0, true},
        {20, 5, {0.0, 400.0, 800.0, 1200.0, 1600.0},
         {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, {}, {}, {11, 12, 13, 14, 15, 16, 17, 18, 19, 20}},
         {{}, {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}, {20, 19, 18, 17, 16, 15, 14, 13, 12, 11}, {}},
         1600.0, 3800.0, true},
    };

    const std::vector<std::tuple<std::string, double, double>> uwb_groups = {
        {"nominal", 850.0, 0.5},
        {"degraded", 600.0, 1.0},
    };

    json rows = json::array();
    for (const auto& sp : specs) {
        const auto src = gf::buildStaggeredTriangulation(
            sp.beacon_xs, sp.src_units, 380.0, 90.0, 100.0, 140.0);
        const auto dst = gf::buildStaggeredTriangulation(
            sp.beacon_xs, sp.dst_units, 380.0, 90.0, 100.0, 140.0);
        const auto [moves, matched_dst] = gf::planDirectMatchedTransfer(src, dst);

        for (const auto& [ch_name, d_r, sig0] : uwb_groups) {
            gf::RouteBPipelinedConfig pcfg;
            pcfg.serial_mode = false;
            pcfg.v_cruise_mps = 15.0;
            pcfg.accel_mps2 = 2.0;
            pcfg.d_pipe_m = 60.0;
            pcfg.d_clear_m = 32.0;
            pcfg.t_settle_s = 1.0;
            pcfg.max_sim_time_s = sp.pipe_cap_s;
            pcfg.uwb_d_r_m = d_r;
            pcfg.uwb_sigma0_m = sig0;
            pcfg.uwb_d_max_m = 1200.0;

            const auto pip = gf::runClosedLoopReconfiguration(src, matched_dst, moves, pcfg);
            std::cout << "[Multiscale N=" << sp.n << " M=" << sp.m << " " << ch_name << " pipe] "
                      << "ok=" << pip.completed << " makespan=" << pip.makespan_s
                      << "s wall=" << pip.wall_clock_s << "s mbb=" << pip.total_mbb_swaps << std::endl;

            json entry = {
                {"scale_N", sp.n},
                {"beacons_M", sp.m},
                {"channel_group", ch_name},
                {"uwb_d_r_m", d_r},
                {"uwb_sigma0_m", sig0},
                {"pipelined", reconfigResultToJson(pip)},
            };

            // Run closed-loop serial baseline (for N=20, run on nominal group so wall clock stays tight)
            const bool run_ser = sp.run_serial && (sp.n < 20 || ch_name == "nominal");
            if (run_ser) {
                auto scfg = pcfg;
                scfg.serial_mode = true;
                scfg.max_sim_time_s = sp.serial_cap_s;
                const auto ser = gf::runClosedLoopReconfiguration(src, matched_dst, moves, scfg);
                const double speedup = (pip.completed && ser.completed && pip.makespan_s > 0.0)
                    ? ser.makespan_s / pip.makespan_s
                    : 0.0;
                std::cout << "[Multiscale N=" << sp.n << " M=" << sp.m << " " << ch_name << " serial] "
                          << "ok=" << ser.completed << " makespan=" << ser.makespan_s
                          << "s wall=" << ser.wall_clock_s << "s speedup=" << speedup << "x" << std::endl;
                entry["serial"] = reconfigResultToJson(ser);
                entry["speedup_vs_serial"] = speedup;
            } else {
                entry["serial"] = nullptr;
                entry["speedup_vs_serial"] = nullptr;
            }
            rows.push_back(std::move(entry));
        }
    }
    payload["scales"] = rows;
    return payload;
}

json runTdd6TwoRegime() {
    json payload;
    payload["provenance"] = provenanceHeader();

    struct MissionSpec {
        std::string domain_name;
        std::string regime_label;
        double t_fork_s;
        double max_mission_s;
    };

    const std::vector<MissionSpec> specs = {
        {"narrow_deep_offshore", "regime_1_physical_reachability", 15.0, 1350.0},
        {"wide_shallow_coastal", "regime_2_time_critical_wide_coastal", 1.5, 700.0},
        {"oblique_asymmetric", "regime_2_time_critical_oblique", 12.0, 650.0},
    };

    const std::vector<std::tuple<std::string, double, double>> uwb_groups = {
        {"nominal", 850.0, 0.5},
        {"degraded", 600.0, 1.0},
    };

    json missions = json::array();
    for (const auto& sp : specs) {
        for (const auto& [ch_name, d_r, sig0] : uwb_groups) {
            gf::RouteBMissionConfig mcfg;
            mcfg.domain_name = sp.domain_name;
            mcfg.t_fork_s = sp.t_fork_s;
            mcfg.max_mission_s = sp.max_mission_s;
            mcfg.patrol_radius_m = 105.0;
            mcfg.v_cruise_mps = (sp.domain_name == "narrow_deep_offshore") ? 15.0 : 14.0;
            mcfg.uwb_d_r_m = d_r;
            mcfg.uwb_sigma0_m = sig0;
            mcfg.uwb_d_max_m = 1200.0;
            mcfg.link_seed = 20261123;
            mcfg.curve_sample_interval_s = 5.0;

            const auto res = gf::runTwoRegimeMissionComparison(mcfg);
            std::cout << "[TDD-6 " << sp.domain_name << " " << ch_name << "] "
                      << "stay_cov=" << res.stay.final_certified_coverage
                      << " sw_cov=" << res.switch_arm.final_certified_coverage
                      << " sw_reconf=" << res.switch_arm.reconfig_complete_s << "s" << std::endl;

            missions.push_back({
                {"domain_name", res.domain_name},
                {"regime_label", sp.regime_label},
                {"channel_group", ch_name},
                {"uwb_d_r_m", d_r},
                {"uwb_sigma0_m", sig0},
                {"valid_cells", res.valid_cells},
                {"area_m2", res.area_m2},
                {"fork_time_s", res.fork_time_s},
                {"checkpoint_byte_identical", res.checkpoint_byte_identical},
                {"fork_truth_coverage", res.fork_truth_coverage},
                {"fork_certified_coverage", res.fork_certified_coverage},
                {"stay", missionArmToJson(res.stay)},
                {"switch", missionArmToJson(res.switch_arm)},
            });
        }
    }
    payload["missions"] = missions;
    return payload;
}

json computeScalarStats(const std::vector<double>& vals) {
    if (vals.empty()) {
        return {
            {"count", 0},
            {"mean", nullptr},
            {"std", nullptr},
            {"min", nullptr},
            {"max", nullptr},
        };
    }
    const double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
    const double mean = sum / static_cast<double>(vals.size());
    double ss = 0.0;
    for (double v : vals) {
        const double d = v - mean;
        ss += d * d;
    }
    const double sd = (vals.size() > 1)
        ? std::sqrt(ss / static_cast<double>(vals.size() - 1))
        : 0.0;
    const auto [mn_it, mx_it] = std::minmax_element(vals.begin(), vals.end());
    return {
        {"count", vals.size()},
        {"mean", mean},
        {"std", sd},
        {"min", *mn_it},
        {"max", *mx_it},
    };
}

double worstAgentValue(const std::map<gf::NodeId, double>& m) {
    if (m.empty()) return 0.0;
    double worst = std::numeric_limits<double>::infinity();
    for (const auto& [id, v] : m) {
        (void)id;
        worst = std::min(worst, v);
    }
    return std::isfinite(worst) ? worst : 0.0;
}

json runTdd8Multiseed() {
    std::vector<unsigned int> seeds;
    for (unsigned int s = 1001u; s <= 1016u; ++s) {
        seeds.push_back(s);
    }

    json payload;
    payload["provenance"] = provenanceHeader("route-b-wave3-tdd8-confirmatory", json(seeds));
    payload["uwb_channel_group"] = "degraded";
    payload["uwb_d_r_m"] = 600.0;
    payload["uwb_sigma0_m"] = 1.0;
    payload["uwb_d_max_m"] = 1200.0;

    const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};
    const auto p6 = gf::buildPinballTriangulation(3, 400.0, 100.0, 140.0);
    const auto dl3 = gf::buildStaggeredTriangulation(
        b4, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto p6_to_dl3 = gf::planLabelledTransfer(p6, dl3);
    const auto dl3_to_p6 = gf::planLabelledTransfer(dl3, p6);

    const auto src14 = gf::buildStaggeredTriangulation(
        b4, {{1, 2, 3, 4, 5, 6, 7}, {}, {8, 9, 10, 11, 12, 13, 14}},
        380.0, 90.0, 100.0, 140.0);
    const auto dst14 = gf::buildStaggeredTriangulation(
        b4, {{}, {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1}, {}},
        380.0, 90.0, 100.0, 140.0);
    const auto [moves14, matched_dst14] = gf::planDirectMatchedTransfer(src14, dst14);

    struct McScenario {
        std::string key;
        std::string label;
        int n;
        int m;
        const gf::RouteBTriangulation& src;
        const gf::RouteBTriangulation& dst;
        const std::vector<gf::RouteBMove>& moves;
        double v_cruise_mps;
        double max_sim_time_s;
    };

    const std::vector<McScenario> scenarios = {
        {"n6_converging_to_dual",
         "N=6, M=4 converging formation to dual staggered chain",
         6, 4, p6, dl3, p6_to_dl3, 14.0, 1200.0},
        {"n6_dual_to_converging",
         "N=6, M=4 dual staggered chain to converging formation",
         6, 4, dl3, p6, dl3_to_p6, 14.0, 1200.0},
        {"n14_dual_to_single_chain",
         "N=14, M=4 dual staggered chain to single deep staggered chain",
         14, 4, src14, matched_dst14, moves14, 15.0, 1600.0},
    };

    json scenario_arr = json::array();
    for (const auto& sc : scenarios) {
        json runs = json::array();
        std::vector<double> makespans;
        std::vector<double> wall_clocks;
        std::vector<double> pos_rmses;
        std::vector<double> vel_rmses;
        std::vector<double> worst_pos_c_envs;
        std::vector<double> worst_vel_c_envs;
        std::vector<double> peak_scales;
        std::vector<double> mean_scales;
        int completed_count = 0;
        int safe_pass_count = 0;

        for (unsigned int seed : seeds) {
            gf::RouteBPipelinedConfig cfg;
            cfg.serial_mode = false;
            cfg.v_cruise_mps = sc.v_cruise_mps;
            cfg.accel_mps2 = 2.0;
            cfg.d_pipe_m = 60.0;
            cfg.d_clear_m = 32.0;
            cfg.t_settle_s = 1.0;
            cfg.max_sim_time_s = sc.max_sim_time_s;
            cfg.uwb_d_r_m = 600.0;
            cfg.uwb_sigma0_m = 1.0;
            cfg.uwb_d_max_m = 1200.0;
            cfg.link_seed = seed;

            const auto res = gf::runClosedLoopReconfiguration(sc.src, sc.dst, sc.moves, cfg);
            const double worst_pos_c = worstAgentValue(res.audit.per_agent_envelope_coverage);
            const double worst_vel_c = worstAgentValue(res.audit.per_agent_velocity_envelope_coverage);
            const bool hard_safe =
                res.completed &&
                res.static_parent_violations == 0 &&
                res.dag_violations == 0 &&
                res.audit.collision_violations == 0 &&
                res.audit.reference_violations == 0 &&
                res.audit.axis_speed_violations == 0;
            if (res.completed) ++completed_count;
            if (hard_safe) ++safe_pass_count;

            makespans.push_back(res.makespan_s);
            wall_clocks.push_back(res.wall_clock_s);
            pos_rmses.push_back(res.audit.position_rmse_m);
            vel_rmses.push_back(res.audit.velocity_rmse_mps);
            worst_pos_c_envs.push_back(worst_pos_c);
            worst_vel_c_envs.push_back(worst_vel_c);
            peak_scales.push_back(res.audit.peak_position_scale_radius_m);
            mean_scales.push_back(res.audit.mean_position_scale_radius_m);

            runs.push_back({
                {"seed", seed},
                {"completed", res.completed},
                {"reason", res.reason},
                {"hard_safety_passed", hard_safe},
                {"makespan_s", res.makespan_s},
                {"wall_clock_s", res.wall_clock_s},
                {"position_rmse_m", res.audit.position_rmse_m},
                {"velocity_rmse_mps", res.audit.velocity_rmse_mps},
                {"peak_position_scale_radius_m", res.audit.peak_position_scale_radius_m},
                {"mean_position_scale_radius_m", res.audit.mean_position_scale_radius_m},
                {"peak_position_covariance_trace_m2", res.audit.peak_position_covariance_trace_m2},
                {"overall_envelope_coverage", res.audit.overall_envelope_coverage},
                {"worst_agent_envelope_coverage", worst_pos_c},
                {"overall_velocity_envelope_coverage", res.audit.overall_velocity_envelope_coverage},
                {"worst_agent_velocity_envelope_coverage", worst_vel_c},
                {"collision_violations", res.audit.collision_violations},
                {"reference_violations", res.audit.reference_violations},
                {"axis_speed_violations", res.audit.axis_speed_violations},
                {"static_parent_violations", res.static_parent_violations},
                {"dag_violations", res.dag_violations},
                {"min_inter_node_distance_m", std::isfinite(res.audit.min_inter_node_distance_m) ? json(res.audit.min_inter_node_distance_m) : json(nullptr)},
                {"max_active_reference_distance_m", res.audit.max_active_reference_distance_m},
                {"max_axis_speed_mps", res.audit.max_axis_speed_mps},
            });
        }

        const auto mk_stats = computeScalarStats(makespans);
        const auto rmse_stats = computeScalarStats(pos_rmses);
        const auto scale_stats = computeScalarStats(peak_scales);
        std::cout << "[Wave3 MC " << sc.key << "] safe=" << safe_pass_count << "/" << seeds.size()
                  << " makespan=" << mk_stats["mean"] << "+-" << mk_stats["std"]
                  << " rmse=" << rmse_stats["mean"] << "+-" << rmse_stats["std"]
                  << " peak_scale=" << scale_stats["mean"] << "+-" << scale_stats["std"] << std::endl;

        scenario_arr.push_back({
            {"key", sc.key},
            {"label", sc.label},
            {"scale_N", sc.n},
            {"beacons_M", sc.m},
            {"uwb_channel_group", "degraded"},
            {"uwb_d_r_m", 600.0},
            {"uwb_sigma0_m", 1.0},
            {"uwb_d_max_m", 1200.0},
            {"runs", std::move(runs)},
            {"summary", {
                {"total_seeds", seeds.size()},
                {"completed_count", completed_count},
                {"hard_safety_pass_count", safe_pass_count},
                {"hard_safety_pass_rate", static_cast<double>(safe_pass_count) / static_cast<double>(seeds.size())},
                {"makespan_s", mk_stats},
                {"wall_clock_s", computeScalarStats(wall_clocks)},
                {"position_rmse_m", rmse_stats},
                {"velocity_rmse_mps", computeScalarStats(vel_rmses)},
                {"worst_agent_envelope_coverage", computeScalarStats(worst_pos_c_envs)},
                {"worst_agent_velocity_envelope_coverage", computeScalarStats(worst_vel_c_envs)},
                {"peak_position_scale_radius_m", scale_stats},
                {"mean_position_scale_radius_m", computeScalarStats(mean_scales)},
            }},
        });
    }

    payload["scenarios"] = std::move(scenario_arr);
    return payload;
}

json runTdd9GeometryContrast() {
    json payload;
    payload["provenance"] = provenanceHeader("route-b-wave3-tdd9-mechanism-contrast", json(1001));
    payload["uwb_channel_group"] = "degraded";
    payload["uwb_d_r_m"] = 600.0;
    payload["uwb_sigma0_m"] = 1.0;
    payload["uwb_d_max_m"] = 1200.0;
    payload["seed"] = 1001;

    const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};

    // (a) Coastal berth height h_b = 40 m vs h_b = 100 m
    const auto src_40 = gf::buildStaggeredTriangulation(b4, {{1, 2}, {}, {}}, 300.0, 80.0, 40.0, 140.0);
    const auto dst_40 = gf::buildStaggeredTriangulation(b4, {{}, {}, {2, 1}}, 300.0, 80.0, 40.0, 140.0);
    const auto moves_40 = gf::planLabelledTransfer(src_40, dst_40);

    const auto src_100 = gf::buildStaggeredTriangulation(b4, {{1, 2}, {}, {}}, 300.0, 80.0, 100.0, 140.0);
    const auto dst_100 = gf::buildStaggeredTriangulation(b4, {{}, {}, {2, 1}}, 300.0, 80.0, 100.0, 140.0);
    const auto moves_100 = gf::planLabelledTransfer(src_100, dst_100);

    gf::RouteBPipelinedConfig berth_cfg;
    berth_cfg.serial_mode = false;
    berth_cfg.v_cruise_mps = 14.0;
    berth_cfg.accel_mps2 = 2.0;
    berth_cfg.d_pipe_m = 60.0;
    berth_cfg.d_clear_m = 32.0;
    berth_cfg.t_settle_s = 12.0;
    berth_cfg.uwb_d_r_m = 600.0;
    berth_cfg.uwb_sigma0_m = 1.0;
    berth_cfg.uwb_d_max_m = 1200.0;
    berth_cfg.link_seed = 1001;
    berth_cfg.range_measurement_scope = gf::RangeMeasurementScope::ActiveTopologyAndAuxiliary;
    berth_cfg.enable_auxiliary_handover_ranges = true;
    berth_cfg.record_scale_radius_series = true;
    berth_cfg.scale_series_interval_s = 0.5;

    const auto res_40 = gf::runClosedLoopReconfiguration(src_40, dst_40, moves_40, berth_cfg);
    const auto res_100 = gf::runClosedLoopReconfiguration(src_100, dst_100, moves_100, berth_cfg);

    json berth_40_json = reconfigResultToJson(res_40);
    berth_40_json["berth_height_m"] = 40.0;
    berth_40_json["worst_agent_envelope_coverage"] = worstAgentValue(res_40.audit.per_agent_envelope_coverage);

    json berth_100_json = reconfigResultToJson(res_100);
    berth_100_json["berth_height_m"] = 100.0;
    berth_100_json["worst_agent_envelope_coverage"] = worstAgentValue(res_100.audit.per_agent_envelope_coverage);

    payload["berth_height_contrast"] = {
        {"berth_40m", std::move(berth_40_json)},
        {"berth_100m", std::move(berth_100_json)},
        {"peak_scale_reduction_pct",
         res_40.audit.peak_position_scale_radius_m > 0.0
             ? 100.0 * (1.0 - res_100.audit.peak_position_scale_radius_m / res_40.audit.peak_position_scale_radius_m)
             : 0.0},
        {"rmse_reduction_pct",
         res_40.audit.position_rmse_m > 0.0
             ? 100.0 * (1.0 - res_100.audit.position_rmse_m / res_40.audit.position_rmse_m)
             : 0.0},
    };

    // (b) Two-reference collinear edge crossing vs circumcenter three-reference dual handover
    const auto p6 = gf::buildPinballTriangulation(3, 400.0, 100.0, 140.0);
    const auto dl3 = gf::buildStaggeredTriangulation(
        b4, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto p6_moves = gf::planLabelledTransfer(p6, dl3);

    gf::RouteBPipelinedConfig cfg_3ref;
    cfg_3ref.serial_mode = false;
    cfg_3ref.v_cruise_mps = 14.0;
    cfg_3ref.accel_mps2 = 2.0;
    cfg_3ref.d_pipe_m = 60.0;
    cfg_3ref.d_clear_m = 32.0;
    cfg_3ref.t_settle_s = 1.0;
    cfg_3ref.uwb_d_r_m = 600.0;
    cfg_3ref.uwb_sigma0_m = 1.0;
    cfg_3ref.uwb_d_max_m = 1200.0;
    cfg_3ref.link_seed = 1001;
    cfg_3ref.range_measurement_scope = gf::RangeMeasurementScope::ActiveTopologyAndAuxiliary;
    cfg_3ref.enable_auxiliary_handover_ranges = true;
    cfg_3ref.record_scale_radius_series = true;
    cfg_3ref.scale_series_interval_s = 1.0;

    auto cfg_2ref = cfg_3ref;
    cfg_2ref.enable_auxiliary_handover_ranges = false;
    cfg_2ref.strict_two_ref_collinear_baseline = true;
    cfg_2ref.max_sim_time_s = 900.0;

    const auto res_3ref = gf::runClosedLoopReconfiguration(p6, dl3, p6_moves, cfg_3ref);
    const auto res_2ref = gf::runClosedLoopReconfiguration(p6, dl3, p6_moves, cfg_2ref);

    json two_ref_json = reconfigResultToJson(res_2ref);
    two_ref_json["worst_agent_envelope_coverage"] = worstAgentValue(res_2ref.audit.per_agent_envelope_coverage);

    json three_ref_json = reconfigResultToJson(res_3ref);
    three_ref_json["worst_agent_envelope_coverage"] = worstAgentValue(res_3ref.audit.per_agent_envelope_coverage);

    payload["handover_reference_contrast"] = {
        {"two_ref_collinear_baseline", std::move(two_ref_json)},
        {"three_ref_dual_handover", std::move(three_ref_json)},
        {"peak_scale_reduction_pct",
         res_2ref.audit.peak_position_scale_radius_m > 0.0
             ? 100.0 * (1.0 - res_3ref.audit.peak_position_scale_radius_m / res_2ref.audit.peak_position_scale_radius_m)
             : 0.0},
        {"peak_trP_reduction_pct",
         res_2ref.audit.peak_position_covariance_trace_m2 > 0.0
             ? 100.0 * (1.0 - res_3ref.audit.peak_position_covariance_trace_m2 / res_2ref.audit.peak_position_covariance_trace_m2)
             : 0.0},
        {"rmse_reduction_pct",
         res_2ref.audit.position_rmse_m > 0.0
             ? 100.0 * (1.0 - res_3ref.audit.position_rmse_m / res_2ref.audit.position_rmse_m)
             : 0.0},
    };

    std::cout << "[Wave3 Contrast berth] 40m_peak=" << res_40.audit.peak_position_scale_radius_m
              << " 100m_peak=" << res_100.audit.peak_position_scale_radius_m
              << " | [handover] 2ref_peak=" << res_2ref.audit.peak_position_scale_radius_m
              << " 3ref_peak=" << res_3ref.audit.peak_position_scale_radius_m << std::endl;
    return payload;
}

json runTdd10NonuniformPentagon() {
    json payload;
    payload["provenance"] = provenanceHeader("route-b-wave3-tdd10-nonuniform", json(1001));
    const std::vector<double> b_xs = {0.0, 320.0, 720.0, 1080.0};
    payload["beacon_xs_m"] = b_xs;
    payload["beacon_spacings_m"] = json::array({320.0, 400.0, 360.0});
    payload["beacon_altitude_m"] = 100.0;
    payload["uwb_channel_group"] = "degraded";
    payload["uwb_d_r_m"] = 600.0;
    payload["uwb_sigma0_m"] = 1.0;
    payload["uwb_d_max_m"] = 1200.0;
    payload["seed"] = 1001;

    const auto tri_dual = gf::buildStaggeredTriangulation(
        b_xs, {{1, 2, 3}, {}, {4, 5, 6}}, 360.0, 68.0, 100.0, 140.0);
    const auto tri_center = gf::buildStaggeredTriangulation(
        b_xs, {{}, {6, 5, 4, 3, 2, 1}, {}}, 360.0, 68.0, 100.0, 140.0);

    const auto fwd_moves = gf::planLabelledTransfer(tri_dual, tri_center);
    const auto rev_moves = gf::planLabelledTransfer(tri_center, tri_dual);

    gf::RouteBPipelinedConfig pcfg;
    pcfg.serial_mode = false;
    pcfg.v_cruise_mps = 14.0;
    pcfg.accel_mps2 = 2.0;
    pcfg.d_pipe_m = 50.0;
    pcfg.d_clear_m = 32.0;
    pcfg.t_settle_s = 1.0;
    pcfg.uwb_d_r_m = 600.0;
    pcfg.uwb_sigma0_m = 1.0;
    pcfg.uwb_d_max_m = 1200.0;
    pcfg.link_seed = 1001;
    pcfg.record_trajectory_samples = true;
    pcfg.trajectory_sample_interval_s = 2.0;

    const auto fwd = gf::runClosedLoopReconfiguration(tri_dual, tri_center, fwd_moves, pcfg);
    const auto rev = gf::runClosedLoopReconfiguration(tri_center, tri_dual, rev_moves, pcfg);

    gf::RouteBConvexDomainSpec pent;
    pent.name = "nonuniform_oblique_pentagon";
    pent.description = "Irregular oblique asymmetric convex pentagon over non-uniform coastal beacons (320, 400, 360) m";
    pent.vertices = {
        {60.0, 220.0}, {880.0, 250.0}, {1140.0, 640.0}, {680.0, 980.0}, {180.0, 740.0}
    };
    pent.initial_tri = tri_dual;
    pent.target_tri = tri_center;
    pent.use_direct_matched = false;

    json verts = json::array();
    for (const auto& v : pent.vertices) {
        verts.push_back(json::array({v.x(), v.y()}));
    }

    gf::RouteBMissionConfig mcfg;
    mcfg.domain_name = pent.name;
    mcfg.custom_domain = pent;
    mcfg.cell_size_m = 10.0;
    mcfg.sensor_radius_m = 400.0;
    mcfg.t_fork_s = 12.0;
    mcfg.max_mission_s = 900.0;
    mcfg.patrol_radius_m = 105.0;
    mcfg.d_pipe_m = 50.0;
    mcfg.uwb_d_r_m = 600.0;
    mcfg.uwb_sigma0_m = 1.0;
    mcfg.uwb_d_max_m = 1200.0;
    mcfg.link_seed = 1001;
    mcfg.curve_sample_interval_s = 5.0;
    mcfg.record_trajectory_samples = true;
    mcfg.trajectory_sample_interval_s = 5.0;

    const auto mission = gf::runTwoRegimeMissionComparison(mcfg);
    std::cout << "[Wave3 Pentagon] fwd=" << fwd.makespan_s << "s rev=" << rev.makespan_s
              << "s switch_t100=" << mission.switch_arm.t100_certified_s.value_or(-1.0) << "s" << std::endl;

    payload["domain_name"] = pent.name;
    payload["vertices_m"] = std::move(verts);
    payload["reconfiguration"] = {
        {"dual_to_single_chain", reconfigResultToJson(fwd)},
        {"single_to_dual_chain", reconfigResultToJson(rev)},
    };
    payload["mission"] = {
        {"valid_cells", mission.valid_cells},
        {"area_m2", mission.area_m2},
        {"fork_time_s", mission.fork_time_s},
        {"checkpoint_byte_identical", mission.checkpoint_byte_identical},
        {"fork_truth_coverage", mission.fork_truth_coverage},
        {"fork_certified_coverage", mission.fork_certified_coverage},
        {"stay", missionArmToJson(mission.stay)},
        {"switch", missionArmToJson(mission.switch_arm)},
    };
    return payload;
}

void buildTdd7Setup(
    double d_r,
    double sig0,
    gf::RouteBConvexDomainSpec& out_dom,
    std::vector<double>& out_b_xs,
    json& out_verts,
    gf::RouteBMissionConfig& out_mcfg) {
    out_dom.name = "wide_broad_regime2";
    out_dom.description = "Broad convex coastal trapezoid exceeding critical area where parallel multi-unit expansion achieves net T95 and T100 breakeven";
    out_dom.vertices = {
        {-2050.0, 120.0},
        {4050.0, 120.0},
        {2950.0, 2050.0},
        {-950.0, 2050.0}
    };
    out_b_xs = {0.0, 400.0, 800.0, 1200.0, 1600.0, 2000.0};
    out_dom.initial_tri = gf::buildStaggeredTriangulation(
        out_b_xs, {{}, {}, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, {}, {}},
        380.0, 90.0, 100.0, 140.0);
    out_dom.target_tri = gf::buildStaggeredTriangulation(
        out_b_xs, {{1, 2, 3, 4, 5}, {}, {}, {}, {6, 7, 8, 9, 10}},
        380.0, 90.0, 100.0, 140.0);
    out_dom.use_direct_matched = true;

    out_verts = json::array();
    for (const auto& v : out_dom.vertices) {
        out_verts.push_back(json::array({v.x(), v.y()}));
    }

    out_mcfg = gf::RouteBMissionConfig{};
    out_mcfg.domain_name = out_dom.name;
    out_mcfg.custom_domain = out_dom;
    out_mcfg.cell_size_m = 10.0;
    out_mcfg.sensor_radius_m = 400.0;
    out_mcfg.dynamic_chain_sweep = true;
    out_mcfg.search_speed_cap_mps = 5.0;
    out_mcfg.patrol_radius_m = 2600.0;
    out_mcfg.v_cruise_mps = 23.5;
    out_mcfg.accel_mps2 = 3.5;
    out_mcfg.d_pipe_m = 45.0;
    out_mcfg.t_settle_s = 0.2;
    out_mcfg.t_fork_s = 10.0;
    out_mcfg.max_mission_s = 2800.0;
    out_mcfg.uwb_d_r_m = d_r;
    out_mcfg.uwb_sigma0_m = sig0;
    out_mcfg.uwb_d_max_m = 1200.0;
    out_mcfg.link_seed = 20261123;
    out_mcfg.curve_sample_interval_s = 10.0;
    out_mcfg.record_trajectory_samples = true;
    out_mcfg.trajectory_sample_interval_s = 20.0;
}

json snapshotBundleToJson(
    const gf::RouteBMissionComparison& res,
    const std::vector<double>& b_xs,
    const json& verts,
    const gf::RouteBMissionConfig& mcfg) {
    const auto& gm = res.grid_metadata;
    const auto& sw = res.switch_arm;
    auto rleToJson = [](const std::vector<std::pair<int, int>>& rle) {
        json arr = json::array();
        for (const auto& [st, len] : rle) {
            arr.push_back(json::array({st, len}));
        }
        return arr;
    };

    const double cell_dx = gm.x_num > 0 ? (gm.x_max_m - gm.x_min_m) / static_cast<double>(gm.x_num) : gm.cell_size_m;
    const double cell_dy = gm.y_num > 0 ? (gm.y_max_m - gm.y_min_m) / static_cast<double>(gm.y_num) : gm.cell_size_m;

    json snaps = json::array();
    json top_beacons = json::object();
    for (const auto& s : sw.mission_snapshots) {
        json pos = json::object();
        for (const auto& [id, p] : s.positions) {
            pos[std::to_string(id)] = json::array({p.x(), p.y()});
        }
        json bpos = json::object();
        for (const auto& [id, p] : s.beacons) {
            bpos[std::to_string(id)] = json::array({p.x(), p.y()});
        }
        if (top_beacons.empty()) top_beacons = bpos;

        json edges = json::array();
        for (const auto& [p, c] : s.active_edges) {
            edges.push_back(json::array({p, c}));
        }
        snaps.push_back({
            {"time_s", s.time_s},
            {"tick", s.tick},
            {"stage", s.stage},
            {"truth_coverage", s.truth_coverage},
            {"certified_coverage", s.certified_coverage},
            {"certified_count", s.certified_count},
            {"positions", std::move(pos)},
            {"beacons", std::move(bpos)},
            {"active_edges", std::move(edges)},
            {"certified_rle", rleToJson(s.certified_rle)},
        });
    }

    return {
        {"domain_name", res.domain_name},
        {"uwb_channel_group", "nominal"},
        {"uwb_d_r_m", mcfg.uwb_d_r_m},
        {"uwb_sigma0_m", mcfg.uwb_sigma0_m},
        {"uwb_d_max_m", mcfg.uwb_d_max_m},
        {"vertices_m", verts},
        {"beacon_xs_m", b_xs},
        {"beacons", top_beacons},
        {"grid_origin_m", json::array({gm.x_min_m, gm.y_min_m})},
        {"x_lim_m", json::array({gm.x_min_m, gm.x_max_m})},
        {"y_lim_m", json::array({gm.y_min_m, gm.y_max_m})},
        {"x_num", gm.x_num},
        {"y_num", gm.y_num},
        {"cell_size_m", gm.cell_size_m},
        {"cell_dx_m", cell_dx},
        {"cell_dy_m", cell_dy},
        {"linear_index_order", "ix * y_num + iy"},
        {"valid_cells", res.valid_cells},
        {"area_m2", res.area_m2},
        {"valid_cell_bbox", {
            {"ix_min", gm.bbox_ix_min},
            {"iy_min", gm.bbox_iy_min},
            {"ix_max", gm.bbox_ix_max},
            {"iy_max", gm.bbox_iy_max},
            {"x_min_m", gm.x_min_m + static_cast<double>(gm.bbox_ix_min) * cell_dx},
            {"y_min_m", gm.y_min_m + static_cast<double>(gm.bbox_iy_min) * cell_dy},
            {"x_max_m", gm.x_min_m + static_cast<double>(gm.bbox_ix_max + 1) * cell_dx},
            {"y_max_m", gm.y_min_m + static_cast<double>(gm.bbox_iy_max + 1) * cell_dy},
        }},
        {"valid_rle", rleToJson(gm.valid_rle)},
        {"switch_summary", {
            {"reconfig_completed", sw.reconfig_completed},
            {"status_reason", sw.status_reason},
            {"reconfig_entry_s", sw.reconfig_entry_s},
            {"reconfig_complete_s", sw.reconfig_complete_s},
            {"final_time_s", sw.final_time_s},
            {"reached_t95", sw.reached_t95},
            {"reached_t100", sw.reached_t100},
            {"t95_certified_s", sw.t95_certified_s.has_value() ? json(*sw.t95_certified_s) : json(nullptr)},
            {"t100_certified_s", sw.t100_certified_s.has_value() ? json(*sw.t100_certified_s) : json(nullptr)},
            {"final_truth_coverage", sw.final_truth_coverage},
            {"final_certified_coverage", sw.final_certified_coverage},
            {"false_certified_count", sw.false_certified_count},
            {"certified_subset_of_truth", sw.certified_subset_of_truth},
            {"static_parent_violations", sw.static_parent_violations},
            {"dag_violations", sw.dag_violations},
            {"total_mbb_swaps", sw.total_mbb_swaps},
            {"audit", auditToJson(sw.audit)},
        }},
        {"snapshots", std::move(snaps)},
    };
}

json runTdd7SingleArm(const std::string& arm_key) {
    std::string ch_name;
    double d_r = 850.0;
    double sig0 = 0.5;
    bool is_stay = false;
    bool with_snaps = false;

    if (arm_key == "nom_stay") {
        ch_name = "nominal"; d_r = 850.0; sig0 = 0.5; is_stay = true;
    } else if (arm_key == "nom_switch") {
        ch_name = "nominal"; d_r = 850.0; sig0 = 0.5; is_stay = false; with_snaps = true;
    } else if (arm_key == "deg_stay") {
        ch_name = "degraded"; d_r = 600.0; sig0 = 1.0; is_stay = true;
    } else if (arm_key == "deg_switch") {
        ch_name = "degraded"; d_r = 600.0; sig0 = 1.0; is_stay = false;
    } else {
        throw std::invalid_argument("unknown --arm value: " + arm_key);
    }

    gf::RouteBConvexDomainSpec dom;
    std::vector<double> b_xs;
    json verts;
    gf::RouteBMissionConfig mcfg;
    buildTdd7Setup(d_r, sig0, dom, b_xs, verts, mcfg);
    mcfg.skip_stay_arm = !is_stay;
    mcfg.skip_switch_arm = is_stay;
    if (with_snaps) {
        mcfg.record_mission_snapshots = true;
        mcfg.snapshot_times_s = {10.0, 160.0, 260.0, 380.0, 480.0, 950.0, 1250.0};
    }

    const auto res = gf::runTwoRegimeMissionComparison(mcfg);
    const auto& arm_res = is_stay ? res.stay : res.switch_arm;
    std::cout << "[Wave5 TDD-7 Arm " << arm_key << "] cells=" << res.valid_cells
              << " wall=" << arm_res.wall_clock_s << "s"
              << " reconf=" << arm_res.reconfig_complete_s
              << " t95=" << arm_res.t95_certified_s.value_or(-1.0)
              << " t100=" << arm_res.t100_certified_s.value_or(-1.0)
              << " snaps=" << arm_res.mission_snapshots.size() << std::endl;

    json out = {
        {"arm_key", arm_key},
        {"channel_group", ch_name},
        {"uwb_d_r_m", d_r},
        {"uwb_sigma0_m", sig0},
        {"domain_name", res.domain_name},
        {"vertices_m", verts},
        {"beacon_xs_m", b_xs},
        {"cell_size_m", mcfg.cell_size_m},
        {"sensor_radius_m", mcfg.sensor_radius_m},
        {"valid_cells", res.valid_cells},
        {"area_m2", res.area_m2},
        {"fork_time_s", res.fork_time_s},
        {"checkpoint_byte_identical", res.checkpoint_byte_identical},
        {"fork_truth_coverage", res.fork_truth_coverage},
        {"fork_certified_coverage", res.fork_certified_coverage},
        {"arm_result", missionArmToJson(arm_res)},
    };
    if (with_snaps) {
        out["snapshot_bundle"] = snapshotBundleToJson(res, b_xs, verts, mcfg);
    }
    return out;
}

void mergeTdd7Parts(const std::filesystem::path& out_dir) {
    auto readJson = [](const std::filesystem::path& p) {
        std::ifstream in(p);
        if (!in.is_open()) {
            throw std::runtime_error("missing TDD-7 partial file: " + p.string());
        }
        json j;
        in >> j;
        return j;
    };

    const std::filesystem::path p_ns = out_dir / "tdd7_part_nom_stay.json";
    const std::filesystem::path p_nw = out_dir / "tdd7_part_nom_switch.json";
    const std::filesystem::path p_ds = out_dir / "tdd7_part_deg_stay.json";
    const std::filesystem::path p_dw = out_dir / "tdd7_part_deg_switch.json";

    const json nom_stay = readJson(p_ns);
    const json nom_sw = readJson(p_nw);
    const json deg_stay = readJson(p_ds);
    const json deg_sw = readJson(p_dw);

    auto makeMissionRow = [](const json& stay_part, const json& sw_part) {
        const json& st_arm = stay_part.at("arm_result");
        const json& sw_arm = sw_part.at("arm_result");
        const json t95_gain = (!st_arm["t95_certified_s"].is_null() && !sw_arm["t95_certified_s"].is_null())
            ? json(st_arm["t95_certified_s"].get<double>() - sw_arm["t95_certified_s"].get<double>())
            : json(nullptr);
        const json t100_gain = (!st_arm["t100_certified_s"].is_null() && !sw_arm["t100_certified_s"].is_null())
            ? json(st_arm["t100_certified_s"].get<double>() - sw_arm["t100_certified_s"].get<double>())
            : json(nullptr);
        return json{
            {"domain_name", stay_part.at("domain_name")},
            {"regime_label", "regime_2_positive_breakeven"},
            {"channel_group", stay_part.at("channel_group")},
            {"uwb_d_r_m", stay_part.at("uwb_d_r_m")},
            {"uwb_sigma0_m", stay_part.at("uwb_sigma0_m")},
            {"vertices_m", stay_part.at("vertices_m")},
            {"beacon_xs_m", stay_part.at("beacon_xs_m")},
            {"cell_size_m", stay_part.at("cell_size_m")},
            {"sensor_radius_m", stay_part.at("sensor_radius_m")},
            {"valid_cells", stay_part.at("valid_cells")},
            {"area_m2", stay_part.at("area_m2")},
            {"fork_time_s", stay_part.at("fork_time_s")},
            {"checkpoint_byte_identical",
             stay_part.at("checkpoint_byte_identical").get<bool>() &&
                 sw_part.at("checkpoint_byte_identical").get<bool>()},
            {"fork_truth_coverage", stay_part.at("fork_truth_coverage")},
            {"fork_certified_coverage", stay_part.at("fork_certified_coverage")},
            {"net_t95_gain_s", t95_gain},
            {"net_t100_gain_s", t100_gain},
            {"stay", st_arm},
            {"switch", sw_arm},
        };
    };

    json breakeven;
    breakeven["provenance"] = provenanceHeader("route-b-wave5-tdd7-10m-breakeven", json(20261123));
    breakeven["domain_name"] = nom_stay.at("domain_name");
    breakeven["vertices_m"] = nom_stay.at("vertices_m");
    breakeven["beacon_xs_m"] = nom_stay.at("beacon_xs_m");
    breakeven["missions"] = json::array({
        makeMissionRow(nom_stay, nom_sw),
        makeMissionRow(deg_stay, deg_sw),
    });
    writeJsonFile(out_dir / "tdd7_regime2_positive_breakeven.json", breakeven);

    json snap_doc = nom_sw.at("snapshot_bundle");
    snap_doc["provenance"] = provenanceHeader("route-b-wave5-tdd11-10m-mission-snapshots", json(20261123));
    writeJsonFile(out_dir / "tdd7_mission_snapshots.json", snap_doc);

    for (const auto& p : {p_ns, p_nw, p_ds, p_dw}) {
        std::filesystem::remove(p);
    }
    std::cout << "[Wave5 TDD-7 Merge] wrote tdd7_regime2_positive_breakeven.json + tdd7_mission_snapshots.json"
              << std::endl;
}

json runTdd7Breakeven() {
    json payload;
    payload["provenance"] = provenanceHeader("route-b-wave5-tdd7-10m-breakeven", json(20261123));

    const std::vector<std::tuple<std::string, double, double>> uwb_groups = {
        {"nominal", 850.0, 0.5},
        {"degraded", 600.0, 1.0},
    };

    json missions = json::array();
    json verts_saved, b_xs_saved;
    for (const auto& [ch_name, d_r, sig0] : uwb_groups) {
        gf::RouteBConvexDomainSpec dom;
        std::vector<double> b_xs;
        json verts;
        gf::RouteBMissionConfig mcfg;
        buildTdd7Setup(d_r, sig0, dom, b_xs, verts, mcfg);
        verts_saved = verts;
        b_xs_saved = b_xs;

        const auto res = gf::runTwoRegimeMissionComparison(mcfg);
        const json t95_gain = (res.stay.t95_certified_s.has_value() && res.switch_arm.t95_certified_s.has_value())
            ? json(*res.stay.t95_certified_s - *res.switch_arm.t95_certified_s)
            : json(nullptr);
        const json t100_gain = (res.stay.t100_certified_s.has_value() && res.switch_arm.t100_certified_s.has_value())
            ? json(*res.stay.t100_certified_s - *res.switch_arm.t100_certified_s)
            : json(nullptr);

        std::cout << "[Wave5 Regime2 Breakeven " << ch_name << "] area_km2="
                  << (res.area_m2 / 1.0e6)
                  << " stay_t100=" << res.stay.t100_certified_s.value_or(-1.0)
                  << " sw_reconf=" << res.switch_arm.reconfig_complete_s
                  << " sw_t100=" << res.switch_arm.t100_certified_s.value_or(-1.0) << std::endl;

        missions.push_back({
            {"domain_name", res.domain_name},
            {"regime_label", "regime_2_positive_breakeven"},
            {"channel_group", ch_name},
            {"uwb_d_r_m", d_r},
            {"uwb_sigma0_m", sig0},
            {"vertices_m", verts},
            {"beacon_xs_m", b_xs},
            {"cell_size_m", mcfg.cell_size_m},
            {"sensor_radius_m", mcfg.sensor_radius_m},
            {"valid_cells", res.valid_cells},
            {"area_m2", res.area_m2},
            {"fork_time_s", res.fork_time_s},
            {"checkpoint_byte_identical", res.checkpoint_byte_identical},
            {"fork_truth_coverage", res.fork_truth_coverage},
            {"fork_certified_coverage", res.fork_certified_coverage},
            {"net_t95_gain_s", t95_gain},
            {"net_t100_gain_s", t100_gain},
            {"stay", missionArmToJson(res.stay)},
            {"switch", missionArmToJson(res.switch_arm)},
        });
    }
    payload["domain_name"] = "wide_broad_regime2";
    payload["vertices_m"] = verts_saved;
    payload["beacon_xs_m"] = b_xs_saved;
    payload["missions"] = std::move(missions);
    return payload;
}

json runTdd7Snapshots() {
    gf::RouteBConvexDomainSpec dom;
    std::vector<double> b_xs;
    json verts;
    gf::RouteBMissionConfig mcfg;
    buildTdd7Setup(850.0, 0.5, dom, b_xs, verts, mcfg);
    mcfg.skip_stay_arm = true;
    mcfg.record_mission_snapshots = true;
    mcfg.snapshot_times_s = {10.0, 160.0, 260.0, 380.0, 480.0, 950.0, 1250.0};

    const auto res = gf::runTwoRegimeMissionComparison(mcfg);
    const auto& sw = res.switch_arm;
    std::cout << "[Wave5 TDD-11 Snapshots] count=" << sw.mission_snapshots.size()
              << " reconf=" << sw.reconfig_complete_s
              << " t95=" << sw.t95_certified_s.value_or(-1.0)
              << " t100=" << sw.t100_certified_s.value_or(-1.0)
              << " swaps=" << sw.total_mbb_swaps
              << " false_cert=" << sw.false_certified_count << std::endl;

    json payload = snapshotBundleToJson(res, b_xs, verts, mcfg);
    payload["provenance"] = provenanceHeader("route-b-wave5-tdd11-10m-mission-snapshots", json(20261123));
    return payload;
}

json runWave9Validations() {
    json payload;
    payload["provenance"] = provenanceHeader("route-b-wave9-search-timing", json(20261123));

    // 1. Sub-critical coastal Regime 2 search mission (nominal & degraded)
    json sub_missions = json::array();
    json online_subcritical_decision;
    for (const std::string& ch : std::vector<std::string>{"nominal", "degraded"}) {
        const auto bundle = gf::runSubcriticalCoastalMission(ch, true);
        const auto& res = bundle.comparison;
        const auto& dec = bundle.online_decision;
        const auto& inp = bundle.decision_inputs;
        if (ch == "nominal") {
            online_subcritical_decision = {
                {"domain_name", res.domain_name},
                {"area_km2", res.area_m2 / 1.0e6},
                {"fork_time_s", res.fork_time_s},
                {"fork_certified_coverage", res.fork_certified_coverage},
                {"measured_r0_m2ps", inp.measured_r0_m2ps},
                {"predicted_r1_m2ps", inp.predicted_r1_m2ps},
                {"predicted_t_trans_s", inp.predicted_t_trans_s},
                {"predicted_a_trans_km2", inp.predicted_a_trans_m2 / 1.0e6},
                {"remaining_area_km2", dec.remaining_area_m2 / 1.0e6},
                {"a_crit_km2", dec.a_crit_m2 / 1.0e6},
                {"should_switch", dec.should_switch},
                {"decision_regime", "regime_2_subcritical_stay"},
            };
        }
        sub_missions.push_back({
            {"domain_name", res.domain_name},
            {"channel_group", ch},
            {"uwb_d_r_m", ch == "nominal" ? 850.0 : 600.0},
            {"uwb_sigma0_m", ch == "nominal" ? 0.5 : 1.0},
            {"uwb_d_max_m", 1200.0},
            {"vertices_m", [&]() {
                json v = json::array();
                for (const auto& pt : gf::buildSubcriticalCoastalRegime2Spec().vertices) {
                    v.push_back(json::array({pt.x(), pt.y()}));
                }
                return v;
            }()},
            {"beacon_xs_m", json::array({0.0, 400.0, 800.0, 1200.0, 1600.0, 2000.0})},
            {"valid_cells", res.valid_cells},
            {"area_m2", res.area_m2},
            {"fork_time_s", res.fork_time_s},
            {"checkpoint_byte_identical", res.checkpoint_byte_identical},
            {"fork_truth_coverage", res.fork_truth_coverage},
            {"fork_certified_coverage", res.fork_certified_coverage},
            {"stay", missionArmToJson(res.stay)},
            {"switch", missionArmToJson(res.switch_arm)},
        });
    }
    payload["subcritical_coastal_regime2"] = std::move(sub_missions);

    // 2. Online autonomous two-regime decisions across Regime 1, Sub-critical Regime 2, Broad Regime 2
    gf::OnlineTwoRegimeInputs reg1_in;
    reg1_in.uncertified_max_reach_m = 1550.0;
    reg1_in.current_formation_reach_m = gf::estimateCertifiedReachY(
        gf::findRouteBDomain("narrow_deep_offshore").initial_tri, 80.0, 400.0);
    reg1_in.target_formation_reach_m = gf::estimateCertifiedReachY(
        gf::findRouteBDomain("narrow_deep_offshore").target_tri, 80.0, 400.0);
    reg1_in.remaining_area_m2 = 0.44e6;
    reg1_in.measured_r0_m2ps = 1900.0;
    reg1_in.predicted_r1_m2ps = 1400.0;
    reg1_in.predicted_t_trans_s = 600.0;
    const auto reg1_dec = gf::evaluateOnlineTwoRegimeDecision(reg1_in);

    gf::OnlineTwoRegimeInputs reg2_broad_in;
    reg2_broad_in.uncertified_max_reach_m = 2050.0;
    reg2_broad_in.current_formation_reach_m = 3750.0;
    reg2_broad_in.target_formation_reach_m = 3390.0;
    reg2_broad_in.remaining_area_m2 = 0.878 * 9.650e6;
    reg2_broad_in.measured_r0_m2ps = 4035.0;
    reg2_broad_in.predicted_r1_m2ps = 8070.0;
    reg2_broad_in.predicted_t_trans_s = 603.8;
    reg2_broad_in.predicted_a_trans_m2 = (0.285 - 0.122) * 9.650e6;
    const auto reg2_broad_dec = gf::evaluateOnlineTwoRegimeDecision(reg2_broad_in);

    payload["online_autonomous_decisions"] = json::array({
        {
            {"domain_name", "narrow_deep_offshore"},
            {"area_km2", 0.494},
            {"current_reach_m", reg1_in.current_formation_reach_m},
            {"target_reach_m", reg1_in.target_formation_reach_m},
            {"uncertified_max_y_m", reg1_in.uncertified_max_reach_m},
            {"should_switch", reg1_dec.should_switch},
            {"decision_regime", "regime_1_reachability_switch"},
        },
        online_subcritical_decision,
        {
            {"domain_name", "wide_broad_regime2"},
            {"area_km2", 9.650},
            {"remaining_area_km2", reg2_broad_dec.remaining_area_m2 / 1.0e6},
            {"measured_r0_m2ps", reg2_broad_in.measured_r0_m2ps},
            {"predicted_r1_m2ps", reg2_broad_in.predicted_r1_m2ps},
            {"predicted_t_trans_s", reg2_broad_in.predicted_t_trans_s},
            {"predicted_a_trans_km2", reg2_broad_in.predicted_a_trans_m2 / 1.0e6},
            {"a_crit_km2", reg2_broad_dec.a_crit_m2 / 1.0e6},
            {"should_switch", reg2_broad_dec.should_switch},
            {"decision_regime", "regime_2_efficiency_switch"},
        },
    });

    // 3. Per-step real-time compute and communication telemetry across N = 6, 14, 20
    json timing = json::array();
    for (int scale_n : {6, 14, 20}) {
        const auto s = gf::measureRouteBComputeAndComm(scale_n);
        timing.push_back({
            {"scale_N", s.scale_n},
            {"ticks", s.ticks},
            {"qp_calls", s.qp_calls},
            {"qp_solve_mean_ms", s.qp_solve_mean_ms},
            {"qp_solve_p99_ms", s.qp_solve_p99_ms},
            {"qp_solve_max_ms", s.qp_solve_max_ms},
            {"qp_total_mean_ms", s.qp_total_mean_ms},
            {"qp_total_p99_ms", s.qp_total_p99_ms},
            {"estimator_mean_ms", s.estimator_mean_ms},
            {"estimator_p99_ms", s.estimator_p99_ms},
            {"tick_total_mean_ms", s.tick_total_mean_ms},
            {"tick_total_p99_ms", s.tick_total_p99_ms},
            {"mean_packets_per_tick", s.mean_packets_per_tick},
            {"mean_accepted_packets_per_tick", s.mean_accepted_packets_per_tick},
            {"mean_bandwidth_bytes_per_s", s.mean_bandwidth_bytes_per_s},
        });
    }
    payload["realtime_compute_and_comm"] = std::move(timing);

    // 4. Mild marine perturbation check (wind + multipath positive bias + decimeter beacon drift)
    const auto pert = gf::runMildMarinePerturbationCheck();
    payload["mild_marine_perturbation"] = {
        {"wind_disturbance_mps2", json::array({0.10, 0.06})},
        {"range_positive_bias_m", 0.20},
        {"beacon_pose_offset_norm_m", std::hypot(0.08, 0.06)},
        {"result", reconfigResultToJson(pert)},
        {"worst_agent_envelope_coverage", worstAgentValue(pert.audit.per_agent_envelope_coverage)},
    };
    return payload;
}

}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path out_dir = "./route_b_outputs";
    std::string mode = "all";
    std::string arm;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--output-dir" && i + 1 < argc) {
            out_dir = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (arg == "--arm" && i + 1 < argc) {
            arm = argv[++i];
        }
    }

    if (mode == "all" || mode == "tdd5_pinball") {
        writeJsonFile(out_dir / "tdd5_pinball_closed_loop.json", runTdd5PinballAndDl3());
    }
    if (mode == "all" || mode == "tdd5_multiscale") {
        writeJsonFile(out_dir / "tdd5_multiscale_closed_loop.json", runTdd5Multiscale());
    }
    if (mode == "all" || mode == "tdd6") {
        writeJsonFile(out_dir / "tdd6_two_regime_missions.json", runTdd6TwoRegime());
    }
    if (mode == "all" || mode == "wave3" || mode == "tdd7") {
        writeJsonFile(out_dir / "tdd7_regime2_positive_breakeven.json", runTdd7Breakeven());
    }
    if (mode == "all" || mode == "wave3" || mode == "tdd8") {
        writeJsonFile(out_dir / "tdd8_multiseed_monte_carlo.json", runTdd8Multiseed());
    }
    if (mode == "all" || mode == "wave3" || mode == "tdd9") {
        writeJsonFile(out_dir / "tdd9_geometry_closed_loop_contrast.json", runTdd9GeometryContrast());
    }
    if (mode == "all" || mode == "wave3" || mode == "tdd10") {
        writeJsonFile(out_dir / "tdd10_nonuniform_beacon_pentagon.json", runTdd10NonuniformPentagon());
    }
    if (mode == "tdd7_snapshots") {
        writeJsonFile(out_dir / "tdd7_mission_snapshots.json", runTdd7Snapshots());
    }
    if (mode == "tdd7_arm") {
        if (arm.empty()) {
            throw std::invalid_argument("--mode tdd7_arm requires --arm <nom_stay|nom_switch|deg_stay|deg_switch>");
        }
        writeJsonFile(out_dir / ("tdd7_part_" + arm + ".json"), runTdd7SingleArm(arm));
    }
    if (mode == "tdd7_merge") {
        mergeTdd7Parts(out_dir);
    }
    if (mode == "all" || mode == "wave9" || mode == "wave9_validations") {
        writeJsonFile(out_dir / "wave9_search_and_timing.json", runWave9Validations());
    }
    return 0;
}
