#include "grand_finale/Task11aDynamicTopologyCoordinator.hpp"
#include "grand_finale/PlantSpeedAppliedControl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>

namespace {

using json=nlohmann::json;

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeFixture(
    double tau,bool s1_on,const std::string& variant) {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(scenario,
        gf::SolverProfile::Gurobi);
    const bool s1_v4=variant=="s1_v4";
    const bool s1_v4_prime=variant=="s1_v4_prime";
    const bool s1_v4_bare=variant=="s1_v4_bare";
    const bool s1_rung_b_prime=variant=="s1_rung_b_prime";
    const bool s1_rung_b2=variant=="s1_rung_b2";
    if (!s1_on&&variant=="baseline") {
        return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau,s1_on,
            false,false,false,false,false,false);
    }
    if (s1_on&&s1_v4) {
        return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau,true,
            false,false,false,false,false,true,true);
    }
    if (s1_on&&s1_v4_prime) {
        return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau,false,
            false,false,false,true,false,false,true,true);
    }
    if (s1_on&&s1_v4_bare) {
        return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau,true,
            false,false,false,false,false,false,true,
            false,false,true);
    }
    if (s1_on&&s1_rung_b_prime) {
        // Rung B' (post-adjudication): single speed row @30, unsaturated
        // nominal, preflight demoted to the 31 m/s fuse, QP-side estimate
        // SpeedLimit initial-set precheck bypassed (truth-gate flag).
        // NOTE: speed_rows_removed must stay false - the single row IS the
        // experiment; only the 896-row facet block is dropped (via
        // plant_speed_facet_count=0 in the config builder).
        return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau,true,
            false,false,false,false,false,false,
            false,false,false,false,true);
    }
    if (s1_on&&s1_rung_b2) {
        // Rung B'' (2026-09-01): rung B' with speed_cbf_gain=7.0 - the
        // discretization speed asymptote drops to ~30.0076 at box-corner
        // |u|, inside the 30.01 truth gate.  Speed row stays at the full
        // 30 m/s limit; saturation stays off.
        return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau,true,
            false,false,false,false,false,false,
            false,false,false,false,true,true);
    }
    throw std::runtime_error("variant not implemented:"+variant);
}

}  // namespace

// Trajectory-level adjudication runner (researcher directive 2026-08-31):
// identical dynamics to GrandFinaleTask11bRun for the supported variants,
// plus a per-tick telemetry stream (applied controls, raw/post-wrapper
// nominals, estimate and truth states, gamma-feedback branch records,
// committed targets).  The final result.json keeps the Task11b schema so
// archived cells can be certified bit-identical by final coverage.
int main(int argc,char** argv) {
    if (argc!=8) {
        std::cerr<<"usage: GrandFinaleTask11bTelemetryRun TAU S1(on|off) "
            "OUTPUT_JSON PROGRESS_DIR WINDOW_S VARIANT TELEMETRY_JSONL\n";
        return 2;
    }
    const auto started=std::chrono::steady_clock::now();
    const auto elapsed=[&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
    };
    try {
        const double tau=std::stod(argv[1]);
        const bool s1_on=std::string(argv[2])=="on";
        const double window_s=std::stod(argv[5]);
        const std::string variant=argv[6];
        if (variant!="baseline"&&variant!="s1_v4"&&
            variant!="s1_v4_prime"&&variant!="s1_v4_bare"&&
            variant!="s1_rung_b_prime"&&variant!="s1_rung_b2") {
            std::cerr<<"variant "<<variant<<" not implemented\n";
            return 2;
        }
        const auto constants=gf::task11aFrozenConstants();
        auto fixture=makeFixture(tau,s1_on,variant);
        if (!fixture->adapter.initializeStageZero().initialized)
            throw std::runtime_error("stage-zero initialization failed");
        const auto request0=gf::task10p11zCaptureBeforeOverride(*fixture).
            request;
        const auto rows0=gf::buildCanonicalHardRows(request0);
        std::size_t speed_limit_rows=0,plant_facet_rows=0;
        for (const auto& row:rows0) {
            if (row.kind==gf::CanonicalHardRowKind::SpeedLimit)
                ++speed_limit_rows;
            if (row.kind==
                gf::CanonicalHardRowKind::PlantSpeedAppliedControl)
                ++plant_facet_rows;
        }
        json record={{"protocol","task11b-efficiency-run-v1"},
            {"preregistration",variant=="s1_rung_b2"?
                json("task-11b-rung-b2-2026-09-01"):
                json("task-11b-rung-b-prime-2026-08-31")},
            {"tau_mps2",tau},{"s1_speed_row_nominal",s1_on},
            {"variant",variant},
            {"margin_gate_threshold_mps2",json(nullptr)},
            {"window_s",window_s},
            {"identity_t0",{{"topology_matches_fixed_baseline",
                fixture->topologyFrozen()}}},
            {"row_counts",{{"speed_limit_rows",speed_limit_rows},
                {"plant_facet_rows",plant_facet_rows},
                {"total",rows0.size()}}},
            {"config_digest",{{"speed_row_nominal",
                fixture->adapter.config().speed_row_nominal},
                {"speed_row_nominal_limit_mps",
                fixture->adapter.config().speed_row_nominal_limit_mps},
                {"speed_limit_mps",fixture->adapter.config().speed_limit_mps},
                {"plant_speed_facet_count",
                fixture->adapter.config().plant_speed_facet_count},
                {"speed_rows_removed",
                fixture->adapter.config().speed_rows_removed},
                {"nominal_speed_saturation_mps",
                fixture->adapter.config().nominal_speed_saturation_mps},
                {"speed_preflight_demoted",
                fixture->adapter.config().speed_preflight_demoted},
                {"speed_preflight_fuse_mps",
                fixture->adapter.config().speed_preflight_fuse_mps},
                {"speed_initial_set_truth_gate",
                fixture->adapter.config().speed_initial_set_truth_gate},
                {"speed_cbf_gain",fixture->adapter.config().speed_cbf_gain},
                {"acceleration_half_box",
                fixture->adapter.config().acceleration_half_box},
                {"residual_tolerance",
                fixture->adapter.config().residual_tolerance},
                {"dt_s",fixture->adapter.config().dt_s},
                {"truth_initial_set_gate_mps",30.01}}},
            {"s3_stop_rule","terminate_at_t100_latch"},
            {"evaluations",json::array()},
            {"complete",false}};
        std::filesystem::create_directories(argv[4]);
        gf::writeTask10p11vJson(
            std::filesystem::path(argv[4])/"00-config.json",record);
        std::ofstream telemetry(argv[7]);
        if (!telemetry) throw std::runtime_error(
            "cannot open telemetry stream");
        double max_truth_overspeed=0.0,max_interval_overspeed=0.0;
        std::size_t truth_overspeed_ticks=0,interval_overspeed_ticks=0,
            interval_overspeed_005_ticks=0;
        double sum_truth_overspeed=0.0,sum_interval_overspeed=0.0;
        std::optional<std::size_t> t95_tick,t100_tick;
        std::size_t tick=0;
        bool hard_stop=false;
        bool truth_gate_violation=false;
        double truth_gate_max_speed=0.0;
        gf::SimpleCoverageControlStep last_step;
        const auto mobile_ids=
            fixture->adapter.runtimeSnapshot().estimate.mobile_ids;
        const auto robot_by_id=[&](gf::NodeId id)
            -> Robot& {
            for (const auto& robot:fixture->swarm.robots)
                if (static_cast<gf::NodeId>(robot->id)==id) return *robot;
            throw std::runtime_error("unknown robot id");
        };
        while (fixture->adapter.runtimeSnapshot().runtime_s<window_s) {
            // Rung B' truth initial-set gate: fail closed BEFORE advancing
            // when truth telemetry leaves the row's initial set (30.01 m/s
            // = the established 0.01 m/s realism threshold; boundary
            // numerics are ~1e-9).  Fuse at 31 m/s stays armed inside the
            // adapter as the last-resort layer.
            if (fixture->adapter.config().speed_initial_set_truth_gate) {
                truth_gate_max_speed=0.0;
                for (const auto& robot:fixture->swarm.robots) {
                    const Eigen::VectorXd tv=robot->model->getVelocity();
                    truth_gate_max_speed=std::max(truth_gate_max_speed,
                        tv.head<2>().norm());
                }
                if (truth_gate_max_speed>30.01) {
                    truth_gate_violation=true;
                    hard_stop=true;
                    record["evaluations"].push_back({{"tick",tick},
                        {"advanced",false},
                        {"reason","speed_initial_set_truth_violated"},
                        {"coverage_fraction",
                        fixture->adapter.coverage().truthFraction()},
                        {"minimum_hard_residual",json(nullptr)}});
                    break;
                }
            }
            // Interval-audit fix (B0-a code round): audit the PRE-advance
            // truth velocity (the retired form projected two ticks).
            std::map<gf::NodeId,Eigen::Vector2d> pre_velocities;
            for (const auto& robot:fixture->swarm.robots)
                pre_velocities[static_cast<gf::NodeId>(robot->id)]=
                    Eigen::Vector2d(robot->model->getVelocity().head<2>());
            last_step=fixture->controller.advance();
            const double fraction=
                fixture->adapter.coverage().truthFraction();
            if (!t95_tick.has_value()&&fraction>=0.95) t95_tick=tick;
            if (!t100_tick.has_value()&&fraction>=1.0-1e-12) t100_tick=tick;
            if (last_step.step.advanced) {
                double truth_max=0.0,interval_max=0.0;
                for (const auto& robot:fixture->swarm.robots) {
                    const Eigen::VectorXd truth_velocity=
                        robot->model->getVelocity();
                    const Eigen::Vector2d u=
                        last_step.step.applied_controls.at(
                            static_cast<gf::NodeId>(robot->id));
                    const double truth_speed=truth_velocity.head<2>().norm();
                    truth_max=std::max(truth_max,truth_speed-30.0);
                    const auto audit=gf::auditPlantSpeedExactZoh(
                        pre_velocities.at(static_cast<gf::NodeId>(
                            robot->id)),u,0.1,30.0,1.0e-9);
                    interval_max=std::max(interval_max,
                        audit.maximum_interval_speed_mps-30.0);
                }
                max_truth_overspeed=std::max(max_truth_overspeed,truth_max);
                max_interval_overspeed=std::max(max_interval_overspeed,
                    interval_max);
                if (truth_max>0.0) {
                    ++truth_overspeed_ticks;
                    sum_truth_overspeed+=truth_max;
                }
                if (interval_max>0.0) {
                    ++interval_overspeed_ticks;
                    sum_interval_overspeed+=interval_max;
                }
                if (interval_max>0.05) ++interval_overspeed_005_ticks;
            } else {
                hard_stop=true;
            }
            // Per-tick telemetry (raw nominal = pre-saturation/throttle;
            // selected nominal = post-wrapper value fed to the QP).
            // Defensive: on a non-advancing tick some maps are partial.
            const auto snapshot=fixture->adapter.runtimeSnapshot();
            const auto raw_nominals=
                fixture->controller.lastNominalControls();
            const auto throttle=fixture->adapter.lastThrottleTelemetry();
            json owners=json::array();
            for (gf::NodeId owner:mobile_ids) {
                const Eigen::Index off=4*static_cast<Eigen::Index>(
                    std::distance(mobile_ids.begin(),
                        std::find(mobile_ids.begin(),mobile_ids.end(),
                            owner)));
                const Eigen::Vector2d estimate_position=
                    snapshot.estimate.mean.segment<2>(off);
                const Eigen::Vector2d estimate_velocity=
                    snapshot.estimate.mean.segment<2>(off+2);
                json diagnostic_json=json(nullptr);
                const auto diagnostic=
                    last_step.step.gamma_feedback.find(owner);
                if (diagnostic!=last_step.step.gamma_feedback.end())
                    diagnostic_json={{"n_sel",{
                        diagnostic->second.selected_nominal.x(),
                        diagnostic->second.selected_nominal.y()}},
                        {"gamma",gf::task10p11w_detail::number(
                            diagnostic->second.current_gamma)},
                        {"intervened",diagnostic->second.intervened},
                        {"fallback",diagnostic->second.fallback_reason},
                        {"dominant_row",diagnostic->second.dominant_row}};
                json applied_json=json(nullptr);
                const auto applied=
                    last_step.step.applied_controls.find(owner);
                if (applied!=last_step.step.applied_controls.end())
                    applied_json={applied->second.x(),applied->second.y()};
                json raw_json=json(nullptr);
                const auto raw=raw_nominals.find(owner);
                if (raw!=raw_nominals.end())
                    raw_json={raw->second.x(),raw->second.y()};
                json target=json(nullptr);
                const auto committed=last_step.committed_targets.find(owner);
                if (committed!=last_step.committed_targets.end())
                    target={{"id",committed->second.id()},
                        {"center",{committed->second.center.x(),
                            committed->second.center.y()}}};
                const Robot& robot=robot_by_id(owner);
                owners.push_back({{"id",owner},
                    {"est",{estimate_position.x(),estimate_position.y(),
                        estimate_velocity.x(),estimate_velocity.y()}},
                    {"truth",{robot.model->getStateVariable("x"),
                        robot.model->getStateVariable("y"),
                        robot.model->getStateVariable("vx"),
                        robot.model->getStateVariable("vy")}},
                    {"u_applied",std::move(applied_json)},
                    {"n_raw",std::move(raw_json)},
                    {"n_sel_gamma",std::move(diagnostic_json)},
                    {"target",std::move(target)}});
            }
            telemetry<<json({{"tick",tick},
                {"runtime_s",snapshot.runtime_s},
                {"coverage_fraction",fraction},
                {"advanced",last_step.step.advanced},
                {"reason",last_step.step.reason},
                {"target_epoch",last_step.target_epoch},
                {"throttle",{{"active",throttle.active},
                    {"s",throttle.s}}},
                {"owners",std::move(owners)}}).dump()<<'\n';
            if (tick%100==0||hard_stop)
                record["evaluations"].push_back({{"tick",tick},
                    {"advanced",last_step.step.advanced},
                    {"reason",last_step.step.reason},
                    {"coverage_fraction",fraction},
                    {"minimum_hard_residual",gf::task10p11w_detail::number(
                        last_step.step.minimum_hard_residual)}});
            if (hard_stop) break;
            ++tick;
            if (t100_tick.has_value()) break;
        }
        telemetry.close();
        record["t95_tick"]=(t95_tick.has_value()?json(*t95_tick):
            json(nullptr));
        record["t100_tick"]=(t100_tick.has_value()?json(*t100_tick):
            json(nullptr));
        record["final_coverage_fraction"]=
            fixture->adapter.coverage().truthFraction();
        record["cycles"]=tick;
        record["hard_stop"]=hard_stop;
        record["failure"]={
            {"advanced",truth_gate_violation?false:last_step.step.advanced},
            {"reason",truth_gate_violation?
                json("speed_initial_set_truth_violated"):
                json(last_step.step.reason)},
            {"truth_gate_max_speed_mps",truth_gate_violation?
                json(truth_gate_max_speed):json(nullptr)},
            {"same_reason_as_fixed_132p4",!truth_gate_violation&&
                last_step.step.reason=="current_gamma_negative"}};
        record["throttle_telemetry_summary"]={
            {"activation_ticks",-1},
            {"last_active",fixture->adapter.lastThrottleTelemetry().active},
            {"last_s",fixture->adapter.lastThrottleTelemetry().s},
            {"last_min_gamma_mps2",gf::task10p11w_detail::number(
                fixture->adapter.lastThrottleTelemetry().min_gamma_mps2)},
            {"last_owner",fixture->adapter.lastThrottleTelemetry().owner}};
        record["realism"]={{"max_truth_overspeed_mps",
            gf::task10p11w_detail::number(max_truth_overspeed)},
            {"truth_overspeed_ticks",truth_overspeed_ticks},
            {"max_interval_overspeed_mps",
            gf::task10p11w_detail::number(max_interval_overspeed)},
            {"interval_overspeed_ticks",interval_overspeed_ticks},
            {"sum_truth_overspeed",gf::task10p11w_detail::number(
                sum_truth_overspeed)},
            {"sum_interval_overspeed",gf::task10p11w_detail::number(
                sum_interval_overspeed)},
            {"truth_overspeed_fail",max_truth_overspeed>0.01},
            {"interval_overspeed_005_ticks",interval_overspeed_005_ticks},
            {"interval_overspeed_fail",interval_overspeed_005_ticks>=5},
            {"verdict",(max_truth_overspeed>0.01||
                interval_overspeed_005_ticks>=5)?"S1_rejected":"S1_pass"},
            {"s1v4_fuse_limit_mps",variant=="s1_v4"?json(1.0):json(nullptr)},
            {"s1v4_fuse_tripped",variant=="s1_v4"&&
                max_interval_overspeed>1.0},
            {"s1v4_verdict",variant!="s1_v4"?json("n/a"):
                (max_interval_overspeed>1.0?json("fuse_tripped"):
                 json("s1v4_pass"))}};
        record["complete"]=true;
        record["wall_time_s"]=elapsed();
        record["telemetry_file"]=std::filesystem::path(argv[7]).filename().
            string();
        record["claim_boundary"]={{"efficiency_readout_development_only",
            true},{"zoh_guarantee_retired_to_monitoring",s1_on},
            {"t95_comparison_exploratory_only",true},
            {"claim_a_not_reopened",true}};
        gf::writeTask10p11vJson(argv[3],record);
        std::cout<<record.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 11b telemetry run failed: "<<error.what()<<'\n';
        return 4;
    }
}
