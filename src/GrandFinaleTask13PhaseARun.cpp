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

struct PhaseATemplate {
    std::string id;
    std::string description;
    std::vector<Eigen::Vector2d> positions;
    std::vector<gf::DirectedEdge> topology;
};

std::vector<Eigen::Vector2d> scaledPositions(double scale) {
    // Branch roots at nodes 1 (1380,10) and 8 (1620,10); spacing scaled
    // about each root along the standard launch geometry.
    const std::vector<Eigen::Vector2d> launch=
        gf::task10p11pStandardLaunchPositions();
    std::vector<Eigen::Vector2d> result;
    for (std::size_t index=0;index<launch.size();++index) {
        const gf::NodeId id=static_cast<gf::NodeId>(index+1);
        const Eigen::Vector2d root=
            (id<=7)?launch[0]:launch[7];
        result.push_back(root+scale*(launch[index]-root));
    }
    return result;
}

PhaseATemplate makeTemplate(const std::string& id) {
    const auto launch=gf::task10p11pStandardLaunchPositions();
    const auto origin=gf::task10p11rFixedReferenceTopology();
    if (id=="origin") {
        return {"origin","frozen ladder as-is (origin template)",
            launch,origin};
    }
    if (id=="stretch3") {
        return {"stretch3",
            "ladder graph unchanged, inter-node spacing x3 about branch "
            "roots (sweep baseline widened)",
            scaledPositions(3.0),origin};
    }
    if (id=="contract_half") {
        return {"contract_half",
            "ladder graph unchanged, inter-node spacing x0.5 about branch "
            "roots (formation tightened)",
            scaledPositions(0.5),origin};
    }
    if (id=="reanchor") {
        // Branch A re-rooted at anchor 102 (was 101); branch B re-rooted
        // at anchor pair 100+101 (was 101+102).  Ladder graphs unchanged.
        std::vector<gf::DirectedEdge> topology;
        for (const auto& edge:origin) {
            if (edge==gf::DirectedEdge(101,1)) topology.emplace_back(102,1);
            else if (edge==gf::DirectedEdge(101,2))
                topology.emplace_back(102,2);
            else if (edge==gf::DirectedEdge(101,8))
                topology.emplace_back(100,8);
            else topology.push_back(edge);
        }
        return {"reanchor",
            "ladder graphs unchanged, branch A re-rooted at anchor 102, "
            "branch B re-rooted at anchor pair 100+101 (anchor roles "
            "swapped between branches)",
            launch,topology};
    }
    if (id=="crosslink") {
        // Cross-branch interlink, acyclic: node 10 re-references {9,4}
        // (drops 8) and node 11 re-references {10,5} (drops 9); branch B's
        // upper half hangs off branch A's tail.  Indegree stays 2.
        std::vector<gf::DirectedEdge> topology;
        for (const auto& edge:origin) {
            if (edge==gf::DirectedEdge(8,10)) topology.emplace_back(4,10);
            else if (edge==gf::DirectedEdge(9,11))
                topology.emplace_back(5,11);
            else topology.push_back(edge);
        }
        return {"crosslink",
            "origin ladder with cross-branch interlink: node 10 "
            "re-references {9,4}, node 11 re-references {10,5} "
            "(acyclic, indegree preserved)",
            launch,topology};
    }
    if (id=="microfix") {
        // Causal micro-experiment (researcher-approved 2026-09-01):
        // origin graph with exactly two edge changes - owner 2
        // re-referenced 101->100 and owner 9 re-referenced 101->102 -
        // aligning each owner's anchor with its target side (the
        // bias-inversion finding: branch A targets 76-83% on the 100
        // side, branch B 85-92% on the 102 side, while owners 2/9 anchor
        // at center anchor 101 and historically hit the reference gate
        // there).
        std::vector<gf::DirectedEdge> topology;
        for (const auto& edge:origin) {
            if (edge==gf::DirectedEdge(101,2)) topology.emplace_back(100,2);
            else if (edge==gf::DirectedEdge(101,9))
                topology.emplace_back(102,9);
            else topology.push_back(edge);
        }
        return {"microfix",
            "origin graph + two edge changes (101->2 becomes 100->2, "
            "101->9 becomes 102->9); all else frozen",
            launch,topology};
    }
    throw std::runtime_error("unknown template:"+id);
}

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeFixture(
    const PhaseATemplate& def,double tau) {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.mobile_positions=def.positions;
    scenario.initial_topology=def.topology;
    auto settings=gf::task10p11pSwarmSettings(scenario,
        gf::SolverProfile::Gurobi);
    // Rung B'' speed domain: single nominal row @30, gain 7, no
    // saturation, preflight demoted (31 m/s fuse), truth initial-set gate.
    // Bool order: srnm, margin, family, analytic, demoted, throttle,
    // throttle_v2, rows_removed, rung_b, v4_prime, bare, rung_b_prime,
    // rung_b2.
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,tau,true,
        false,false,false,false,false,false,
        false,false,false,false,false,true);
}

}  // namespace

// Task 13 Phase A: frozen-formation sensitivity sweep.  One template per
// invocation, 500 s window with T100 latch, rung-B'' 14-row speed domain,
// per-tick telemetry and cumulative compute profile (profiler deliverable).
int main(int argc,char** argv) {
    if (argc!=5&&argc!=6&&argc!=7) {
        std::cerr<<"usage: GrandFinaleTask13PhaseARun TEMPLATE OUTPUT_JSON "
            "PROGRESS_DIR TELEMETRY_JSONL [TAU] [WINDOW_S]\n";
        return 2;
    }
    const auto started=std::chrono::steady_clock::now();
    const auto elapsed=[&]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now()-started).count();
    };
    try {
        const std::string template_id=argv[1];
        const double window_s=argc==7?std::stod(argv[6]):500.0;
        const double tau=argc>=6?std::stod(argv[5]):22.0;
        const auto def=makeTemplate(template_id);
        auto fixture=makeFixture(def,tau);
        if (!fixture->adapter.initializeStageZero().initialized) {
            // Qualification-gate boundary point: recorded, not run.
            json boundary={{"protocol","task13-phase-a-run-v1"},
                {"preregistration","task-13-phase-a-launch-2026-09-01"},
                {"template",template_id},{"description",def.description},
                {"tau_mps2",tau},{"qualified",false},
                {"boundary_reason","stage_zero_initialization_failed"},
                {"complete",true}};
            gf::writeTask10p11vJson(argv[2],boundary);
            std::cout<<boundary.dump(2)<<'\n';
            return 0;
        }
        const auto request0=gf::task10p11zCaptureBeforeOverride(*fixture).
            request;
        const auto rows0=gf::buildCanonicalHardRows(request0);
        json topology_json=json::array();
        for (const auto& edge:def.topology)
            topology_json.push_back(
                std::to_string(edge.reference)+"->"+
                std::to_string(edge.owner));
        json positions_json=json::array();
        for (std::size_t index=0;index<def.positions.size();++index)
            positions_json.push_back({index+1,
                def.positions[index].x(),def.positions[index].y()});
        std::size_t speed_limit_rows=0,plant_facet_rows=0;
        for (const auto& row:rows0) {
            if (row.kind==gf::CanonicalHardRowKind::SpeedLimit)
                ++speed_limit_rows;
            if (row.kind==
                gf::CanonicalHardRowKind::PlantSpeedAppliedControl)
                ++plant_facet_rows;
        }
        const auto& config=fixture->adapter.config();
        json record={{"protocol","task13-phase-a-run-v1"},
            {"preregistration","task-13-phase-a-launch-2026-09-01"},
            {"template",template_id},{"description",def.description},
            {"tau_mps2",tau},{"qualified",true},
            {"window_s",window_s},
            {"identity_t0",{{"topology_matches_template",
                fixture->topologyFrozen()}}},
            {"template_topology",std::move(topology_json)},
            {"template_positions",std::move(positions_json)},
            {"runtime_topology_t0",[&]() {
                json edges=json::array();
                for (const auto& edge:
                    fixture->adapter.runtimeSnapshot().topology)
                    edges.push_back(edge.id());
                return edges; }()},
            {"row_counts",{{"speed_limit_rows",speed_limit_rows},
                {"plant_facet_rows",plant_facet_rows},
                {"total",rows0.size()}}},
            {"config_digest",{{"speed_row_nominal",
                config.speed_row_nominal},
                {"speed_row_nominal_limit_mps",
                config.speed_row_nominal_limit_mps},
                {"speed_limit_mps",config.speed_limit_mps},
                {"plant_speed_facet_count",config.plant_speed_facet_count},
                {"speed_rows_removed",config.speed_rows_removed},
                {"nominal_speed_saturation_mps",
                config.nominal_speed_saturation_mps},
                {"speed_preflight_demoted",config.speed_preflight_demoted},
                {"speed_preflight_fuse_mps",
                config.speed_preflight_fuse_mps},
                {"speed_initial_set_truth_gate",
                config.speed_initial_set_truth_gate},
                {"speed_cbf_gain",config.speed_cbf_gain},
                {"acceleration_half_box",config.acceleration_half_box},
                {"residual_tolerance",config.residual_tolerance},
                {"dt_s",config.dt_s},
                {"truth_initial_set_gate_mps",30.01}}},
            {"s3_stop_rule","terminate_at_t100_latch"},
            {"evaluations",json::array()},
            {"complete",false}};
        std::filesystem::create_directories(argv[3]);
        gf::writeTask10p11vJson(
            std::filesystem::path(argv[3])/"00-config.json",record);
        std::ofstream telemetry(argv[4]);
        if (!telemetry) throw std::runtime_error(
            "cannot open telemetry stream");
        double max_truth_overspeed=0.0,max_interval_overspeed=0.0;
        std::size_t truth_overspeed_ticks=0,interval_overspeed_ticks=0;
        std::optional<std::size_t> t95_tick,t100_tick;
        std::size_t tick=0;
        bool hard_stop=false;
        bool truth_gate_violation=false;
        double truth_gate_max_speed=0.0;
        gf::SimpleCoverageControlStep last_step;
        gf::Task10p11ComputeProfile profiler;
        const auto mobile_ids=
            fixture->adapter.runtimeSnapshot().estimate.mobile_ids;
        const auto robot_by_id=[&](gf::NodeId id)->Robot& {
            for (const auto& robot:fixture->swarm.robots)
                if (static_cast<gf::NodeId>(robot->id)==id) return *robot;
            throw std::runtime_error("unknown robot id");
        };
        while (fixture->adapter.runtimeSnapshot().runtime_s<window_s) {
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
            last_step=fixture->controller.advance();
            profiler.merge(last_step.compute_profile);
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
                        truth_velocity.head<2>(),u,0.1,30.0,1.0e-9);
                    interval_max=std::max(interval_max,
                        audit.maximum_interval_speed_mps-30.0);
                }
                max_truth_overspeed=std::max(max_truth_overspeed,truth_max);
                max_interval_overspeed=std::max(max_interval_overspeed,
                    interval_max);
                if (truth_max>0.0) ++truth_overspeed_ticks;
                if (interval_max>0.0) ++interval_overspeed_ticks;
            } else {
                hard_stop=true;
            }
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
        json profiler_json=json::array();
        static const std::vector<gf::Task10p11ComputePhase> phases={
            gf::Task10p11ComputePhase::CandidateBundleConstruction,
            gf::Task10p11ComputePhase::EstimatorPropagation,
            gf::Task10p11ComputePhase::CanonicalRowRebuild,
            gf::Task10p11ComputePhase::ExactHardProjection,
            gf::Task10p11ComputePhase::CurrentGamma,
            gf::Task10p11ComputePhase::PredictedGamma,
            gf::Task10p11ComputePhase::RobustQpSetup,
            gf::Task10p11ComputePhase::RobustQpSolve,
            gf::Task10p11ComputePhase::ResidualTokenAudit,
            gf::Task10p11ComputePhase::SolverInitialization,
            gf::Task10p11ComputePhase::SolverModelUpdate,
            gf::Task10p11ComputePhase::GridWorldTarget,
            gf::Task10p11ComputePhase::CurrentCanonicalRowRebuild,
            gf::Task10p11ComputePhase::PredictedCanonicalRowRebuild,
            gf::Task10p11ComputePhase::FinalQp,
            gf::Task10p11ComputePhase::InformationAudit,
            gf::Task10p11ComputePhase::TruthOnlyAudit,
            gf::Task10p11ComputePhase::OnlineEstimator,
            gf::Task10p11ComputePhase::PlantPreflightZoh};
        double profiled_total=0.0;
        for (const auto phase:phases) {
            const auto summary=profiler.summary(phase);
            if (summary.calls==0) continue;
            profiled_total+=summary.total_s;
            profiler_json.push_back({{"phase",
                gf::task10p11ComputePhaseName(phase)},
                {"calls",summary.calls},
                {"total_s",gf::task10p11w_detail::number(summary.total_s)},
                {"median_s",gf::task10p11w_detail::number(
                    summary.median_s)},
                {"p95_s",gf::task10p11w_detail::number(summary.p95_s)},
                {"maximum_s",gf::task10p11w_detail::number(
                    summary.maximum_s)}});
        }
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
        record["realism"]={{"max_truth_overspeed_mps",
            gf::task10p11w_detail::number(max_truth_overspeed)},
            {"truth_overspeed_ticks",truth_overspeed_ticks},
            {"max_interval_overspeed_mps",
            gf::task10p11w_detail::number(max_interval_overspeed)},
            {"interval_overspeed_ticks",interval_overspeed_ticks},
            {"fuse_limit_mps",config.speed_preflight_fuse_mps},
            {"fuse_tripped",max_interval_overspeed>1.0}};
        record["profiler"]={{"note",
            "cumulative over run; runner interval telemetry is two-tick "
            "defective (known defect), adapter preflight authoritative"},
            {"profiled_total_s",gf::task10p11w_detail::number(
                profiled_total)},
            {"phases",std::move(profiler_json)}};
        record["complete"]=true;
        record["wall_time_s"]=elapsed();
        gf::writeTask10p11vJson(argv[2],record);
        std::cout<<record.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 13 Phase A run failed: "<<error.what()<<'\n';
        return 4;
    }
}
