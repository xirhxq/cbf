#include "grand_finale/Task11aDynamicTopologyCoordinator.hpp"
#include "grand_finale/PlantSpeedAppliedControl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>

namespace {

using json=nlohmann::json;

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeFixture(
    double tau,bool s1_on,const std::string& variant) {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(scenario,
        gf::SolverProfile::Gurobi);
    const bool margin_gate=variant=="margin_gate";
    const bool family_predict=variant=="family_predict";
    const bool analytic=variant=="analytic_first_order";
    if (!s1_on&&(variant=="baseline"||margin_gate||family_predict||
            analytic)) {
        return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau,s1_on,
            margin_gate,family_predict,analytic);
    }
    if (s1_on&&variant=="s1_v3") {
        return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
            std::move(scenario),std::move(settings),
            gf::GammaFeedbackSelectionMode::LeastIntervention,tau,true,
            false,false,false,true);
    }
    throw std::runtime_error("variant not implemented:"+variant);
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=6&&argc!=7) {
        std::cerr<<"usage: GrandFinaleTask11bRun TAU S1(on|off) OUTPUT_JSON "
            "PROGRESS_DIR WINDOW_S [VARIANT]\n";
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
        const std::string variant=argc==7?argv[6]:"baseline";
        if (variant!="baseline"&&variant!="margin_gate"&&
            variant!="family_predict"&&variant!="analytic_first_order") {
            std::cerr<<"variant "<<variant<<" not implemented\n";
            return 2;
        }
        const auto constants=gf::task11aFrozenConstants();
        auto fixture=makeFixture(tau,s1_on,variant);
        if (!fixture->adapter.initializeStageZero().initialized)
            throw std::runtime_error("stage-zero initialization failed");
        // S1 evidence: row-kind counts at the live request.
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
            {"preregistration","task-11b-approved-v1.1-2026-09-01"},
            {"tau_mps2",tau},{"s1_speed_row_nominal",s1_on},
            {"variant",variant},
            {"margin_gate_threshold_mps2",variant=="margin_gate"?json(1.0):json(nullptr)},
            {"window_s",window_s},
            {"identity_t0",{{"topology_matches_fixed_baseline",
                fixture->topologyFrozen()}}},
            {"row_counts",{{"speed_limit_rows",speed_limit_rows},
                {"plant_facet_rows",plant_facet_rows},
                {"total",rows0.size()}}},
            {"s3_stop_rule","terminate_at_t100_latch"},
            {"evaluations",json::array()},
            {"complete",false}};
        auto emit=[&](const std::string& name,const json& payload) {
            gf::writeTask10p11vJson(
                std::filesystem::path(argv[4])/name,payload);
        };
        std::filesystem::create_directories(argv[4]);
        emit("00-config.json",record);
        // Realism telemetry (prereg v1.1 section 11.4).
        double max_truth_overspeed=0.0,max_interval_overspeed=0.0;
        std::size_t truth_overspeed_ticks=0,interval_overspeed_ticks=0,
            interval_overspeed_005_ticks=0;
        double sum_truth_overspeed=0.0,sum_interval_overspeed=0.0;
        std::optional<std::size_t> t95_tick,t100_tick;
        std::size_t tick=0;
        bool hard_stop=false;
        gf::SimpleCoverageControlStep last_step;
        while (fixture->adapter.runtimeSnapshot().runtime_s<window_s) {
            last_step=fixture->controller.advance();
            const double fraction=
                fixture->adapter.coverage().truthFraction();
            if (!t95_tick.has_value()&&fraction>=0.95) t95_tick=tick;
            if (!t100_tick.has_value()&&fraction>=1.0-1e-12) t100_tick=tick;
            if (last_step.step.advanced) {
                double truth_max=0.0,interval_max=0.0;
                const auto& estimate=fixture->adapter.runtimeSnapshot().
                    estimate;
                for (const auto& robot:fixture->swarm.robots) {
                    const Eigen::VectorXd truth_velocity=
                        robot->model->getVelocity();
                    const Eigen::Vector2d u=
                        last_step.step.applied_controls.at(robot->id);
                    const double truth_speed=truth_velocity.head<2>().norm();
                    truth_max=std::max(truth_max,truth_speed-30.0);
                    const auto audit=gf::auditPlantSpeedExactZoh(
                        truth_velocity.head<2>(),u,0.1,30.0,1.0e-9);
                    interval_max=std::max(interval_max,
                        audit.maximum_interval_speed_mps-30.0);
                    (void)estimate;
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
            if (tick%100==0||hard_stop)
                record["evaluations"].push_back({{"tick",tick},
                    {"advanced",last_step.step.advanced},
                    {"reason",last_step.step.reason},
                    {"coverage_fraction",fraction},
                    {"minimum_hard_residual",gf::task10p11w_detail::number(
                        last_step.step.minimum_hard_residual)}});
            if (hard_stop) break;
            ++tick;
            // S3: terminate at the T100 latch.
            if (t100_tick.has_value()) break;
        }
        record["t95_tick"]=(t95_tick.has_value()?json(*t95_tick):
            json(nullptr));
        record["t100_tick"]=(t100_tick.has_value()?json(*t100_tick):
            json(nullptr));
        record["final_coverage_fraction"]=
            fixture->adapter.coverage().truthFraction();
        record["cycles"]=tick;
        record["hard_stop"]=hard_stop;
        record["failure"]={{"advanced",last_step.step.advanced},
            {"reason",last_step.step.reason},
            {"same_reason_as_fixed_132p4",last_step.step.reason==
                "current_gamma_negative"}};
        // Prereg v1.1 section 11.4 quantified realism verdicts.
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
                interval_overspeed_005_ticks>=5)?"S1_rejected":"S1_pass"}};
        record["complete"]=true;
        record["wall_time_s"]=elapsed();
        record["claim_boundary"]={{"efficiency_readout_development_only",
            true},{"zoh_guarantee_retired_to_monitoring",s1_on},
            {"t95_comparison_exploratory_only",true},
            {"claim_a_not_reopened",true}};
        gf::writeTask10p11vJson(argv[3],record);
        std::cout<<record.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 11b run failed: "<<error.what()<<'\n';
        return 4;
    }
}
