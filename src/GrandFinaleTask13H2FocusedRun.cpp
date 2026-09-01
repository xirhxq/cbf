#include "grand_finale/Task10p11rFixedBaseline.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <filesystem>
#include <iostream>

namespace {

using json=nlohmann::json;

gf::FrontierCell cell(int x,int y) {
    return {x,y,{10.0*x+5.0,10.0*y+5.0}};
}

Eigen::Vector2d servicePose(const gf::FrontierCell& value,double rho=350.0) {
    const Eigen::Vector2d radial=value.center-Eigen::Vector2d(1500.0,1500.0);
    return value.center-rho*radial/std::max(radial.norm(),rho);
}

struct FocusedCase {
    std::string id;
    gf::FrontierCell old_a;
    gf::FrontierCell old_b;
    gf::FrontierCell task;
    gf::NodeId forced_old_a_member=0;
    gf::NodeId forced_old_b_member=0;
    double yaw_deg=90.0;
    std::size_t maximum_ticks=600;
    bool require_service=true;
};

std::pair<gf::Task13UnifiedCoverageWitness,
          gf::Task13UnifiedCoverageWitness> pairFor(
    const FocusedCase& test,double service_standoff_m) {
    const auto squads=gf::task13UnifiedCoverageSquads();
    const auto fixed=gf::task10p11pStandardCoastalAnchors();
    gf::Task13UnifiedCoverageConfig config;
    config.certified_service_standoff_m=service_standoff_m;
    for (gf::NodeId ma:squads[0].members) {
        if (test.forced_old_a_member!=0&&ma!=test.forced_old_a_member)
            continue;
        const auto wa=gf::task13TaperedWitness(
            squads[0],ma,test.old_a,fixed,config);
        if (!wa.has_value()) continue;
        for (gf::NodeId mb:squads[1].members) {
            if (test.forced_old_b_member!=0&&mb!=test.forced_old_b_member)
                continue;
            const auto wb=gf::task13TaperedWitness(
                squads[1],mb,test.old_b,fixed,config);
            if (wb.has_value()&&gf::task13CrossMinimum(*wa,*wb)>10.0+1e-9)
                return {*wa,*wb};
        }
    }
    throw std::runtime_error("no compatible initial witness pair for "+
        test.old_a.id()+"/"+test.old_b.id());
}

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> fixtureFor(
    const FocusedCase& test,
    const gf::Task13UnifiedCoverageWitness& a,
    const gf::Task13UnifiedCoverageWitness& b,double tau_mps2,
    gf::GammaFeedbackSelectionMode selection,double service_standoff_m) {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    for (const auto& [member,target]:a.targets)
        scenario.mobile_positions.at(member-1)=target;
    for (const auto& [member,target]:b.targets)
        scenario.mobile_positions.at(member-1)=target;
    auto settings=gf::task10p11pSwarmSettings(
        scenario,gf::SolverProfile::Gurobi);
    settings["initial"]["yawDeg"]=test.yaw_deg;
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
        std::move(scenario),std::move(settings),
        selection,tau_mps2,
        true,false,false,false,false,false,false,false,false,false,false,
        false,true,false,true,false,false,false,true,service_standoff_m);
}

json runCase(const FocusedCase& test,double tau_mps2,
             gf::GammaFeedbackSelectionMode selection,
             double service_standoff_m) {
    const auto pair=pairFor(test,service_standoff_m);
    auto fixture=fixtureFor(test,pair.first,pair.second,tau_mps2,selection,
        service_standoff_m);
    const auto initialized=fixture->adapter.initializeStageZero();
    json result={{"id",test.id},{"initialized",initialized.initialized},
        {"old_a",test.old_a.id()},{"old_b",test.old_b.id()},
        {"old_a_member",pair.first.responsible_member},
        {"old_b_member",pair.second.responsible_member},
        {"task",test.task.id()},{"require_service",test.require_service},
        {"maximum_ticks",test.maximum_ticks}};
    if (!initialized.initialized) {
        result["passed"]=false;
        result["reason"]="stage_zero_initialization_failed";
        return result;
    }

    auto adapter_state=fixture->adapter.fixedRestartState();
    std::fill(adapter_state.coverage.truth.begin(),
              adapter_state.coverage.truth.end(),true);
    std::fill(adapter_state.coverage.certified.begin(),
              adapter_state.coverage.certified.end(),true);
    const std::size_t task_index=static_cast<std::size_t>(
        test.task.x_index*300+test.task.y_index);
    adapter_state.coverage.truth.at(task_index)=false;
    adapter_state.coverage.certified.at(task_index)=false;
    fixture->adapter.restoreFixedRestartState(adapter_state);

    gf::SimpleCoverageControllerRestartState controller_state;
    for (const auto& [member,target]:pair.first.targets)
        controller_state.targets[member]={test.old_a.x_index,
            test.old_a.y_index,target};
    for (const auto& [member,target]:pair.second.targets)
        controller_state.targets[member]={test.old_b.x_index,
            test.old_b.y_index,target};
    controller_state.target_epoch=1;
    fixture->controller.restoreRestartState(controller_state);

    double max_actual_reference=0.0,max_target_reference=0.0;
    double min_actual_separation=std::numeric_limits<double>::infinity();
    double min_target_separation=std::numeric_limits<double>::infinity();
    double min_robust_fim=std::numeric_limits<double>::infinity();
    double max_posterior=0.0,min_aoi=std::numeric_limits<double>::infinity();
    double min_gamma=std::numeric_limits<double>::infinity();
    double min_residual=std::numeric_limits<double>::infinity();
    double max_speed=0.0,max_axis_control=0.0;
    double max_raw_nominal=0.0;
    std::size_t advanced_ticks=0,target_events=0,interventions=0;
    double first_transition_maximum_displacement_m=0.0;
    std::string reason="deadline";
    bool hard_failure=false,serviced=false;
    json first_tick_controls=json::array();
    for (std::size_t tick=0;tick<test.maximum_ticks;++tick) {
        const auto step=fixture->controller.advance();
        target_events+=step.unified_allocation_evaluated;
        if (step.unified_allocation_evaluated&&
            step.unified_allocation.valid&&target_events==1)
            first_transition_maximum_displacement_m=
                step.unified_allocation.maximum_target_displacement_m;
        if (!step.step.advanced) {
            hard_failure=true;
            reason=step.reason.empty()?step.step.reason:step.reason;
            break;
        }
        ++advanced_ticks;
        min_residual=std::min(min_residual,step.step.minimum_hard_residual);
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const auto information=fixture->adapter.currentReferenceAudit();
        min_robust_fim=std::min(min_robust_fim,
            information.minimum_robust_fim_cone_lower_bound);
        max_posterior=std::max(max_posterior,
            information.maximum_posterior_eigenvalue);
        min_aoi=std::min(min_aoi,information.minimum_range_aoi_margin_s);
        std::map<gf::NodeId,Eigen::Vector2d> actual;
        for (const auto& robot:fixture->swarm.robots) {
            const gf::NodeId id=static_cast<gf::NodeId>(robot->id);
            actual[id]={robot->model->getStateVariable("x"),
                robot->model->getStateVariable("y")};
            max_speed=std::max(max_speed,
                robot->model->getVelocity().head<2>().norm());
        }
        const auto actualAt=[&](gf::NodeId id) {
            return actual.count(id)?actual.at(id):
                runtime.estimate.fixed_positions.at(id);
        };
        const auto targetAt=[&](gf::NodeId id) {
            return step.committed_targets.count(id)
                ?step.committed_targets.at(id).center:
                runtime.estimate.fixed_positions.at(id);
        };
        for (const auto& edge:runtime.topology) {
            max_actual_reference=std::max(max_actual_reference,
                (actualAt(edge.owner)-actualAt(edge.reference)).norm());
            max_target_reference=std::max(max_target_reference,
                (targetAt(edge.owner)-targetAt(edge.reference)).norm());
        }
        for (gf::NodeId first=1;first<=14;++first) {
            for (gf::NodeId second=first+1;second<=14;++second) {
                min_actual_separation=std::min(min_actual_separation,
                    (actualAt(first)-actualAt(second)).norm());
                min_target_separation=std::min(min_target_separation,
                    (targetAt(first)-targetAt(second)).norm());
            }
            for (const auto& [fixed_id,fixed_position]:
                 runtime.estimate.fixed_positions) {
                min_actual_separation=std::min(min_actual_separation,
                    (actualAt(first)-fixed_position).norm());
                min_target_separation=std::min(min_target_separation,
                    (targetAt(first)-fixed_position).norm());
                (void)fixed_id;
            }
        }
        for (const auto& [owner,diagnostic]:step.step.gamma_feedback) {
            (void)owner;
            min_gamma=std::min(min_gamma,diagnostic.current_gamma);
            interventions+=diagnostic.intervened;
        }
        for (const auto& [owner,control]:step.step.applied_controls) {
            (void)owner;
            max_axis_control=std::max(max_axis_control,
                control.cwiseAbs().maxCoeff());
        }
        for (const auto& [owner,control]:fixture->controller.lastNominalControls()) {
            (void)owner;
            max_raw_nominal=std::max(max_raw_nominal,
                control.cwiseAbs().maxCoeff());
        }
        if (tick==0) {
            for (const auto& [owner,raw]:fixture->controller.lastNominalControls()) {
                const auto diagnostic=step.step.gamma_feedback.at(owner);
                const auto applied=step.step.applied_controls.at(owner);
                const auto target=step.committed_targets.at(owner).center;
                first_tick_controls.push_back({{"owner",owner},
                    {"raw_nominal",{raw.x(),raw.y()}},
                    {"hard_projection",{
                        diagnostic.current_hard_projection.x(),
                        diagnostic.current_hard_projection.y()}},
                    {"selected_nominal",{
                        diagnostic.selected_nominal.x(),
                        diagnostic.selected_nominal.y()}},
                    {"applied",{applied.x(),applied.y()}},
                    {"current_gamma",diagnostic.current_gamma},
                    {"dominant_row",diagnostic.dominant_row},
                    {"target",{target.x(),target.y()}}});
            }
        }
        const GridWorld& grid=fixture->adapter.coverage().certifiedGrid();
        serviced=grid.vis.at(task_index);
        const bool safe=max_actual_reference<=850.0+1e-9&&
            max_target_reference<850.0-1e-9&&
            min_actual_separation>=10.0-1e-9&&
            min_target_separation>10.0+1e-9&&max_speed<=30.0+1e-9&&
            max_axis_control<=4.0+1e-9&&min_residual>=-1e-7;
        if (!safe) {
            hard_failure=true;
            reason="focused_hard_gate_violation";
            break;
        }
        if (serviced) {
            reason="certified_service";
            break;
        }
    }
    const bool passed=!hard_failure&&
        (test.require_service?serviced:advanced_ticks==test.maximum_ticks);
    double final_responsible_distance_m=
        std::numeric_limits<double>::infinity();
    gf::NodeId final_responsible_member=0;
    const Eigen::Vector2d task_service_pose=
        servicePose(test.task,service_standoff_m);
    for (const auto& [member,target]:fixture->controller.committedTargets())
        if (target.id()==test.task.id()&&
            (target.center-task_service_pose).norm()<=1e-8) {
            final_responsible_member=member;
            for (const auto& robot:fixture->swarm.robots)
                if (static_cast<gf::NodeId>(robot->id)==member)
                    final_responsible_distance_m=(Eigen::Vector2d(
                        robot->model->getStateVariable("x"),
                        robot->model->getStateVariable("y"))-
                        test.task.center).norm();
        }
    result.update({{"passed",passed},{"reason",reason},
        {"serviced",serviced},{"advanced_ticks",advanced_ticks},
        {"target_events",target_events},
        {"first_transition_maximum_displacement_m",
            first_transition_maximum_displacement_m},
        {"final_responsible_member",final_responsible_member},
        {"final_responsible_distance_m",final_responsible_distance_m},
        {"maximum_actual_reference_m",max_actual_reference},
        {"maximum_target_reference_m",max_target_reference},
        {"minimum_actual_separation_m",min_actual_separation},
        {"minimum_target_separation_m",min_target_separation},
        {"minimum_robust_fim",min_robust_fim},
        {"maximum_posterior_m2",max_posterior},
        {"minimum_aoi_margin_s",min_aoi},{"minimum_gamma",min_gamma},
        {"minimum_qp_residual",min_residual},
        {"maximum_speed_mps",max_speed},
        {"maximum_axis_control_mps2",max_axis_control},
        {"maximum_raw_nominal_axis_mps2",max_raw_nominal},
        {"intervention_owner_ticks",interventions},
        {"first_tick_controls",first_tick_controls},
        {"final_certified_count",
            fixture->adapter.coverage().certifiedCoveredCount()}});
    return result;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc<2||argc>6) {
        std::cerr<<"usage: GrandFinaleTask13H2FocusedRun OUTPUT_JSON "
            "[CASE] [TAU] [least|diagnostics] [SERVICE_STANDOFF_M]\n";
        return 2;
    }
    try {
        const double tau_mps2=argc>=4?std::stod(argv[3]):22.0;
        const std::string mode=argc==5?argv[4]:"least";
        const auto selection=mode=="diagnostics"
            ?gf::GammaFeedbackSelectionMode::DiagnosticsOnly
            :gf::GammaFeedbackSelectionMode::LeastIntervention;
        const double service_standoff_m=argc==6?std::stod(argv[5]):350.0;
        const std::vector<FocusedCase> cases={
            {"top_left_short_probe",cell(0,289),cell(36,222),
                cell(0,299),7,0,90.0,1,false},
            {"top_left_short_transition",cell(0,289),cell(36,222),
                cell(0,299),7,0,90.0,600,true},
            {"top_left_A_only_retained_blocker",cell(0,259),cell(36,222),
                cell(0,299),7,0,90.0,600,true},
            {"top_right_B_only",cell(263,222),cell(299,259),
                cell(299,299),0,14,90.0,600,true},
            {"top_middle_both",cell(149,259),cell(180,222),
                cell(149,299),0,0,90.0,600,true},
            {"base_near_reverse_approach",cell(150,90),cell(180,90),
                cell(150,50),0,0,-90.0,600,true},
            {"large_target_change_viability",cell(299,0),cell(0,0),
                cell(0,299),0,0,90.0,100,false}};
        json records=json::array();
        bool complete=true;
        for (const auto& test:cases) {
            if (argc>=3&&test.id!=argv[2]) continue;
            auto record=runCase(test,tau_mps2,selection,
                service_standoff_m);
            complete=complete&&record.at("passed").get<bool>();
            records.push_back(std::move(record));
        }
        const json output={{"protocol","task13-h2-focused-v1"},
            {"formula","H2 tapered lifting"},{"w",7.0},
            {"alpha",0.0075},{"tau_mps2",tau_mps2},
            {"gamma_selection",mode},
            {"service_standoff_m",service_standoff_m},
            {"rows","velocity_augmented"},{"complete",complete},
            {"case_filter",argc>=3?json(argv[2]):json(nullptr)},
            {"cases",records}};
        gf::writeTask10p11vJson(argv[1],output);
        std::cout<<output.dump(2)<<'\n';
        return complete?0:3;
    } catch (const std::exception& error) {
        std::cerr<<"H2 focused run failed: "<<error.what()<<'\n';
        return 4;
    }
}
