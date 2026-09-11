#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/Task32UnitFrontLedger.hpp"
#include <fstream>

using json=nlohmann::json;
namespace {
struct FixtureRun {
    json initial;
    std::vector<json> scientific_ticks;
    std::vector<Eigen::MatrixXd> covariance;
};
json point(const Eigen::Vector2d& p) {return {p.x(),p.y()};}
json matrix(const Eigen::MatrixXd& a) {
    auto out=json::array();
    for(Eigen::Index i=0;i<a.rows();++i) {
        auto row=json::array();for(Eigen::Index j=0;j<a.cols();++j)row.push_back(a(i,j));out.push_back(row);
    }
    return out;
}

FixtureRun runFixture(int mechanism,bool explicit_zero,int velocity_variant) {
    std::cout<<"TARGET_FIXTURE_START "<<json({{"mechanism",mechanism},{"explicit_zero",explicit_zero},
        {"velocity_variant",velocity_variant},{"requested_ticks",100}}).dump()<<'\n'<<std::flush;
    std::ifstream file("docs/evidence/task31-triangular-common-bridge/anchor-scene-terminal-port-r1.json");
    REQUIRE(file.good());json asset;file>>asset;
    const auto scene=gf::task31AnchorScene(asset);REQUIRE(scene.valid);
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.width_m=4500;scenario.height_m=2250;scenario.fixed_positions=scene.fixed;
    for(auto& p:scenario.mobile_positions)p.x()+=750;
    auto config=gf::task19ProductionAdapterConfig();
    config.target_policy_task18_cbf2026_outer=false;config.target_policy_task20_dag_lattice=true;
    config.task18_yaw_objective=gf::Task18YawObjective::SourceCvtSoftCbfSecondOrder;
    config.range_noise_std_m=.5;config.range_dropout_probability=0;config.range_random_seed=137029;
    config.distance_range_availability={true,138029};
    if(mechanism!=0||explicit_zero)config.task32_target_mechanism=mechanism;
    REQUIRE(config.task32_front_rate_mps==29.9);
    REQUIRE(config.acceleration_half_box==4);REQUIRE(config.predictive_gamma_tau_mps2==14);
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
    if(velocity_variant) {
        settings["initial"]["velocity"]["values"]=json::array();
        for(auto id:scenario.mobile_ids)settings["initial"]["velocity"]["values"].push_back({std::cos(id),std::sin(id)});
    }
    gf::Task10p11rFixedBaselineFixture fixture(scenario,settings,config);
    REQUIRE(fixture.adapter.initializeStageZero().initialized);
    std::cout<<"TARGET_FIXTURE_INITIALIZED "<<json({{"mechanism",mechanism},{"explicit_zero",explicit_zero},
        {"velocity_variant",velocity_variant}}).dump()<<'\n'<<std::flush;
    gf::Task26ExternalReconstructor observer(fixture.adapter,fixture.controller,
        "pinball-qualified-layered-centeredframe-moving-linearphase-rolecenter-continuingfront",
        60,1,false,true,false,scene,true,0);
    FixtureRun output;
    const auto initial=fixture.adapter.runtimeSnapshot();
    REQUIRE(initial.estimate.mean.size()==56);REQUIRE(initial.estimate.covariance.rows()==56);
    CHECK(initial.estimate.mean.allFinite());CHECK(initial.estimate.covariance.allFinite());
    for(const auto& robot:fixture.swarm.robots) {
        const Eigen::Vector2d expected=velocity_variant?
            Eigen::Vector2d(std::cos(robot->id),std::sin(robot->id)):Eigen::Vector2d::Zero();
        CHECK(robot->model->getVelocity().head<2>()==expected);
    }
    const auto initial_information=gf::task31InformationTelemetry(fixture.adapter);
    output.initial={{"mean",matrix(initial.estimate.mean)},{"covariance",matrix(initial.estimate.covariance)},
        {"information",initial_information},{"settings",settings}};
    std::cout<<"TARGET_FIXTURE_INITIAL "<<json({{"mechanism",mechanism},{"explicit_zero",explicit_zero},
        {"velocity_variant",velocity_variant},{"initial",output.initial},
        {"scope","constructed 14-owner initialization, not a historical checkpoint"}}).dump()<<'\n';
    std::map<std::string,gf::Task32UnitFrontLedger> previous_ledger;
    const auto contract=gf::task25DagContractFromCode(0);
    double minimum_fim=std::numeric_limits<double>::infinity(),maximum_posterior=0,maximum_front_step=0;
    for(int tick=0;tick<100;++tick) {
        const auto before=fixture.adapter.runtimeSnapshot();
        const auto pre_visited=fixture.adapter.coverage().certifiedGrid().vis;
        std::map<gf::NodeId,double> yaw;
        std::map<gf::NodeId,Eigen::Vector2d> velocity;
        for(const auto& robot:fixture.swarm.robots) {
            yaw[robot->id]=robot->model->getStateVariable("yawRad");
            velocity[robot->id]=robot->model->getVelocity().head<2>();
        }
        observer.beforeStep();const auto step=fixture.controller.advance();
        INFO("mechanism="<<mechanism<<" explicit_zero="<<explicit_zero<<" variant="<<velocity_variant<<" tick="<<tick<<" reason="<<step.step.reason);
        REQUIRE(step.step.advanced);CHECK(step.step.minimum_hard_residual>=-1e-9);
        std::cout<<"TARGET_FIXTURE_ADVANCED "<<json({{"mechanism",mechanism},{"explicit_zero",explicit_zero},
            {"velocity_variant",velocity_variant},{"tick",tick}}).dump()<<'\n';
        const auto after=fixture.adapter.runtimeSnapshot();
        CHECK(after.runtime_s==doctest::Approx(before.runtime_s+.1).epsilon(1e-12));
        CHECK(step.step.estimator_version_before==before.estimator_token);
        CHECK(step.step.estimator_version_after==after.estimator_token);
        CHECK(after.topology==scenario.initial_topology);CHECK(after.estimate.fixed_positions==scene.fixed);
        REQUIRE(step.applied_target_centers.size()==14);REQUIRE(step.committed_targets.size()==14);
        auto physical_positions=scene.fixed;
        auto owners=json::array();
        for(std::size_t k=0;k<scenario.mobile_ids.size();++k) {
            const auto id=scenario.mobile_ids[k];const auto u=step.step.applied_controls.at(id);
            const auto cell=step.committed_targets.at(id);
            CHECK(cell.x_index>=0);CHECK(cell.x_index<450);CHECK(cell.y_index>=0);CHECK(cell.y_index<225);
            CHECK(u.cwiseAbs().maxCoeff()<=4+1e-6);
            CHECK(gf::auditPlantSpeedExactZoh(velocity.at(id),u,.1,30,1e-9).maximum_interval_speed_mps<=30+1e-9);
            const Eigen::Vector4d estimate=before.estimate.mean.segment<4>(4*k);
            const auto expected_yaw=gf::task32SourceCvtYaw(estimate.head<2>(),estimate.tail<2>(),yaw.at(id),step.applied_target_centers.at(id),1.);
            REQUIRE(expected_yaw.valid);CHECK(step.step.applied_yaw_rates_radps.at(id)==doctest::Approx(expected_yaw.rate).epsilon(1e-10));
            const Robot* robot=nullptr;for(const auto& r:fixture.swarm.robots)if(r->id==static_cast<int>(id))robot=r.get();REQUIRE(robot);
            physical_positions[id]={robot->model->getStateVariable("x"),robot->model->getStateVariable("y")};
            CHECK(std::abs(gf::wrapYawRad(robot->model->getStateVariable("yawRad")-yaw.at(id)-.1*expected_yaw.rate))<1e-12);
            const Eigen::Vector4d post=after.estimate.mean.segment<4>(4*k);
            owners.push_back({{"id",id},{"est",{post(0),post(1),post(2),post(3)}},
                {"truth",{physical_positions.at(id).x(),physical_positions.at(id).y(),robot->model->getStateVariable("vx"),robot->model->getStateVariable("vy")}},
                {"u_applied",point(u)},{"n_raw",point(fixture.controller.lastNominalControls().at(id))},
                {"target_id",cell.id()},{"applied_center",point(step.applied_target_centers.at(id))},
                {"yaw",robot->model->getStateVariable("yawRad")},{"yaw_rate",step.step.applied_yaw_rates_radps.at(id)}});
        }
        for(const auto& edge:after.topology)CHECK((physical_positions.at(edge.owner)-physical_positions.at(edge.reference)).norm()<850);
        for(const auto& [id,p]:physical_positions)if(id<=14)
            for(const auto& [other,q]:physical_positions)if(other>id)CHECK((p-q).norm()>10);
        for(const auto& row:fixture.adapter.currentSnapshotHardRows(after.topology)) {
            CHECK(row.kind!=gf::CanonicalHardRowKind::Workspace);
            if(row.kind==gf::CanonicalHardRowKind::ReferenceDistance||row.kind==gf::CanonicalHardRowKind::Collision)CHECK(row.barrier_h>0);
        }
        const auto info=gf::task31InformationTelemetry(fixture.adapter);
        REQUIRE(info["qualified_information"]["robust_fim_min"].is_number());
        const double fim=info["qualified_information"]["robust_fim_min"],posterior=info["posterior_max_m2"];
        CHECK(fim>=1e-6);CHECK(posterior<=.1);CHECK(info["aoi_margin_min_s"].get<double>()>=0);
        CHECK(info["qualified_information"]["minimum_count"].get<int>()>=2);
        REQUIRE_FALSE(info["accepted_batch"].empty());
        minimum_fim=std::min(minimum_fim,fim);maximum_posterior=std::max(maximum_posterior,posterior);
        if(mechanism==0) {
            CHECK_FALSE(step.task32_motion_evaluated);CHECK(fixture.controller.task32UnitFrontLedger().empty());
        } else {
            REQUIRE(step.task32_motion_evaluated);REQUIRE(step.task32_motion.valid);
            REQUIRE(step.task32_motion.units.size()==2);REQUIRE(step.task32_motion.targets.size()==14);
            const auto& ledger=fixture.controller.task32UnitFrontLedger();REQUIRE(ledger.size()==2);
            std::map<std::string,Eigen::Vector2d> fronts;
            const auto& grid=fixture.adapter.coverage().certifiedGrid();
            for(const auto& unit:contract.coverage_units) {
                const auto& current=ledger.at(unit.id);const auto& motion=step.task32_motion.units.at(unit.id);
                CHECK(current.task.id()==motion.task.id());CHECK(current.active==motion.active);CHECK(current.applied_front==motion.applied_front);
                CHECK(current.task.center==Eigen::Vector2d(current.task.x_index*10.+5,current.task.y_index*10.+5));
                if(current.active)CHECK_FALSE(pre_visited.at(grid.getIndex(current.task.x_index,current.task.y_index)));
                if(previous_ledger.count(unit.id)) {
                    const double displacement=(current.applied_front-previous_ledger.at(unit.id).applied_front).norm();
                    maximum_front_step=std::max(maximum_front_step,displacement);
                    if(mechanism==2) {
                        CHECK(displacement<=29.9*.1+1e-9);
                        const auto& old_front=previous_ledger.at(unit.id).applied_front;
                        const Eigen::Vector2d delta=current.task.center-old_front;
                        const double length=delta.norm();
                        const Eigen::Vector2d expected=length<=2.99?current.task.center:
                            Eigen::Vector2d(old_front+delta*(2.99/length));
                        CHECK((current.applied_front-expected).norm()<1e-10);
                    }
                    if(mechanism==1&&!current.active)CHECK(displacement==0.);
                }
                fronts[unit.id]=current.applied_front;
                for(auto id:unit.members)CHECK(step.task32_motion.targets.at(id).id()==current.task.id());
            }
            const auto lifted=gf::task20LiftTargets(contract,scene.fixed,fronts);REQUIRE(lifted.valid);
            for(auto id:scenario.mobile_ids)CHECK(step.applied_target_centers.at(id)==lifted.targets.at(id));
            previous_ledger=ledger;
        }
        CHECK(observer.telemetry()["request_count"]==0);CHECK(observer.telemetry()["active_mode"]==0);
        json scientific={{"tick",tick},{"runtime_s",after.runtime_s},{"owners",owners},{"information",info},
            {"estimator_token",after.estimator_token},{"topology_token",after.topology_token},
            {"certified_coverage_fraction",fixture.adapter.coverage().certifiedFraction()}};
        output.scientific_ticks.push_back(scientific);output.covariance.push_back(after.estimate.covariance);
        std::cout<<"TARGET_FIXTURE_TICK "<<json({{"mechanism",mechanism},{"explicit_zero",explicit_zero},
            {"velocity_variant",velocity_variant},{"scientific",scientific}}).dump()<<'\n';
    }
    std::cout<<"TARGET_FIXTURE_SUMMARY "<<json({{"mechanism",mechanism},{"explicit_zero",explicit_zero},
        {"velocity_variant",velocity_variant},{"fixture_ticks",100},{"gaussian_seed",137029},{"link_seed",138029},
        {"minimum_fim",minimum_fim},{"maximum_posterior",maximum_posterior},{"maximum_front_step_m",maximum_front_step},
        {"scope","10-second constructed fixture only, not a formal run/window/confirmation"}}).dump()<<'\n';
    return output;
}
} // namespace

TEST_CASE("Omitted target mechanism matches explicit legacy zero for 100 noisy physical ticks") {
    const auto old=runFixture(0,false,0),explicit_zero=runFixture(0,true,0);
    CHECK(old.initial==explicit_zero.initial);CHECK(old.scientific_ticks==explicit_zero.scientific_ticks);
    REQUIRE(old.covariance.size()==100);REQUIRE(explicit_zero.covariance.size()==100);
    for(std::size_t k=0;k<100;++k)CHECK(old.covariance[k]==explicit_zero.covariance[k]);
}

TEST_CASE("B and C keep all14 task motion control and measurement contracts in short initialized fixtures") {
    json zero_initial,velocity_initial;
    for(int mechanism:{1,2})for(int variant:{0,1}) {
        const auto run=runFixture(mechanism,false,variant);
        auto& expected=variant?velocity_initial:zero_initial;
        if(expected.is_null())expected=run.initial;else CHECK(run.initial==expected);
    }
}
