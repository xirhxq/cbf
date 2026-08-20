#include "grand_finale/Task10p11aaGraphOracle.hpp"
#include "grand_finale/Task10p11acCampaign.hpp"
#include "grand_finale/Task10p11sSnapshotCapture.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <set>

namespace {

using json=nlohmann::json;

bool hexDigest(const std::string& value,std::size_t length) {
    return value.size()==length && std::all_of(value.begin(),value.end(),
        [](unsigned char item){ return std::isxdigit(item)!=0; });
}

json sourceJson(char** argv) {
    return {{"parent_commit",argv[5]},{"parent_tree",argv[6]},
        {"cbf_commit",argv[7]},{"cbf_tree",argv[8]},
        {"binary_sha256",argv[9]}};
}

json auditInitialization(const json& manifest,const std::string& id,
    const json& prereg,const std::string& manifest_sha256) {
    const auto initialization=gf::task10p11acInitializationFromManifest(
        manifest,id);
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.mobile_positions=initialization.positions;
    auto settings=gf::task10p11acSwarmSettings(
        scenario,initialization.velocities);
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,20.0);
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized)
        throw std::runtime_error("stage_zero_initialization_failed:"+
                                 initialized.reason);
    const auto runtime=fixture->adapter.runtimeSnapshot();
    const auto request=fixture->adapter.snapshotHardRowRequest(
        runtime.estimate,runtime.topology);
    const auto rows=gf::buildCanonicalHardRows(request);
    std::map<gf::NodeId,Eigen::Vector2d> truth_positions;
    std::map<gf::NodeId,Eigen::Vector2d> truth_velocities;
    double world_margin=std::numeric_limits<double>::infinity();
    double speed_margin=std::numeric_limits<double>::infinity();
    for (const auto& robot:fixture->swarm.robots) {
        const Point point=robot->model->xy();
        const Eigen::Vector2d position(point.x,point.y);
        const Eigen::Vector2d velocity(
            robot->model->getStateVariable("vx"),
            robot->model->getStateVariable("vy"));
        truth_positions[robot->id]=position;
        truth_velocities[robot->id]=velocity;
        world_margin=std::min({world_margin,position.x(),position.y(),
            fixture->scenario.width_m-position.x(),
            fixture->scenario.height_m-position.y()});
        speed_margin=std::min(speed_margin,30.0-velocity.norm());
    }
    double collision_margin=std::numeric_limits<double>::infinity();
    for (std::size_t first=0;first<request.mobile_ids.size();++first)
        for (std::size_t second=first+1;second<request.mobile_ids.size();++second)
            collision_margin=std::min(collision_margin,
                (truth_positions.at(request.mobile_ids[first])-
                 truth_positions.at(request.mobile_ids[second])).norm()-10.0);
    double reference_850_margin=std::numeric_limits<double>::infinity();
    double reference_849_margin=std::numeric_limits<double>::infinity();
    auto positions=truth_positions;
    positions.insert(fixture->scenario.fixed_positions.begin(),
                     fixture->scenario.fixed_positions.end());
    for (const auto& edge:fixture->frozen_topology) {
        const double distance=(positions.at(edge.owner)-
                               positions.at(edge.reference)).norm();
        reference_850_margin=std::min(reference_850_margin,850.0-distance);
        reference_849_margin=std::min(reference_849_margin,849.0-distance);
    }
    double estimator_position_error=0.0,estimator_velocity_error=0.0;
    for (std::size_t index=0;index<request.mobile_ids.size();++index) {
        const auto owner=request.mobile_ids[index];
        estimator_position_error=std::max(estimator_position_error,
            (runtime.estimate.mean.segment<2>(4*index)-
             truth_positions.at(owner)).norm());
        estimator_velocity_error=std::max(estimator_velocity_error,
            (runtime.estimate.mean.segment<2>(4*index+2)-
             truth_velocities.at(owner)).norm());
    }
    const auto information=fixture->adapter.currentReferenceAudit();
    const auto metrics=gf::task10p11rFixedMetricSnapshot(information,rows);
    std::map<std::string,double> local_gamma;
    double minimum_local_gamma=std::numeric_limits<double>::infinity();
    for (const auto owner:request.mobile_ids) {
        const auto gamma=gf::solveCanonicalGammaStar(
            rows,owner,request.acceleration_half_box);
        if (!gamma.valid || !std::isfinite(gamma.gamma))
            throw std::runtime_error("current_local_gamma_invalid");
        local_gamma[std::to_string(owner)]=gamma.gamma;
        minimum_local_gamma=std::min(minimum_local_gamma,gamma.gamma);
    }
    const auto restart_before=gf::captureTask10p11vRestartFields(*fixture);
    const auto control=fixture->controller.advance();
    if (!control.step.advanced ||
        fixture->controller.lastNominalControls().size()!=14)
        throw std::runtime_error("stage_zero_distributed_control_unavailable:"+
                                 control.reason);
    const auto snapshot=gf::makeTask10p11sSnapshot(runtime,request,
        fixture->controller.lastNominalControls(),fixture->adapter.config());
    const auto snapshot_validation=gf::validateTask10p11sSnapshot(snapshot);
    const auto nominal=gf::task10p11sOrderedControls(request.mobile_ids,
        fixture->controller.lastNominalControls());
    const auto full=gf::task10p11aa_detail::fullStateAudit(request,nominal);
    const auto base_identity=gf::task10p11acBaseIdentityJson(*fixture);
    const bool config_identity_matches=
        prereg.at("base_config_without_tau")==base_identity;
    const bool record_identity_matches=
        prereg.at("initialization_record_sha256").at(id)==
            initialization.record_sha256;
    const bool pass=world_margin>=0.0 && speed_margin>=0.0 &&
        collision_margin>=0.0 && reference_850_margin>=0.0 &&
        reference_849_margin>=0.0 &&
        metrics.minimum_collision_h>=-1.0e-10 &&
        metrics.minimum_collision_psi1>=-1.0e-10 &&
        metrics.minimum_reference_h>=-1.0e-10 &&
        metrics.minimum_reference_psi1>=-1.0e-10 &&
        information.minimum_effective_reference_count>=2 &&
        information.minimum_robust_fim_cone_lower_bound>=1.0e-6 &&
        information.maximum_posterior_eigenvalue<=
            fixture->adapter.config().maximum_posterior_eigenvalue_m2 &&
        information.minimum_range_aoi_margin_s>=0.0 &&
        estimator_position_error<=1.0e-8 &&
        estimator_velocity_error<=1.0e-8 &&
        rows.size()==1226 && snapshot_validation.complete &&
        minimum_local_gamma>=-1.0e-8 && full.feasible &&
        full.problem.rows.size()==1113 && full.minimum_residual>=-1.0e-8 &&
        fixture->topologyFrozen() && config_identity_matches &&
        record_identity_matches;
    return {{"initialization",id},{"pass",pass},
        {"initialization_record_sha256",initialization.record_sha256},
        {"initialization_manifest_sha256",manifest_sha256},
        {"base_config_without_tau",base_identity},
        {"base_config_identity_matches",config_identity_matches},
        {"initialization_record_identity_matches",record_identity_matches},
        {"base_config_sha256_without_tau",
            prereg.at("base_config_sha256_without_tau")},
        {"hard_gate_sha256",prereg.at("hard_gate_sha256")},
        {"world_margin_m",world_margin},{"speed_margin_mps",speed_margin},
        {"truth_collision_margin_m",collision_margin},
        {"reference_850_margin_m",reference_850_margin},
        {"reference_849_margin_m",reference_849_margin},
        {"minimum_collision_h",metrics.minimum_collision_h},
        {"minimum_collision_psi1",metrics.minimum_collision_psi1},
        {"minimum_reference_h",metrics.minimum_reference_h},
        {"minimum_reference_psi1",metrics.minimum_reference_psi1},
        {"minimum_effective_reference_count",
            information.minimum_effective_reference_count},
        {"minimum_information_edge_count",
            information.minimum_information_edge_count},
        {"minimum_robust_fim",
            information.minimum_robust_fim_cone_lower_bound},
        {"maximum_posterior_eigenvalue_m2",
            information.maximum_posterior_eigenvalue},
        {"minimum_aoi_margin_s",information.minimum_range_aoi_margin_s},
        {"maximum_estimator_position_error_m",estimator_position_error},
        {"maximum_estimator_velocity_error_mps",estimator_velocity_error},
        {"canonical_row_count",rows.size()},
        {"snapshot_complete",snapshot_validation.complete},
        {"snapshot_reason",snapshot_validation.reason},
        {"restart_before_complete",restart_before.is_object()},
        {"minimum_current_local_gamma_mps2",minimum_local_gamma},
        {"owner_local_gamma_mps2",local_gamma},
        {"once_reserve_full_pair_feasible",full.feasible},
        {"once_reserve_full_pair_row_count",full.problem.rows.size()},
        {"once_reserve_full_pair_gamma_mps2",full.recomputed_gamma},
        {"once_reserve_full_pair_minimum_residual_mps2",
            full.minimum_residual},{"fixed_topology",fixture->topologyFrozen()}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=10) {
        std::cerr<<"usage: GrandFinaleTask10p11acPreflight OUTPUT_JSON "
            "INITIALIZATION_MANIFEST MANIFEST_SHA256 PREREG_JSON "
            "PARENT_COMMIT PARENT_TREE CBF_COMMIT CBF_TREE BINARY_SHA256\n";
        return 2;
    }
    try {
        for (int index=3;index<=3;++index)
            if (!hexDigest(argv[index],64))
                throw std::runtime_error("manifest_sha256_invalid");
        if (!hexDigest(argv[5],40) || !hexDigest(argv[6],40) ||
            !hexDigest(argv[7],40) || !hexDigest(argv[8],40) ||
            !hexDigest(argv[9],64))
            throw std::runtime_error("source_identity_invalid");
        const auto manifest=gf::readTask10p11vJson(argv[2]);
        const auto prereg=gf::readTask10p11vJson(argv[4]);
        static const std::vector<std::string> order{
            "I0-tau20","I0-tau22","P1-tau20","P1-tau22",
            "P2-tau20","P2-tau22","P3-tau20","P3-tau22"};
        if (manifest.at("protocol")!=
                "task10p11ac-initialization-manifest-v1" ||
            manifest.at("matrix_order").get<std::vector<std::string>>()!=order ||
            manifest.at("replacement_permitted")!=false ||
            prereg.at("protocol")!="task10p11ac-preregistration-v1" ||
            prereg.at("frozen_before_first_trajectory")!=true ||
            prereg.at("profile_order").get<std::vector<std::string>>()!=order ||
            prereg.at("source")!=sourceJson(argv) ||
            prereg.at("initialization_manifest_sha256")!=argv[3] ||
            prereg.at("candidate_count")!=9 ||
            prereg.at("maximum_cells")!=8 ||
            prereg.at("maximum_wall_hours_per_cell")!=6.0 ||
            !hexDigest(prereg.at("base_config_sha256_without_tau").
                get<std::string>(),64) ||
            !hexDigest(prereg.at("hard_gate_sha256").get<std::string>(),64))
            throw std::runtime_error("preflight_preregistration_identity_mismatch");
        for (const auto& cell:order) {
            const auto expected_id=cell.substr(0,2);
            const double expected_tau=cell.substr(cell.size()-2)=="20"?20.0:22.0;
            if (prereg.at("profiles").at(cell).at("initialization")!=
                    expected_id ||
                prereg.at("profiles").at(cell).at("tau_mps2")!=expected_tau)
                throw std::runtime_error("preflight_profile_matrix_mismatch");
        }
        json initializations=json::array();
        bool all_passed=true;
        for (const std::string id:{"I0","P1","P2","P3"}) {
            const auto result=auditInitialization(manifest,id,prereg,argv[3]);
            all_passed=all_passed && result.at("pass").get<bool>();
            initializations.push_back(result);
        }
        const json output={{"protocol","task10p11ac-gate1-preflight-v1"},
            {"valid",true},{"all_passed",all_passed},
            {"source",sourceJson(argv)},{"initializations",initializations},
            {"replacement_initialization_generated",false},
            {"long_trajectory_started",false}};
        gf::writeTask10p11vJson(argv[1],output);
        std::cout<<output.dump(2)<<'\n';
        return all_passed?0:4;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ac preflight failed: "<<error.what()<<'\n';
        return 3;
    }
}
