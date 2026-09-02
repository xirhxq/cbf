#include "grand_finale/Task10p11sSnapshotCapture.hpp"
#include "grand_finale/Task10p11vRestartCheckpoint.hpp"
#include "grand_finale/Task14InputThreshold.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <map>

namespace {

using json=nlohmann::json;

enum class Shape { AxisBox, HorizontalDisk };

struct Evaluation {
    bool current_feasible=false;
    bool successor_feasible=false;
    double current_minimum_gamma=-std::numeric_limits<double>::infinity();
    double successor_minimum_gamma=-std::numeric_limits<double>::infinity();
    gf::NodeId current_limiting_owner=0;
    gf::NodeId successor_limiting_owner=0;
    std::map<gf::NodeId,Eigen::Vector2d> current_controls;
};

std::pair<bool,gf::Task14DiskGammaResult> solveOwner(
    const std::vector<gf::CanonicalHardRow>& rows,gf::NodeId owner,
    double bound,Shape shape) {
    if (shape==Shape::AxisBox) {
        const auto value=gf::solveCanonicalGammaStar(rows,owner,bound);
        return {value.valid,{value.valid,value.gamma,
            {value.accelX,value.accelY}}};
    }
    const auto value=gf::task14SolveDiskGamma(rows,owner,bound,1e-8);
    return {value.valid,value};
}

Evaluation evaluate(const json& snapshot,double bound,Shape shape) {
    Evaluation result;
    auto request=gf::task10p11s_capture_detail::requestFromJson(
        snapshot.at("canonical_request"));
    request.acceleration_half_box=bound;
    const auto current_rows=gf::buildCanonicalHardRows(request);
    result.current_minimum_gamma=std::numeric_limits<double>::infinity();
    for (const gf::NodeId owner:request.mobile_ids) {
        const auto solved=solveOwner(current_rows,owner,bound,shape);
        if (!solved.first) return result;
        result.current_controls[owner]=solved.second.control;
        if (solved.second.gamma<result.current_minimum_gamma) {
            result.current_minimum_gamma=solved.second.gamma;
            result.current_limiting_owner=owner;
        }
    }
    result.current_feasible=result.current_minimum_gamma>=-1e-8;
    if (!result.current_feasible) return result;

    auto successor=gf::rebuildTask10p11sSuccessorRequest(
        snapshot,result.current_controls);
    successor.acceleration_half_box=bound;
    const auto successor_rows=gf::buildCanonicalHardRows(successor);
    result.successor_minimum_gamma=std::numeric_limits<double>::infinity();
    for (const gf::NodeId owner:successor.mobile_ids) {
        const auto solved=solveOwner(successor_rows,owner,bound,shape);
        if (!solved.first) return result;
        if (solved.second.gamma<result.successor_minimum_gamma) {
            result.successor_minimum_gamma=solved.second.gamma;
            result.successor_limiting_owner=owner;
        }
    }
    result.successor_feasible=result.successor_minimum_gamma>=-1e-8;
    return result;
}

struct Threshold {
    bool found=false;
    double bound=std::numeric_limits<double>::quiet_NaN();
    Evaluation evaluation;
    double coarse_step=0.05;
    double refinement_tolerance=1e-6;
};

Threshold threshold(const json& snapshot,Shape shape,bool successor) {
    Threshold result;
    double previous=0.0;
    for (double candidate=result.coarse_step;
         candidate<=12.0+1e-12;candidate+=result.coarse_step) {
        const auto value=evaluate(snapshot,candidate,shape);
        const bool pass=successor
            ?value.current_feasible&&value.successor_feasible
            :value.current_feasible;
        if (!pass) {
            previous=candidate;
            continue;
        }
        double lower=std::max(0.0,previous-result.coarse_step);
        // previous is the last sampled failure except at the first sample.
        lower=previous;
        double upper=candidate;
        Evaluation best=value;
        while (upper-lower>result.refinement_tolerance) {
            const double middle=0.5*(lower+upper);
            const auto refined=evaluate(snapshot,middle,shape);
            const bool refined_pass=successor
                ?refined.current_feasible&&refined.successor_feasible
                :refined.current_feasible;
            if (refined_pass) {
                upper=middle;
                best=refined;
            } else lower=middle;
        }
        result.found=true;
        result.bound=upper;
        result.evaluation=evaluate(snapshot,upper,shape);
        return result;
    }
    return result;
}

json evaluationJson(const Evaluation& value) {
    const auto number=[](double scalar) {
        return std::isfinite(scalar)?json(scalar):json(nullptr);
    };
    json controls=json::object();
    for (const auto& [owner,control]:value.current_controls)
        controls[std::to_string(owner)]={control.x(),control.y()};
    return {{"current_feasible",value.current_feasible},
        {"successor_feasible",value.successor_feasible},
        {"current_minimum_gamma_mps2",number(
            value.current_minimum_gamma)},
        {"current_limiting_owner",value.current_limiting_owner},
        {"successor_minimum_gamma_mps2",number(
            value.successor_minimum_gamma)},
        {"successor_limiting_owner",value.successor_limiting_owner},
        {"current_maximum_margin_controls",controls}};
}

json thresholdJson(const Threshold& value,const std::string& semantics) {
    return {{"found",value.found},
        {"bound_mps2",value.found?json(value.bound):json(nullptr)},
        {"semantics",semantics},
        {"coarse_step_mps2",value.coarse_step},
        {"refinement_tolerance_mps2",value.refinement_tolerance},
        {"search_upper_mps2",12.0},
        {"evaluation",value.found?evaluationJson(value.evaluation):
            json(nullptr)}};
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=3) {
        std::cerr<<"usage: GrandFinaleTask14InputThresholdRun "
            "SNAPSHOT_JSON OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto snapshot=gf::readTask10p11vJson(argv[1]);
        const auto validation=gf::validateTask10p11sSnapshot(snapshot);
        if (!validation.complete)
            throw std::runtime_error("snapshot validation failed:"+
                validation.reason);
        const auto axis_current=threshold(snapshot,Shape::AxisBox,false);
        const auto axis_successor=threshold(snapshot,Shape::AxisBox,true);
        const auto disk_current=threshold(snapshot,Shape::HorizontalDisk,false);
        const auto disk_successor=threshold(snapshot,Shape::HorizontalDisk,true);
        const auto baseline_axis=evaluate(snapshot,4.0,Shape::AxisBox);
        const auto baseline_disk=evaluate(snapshot,4.0,Shape::HorizontalDisk);
        const json output={{"protocol","task14-input-threshold-v1"},
            {"source_snapshot",std::filesystem::path(argv[1]).filename().string()},
            {"snapshot_sha256_external_manifest_required",true},
            {"snapshot_task14",snapshot.value("task14",json(nullptr))},
            {"baseline_axis_box_4",evaluationJson(baseline_axis)},
            {"baseline_horizontal_norm_4",evaluationJson(baseline_disk)},
            {"minimum_current",{
                {"axis_box",thresholdJson(axis_current,
                    "|ax|,|ay|<=a_axis")},
                {"horizontal_norm",thresholdJson(disk_current,
                    "sqrt(ax^2+ay^2)<=a_norm")}}},
            {"minimum_current_and_one_step_successor",{
                {"axis_box",thresholdJson(axis_successor,
                    "|ax|,|ay|<=a_axis")},
                {"horizontal_norm",thresholdJson(disk_successor,
                    "sqrt(ax^2+ay^2)<=a_norm")}}},
            {"successor_command_semantics",
                "per-owner current maximum-minimum-margin control under the same input shape; exact-ZOH no-measurement successor; not a recursive-feasibility proof"},
            {"finite_experiment_not_proof",true}};
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 14 input threshold failed: "<<error.what()<<'\n';
        return 4;
    }
}
