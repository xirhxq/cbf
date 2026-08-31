#include "grand_finale/Task11aDynamicTopologyCoordinator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <regex>

namespace {

using json=nlohmann::json;

json topologyIds(const std::vector<gf::DirectedEdge>& edges) {
    json ids=json::array();
    for (const auto& edge:edges) ids.push_back(edge.id());
    return ids;
}

}  // namespace

// Task 12 Phase 0 (prereg: offline margin-erosion diagnosis).  Reads a
// sequence of archived packed/sparse checkpoints, rebuilds the canonical
// rows and exact per-owner gamma* at each recorded state, and emits the
// margin-erosion timeline: gamma*(t), limiting row, limiting owner, row
// family, and the 28D minimum residual at the applied controls.
int main(int argc,char** argv) {
    if (argc<4) {
        std::cerr<<"usage: GrandFinaleTask12Phase0 OUTPUT_JSON TIMELINE_NAME "
            "CHECKPOINT_JSON [...]\n";
        return 2;
    }
    try {
        json point_array=json::array();
        for (int index=3;index<argc;++index) {
            const std::filesystem::path path=argv[index];
            const auto packed=gf::readTask10p11vJson(path.string());
            const double t=packed.at("t_s").get<double>();
            const auto& snapshot=packed.at("snapshot");
            const auto request=gf::task10p11s_capture_detail::requestFromJson(
                snapshot.at("canonical_request"));
            std::map<gf::NodeId,Eigen::Vector2d> controls;
            std::string controls_source="applied";
            try {
                controls=gf::task10p11afAppliedControlsFromCheckpoint(
                    snapshot);
            } catch (const std::exception&) {
                const auto& nominal=snapshot.at("nominal_controls");
                controls_source="nominal_fallback";
                for (const auto& item:nominal.items()) {
                    const auto value=item.value().get<std::vector<double>>();
                    controls.emplace(static_cast<gf::NodeId>(
                        std::stoull(item.key())),
                        Eigen::Vector2d(value[0],value[1]));
                }
            }
            if (controls.size()!=request.mobile_ids.size())
                throw std::runtime_error("control recovery incomplete");
            const auto rows=gf::buildCanonicalHardRows(request);
            const auto problem=gf::buildTask10p11sRows28d(rows,
                request.mobile_ids,true);
            const Eigen::VectorXd ordered=gf::task10p11sOrderedControls(
                request.mobile_ids,controls);
            std::string limiting;
            const double min_residual_28d=
                gf::task10p11af_detail::independentMinimumResidual(problem,
                    ordered,&limiting);
            double min_gamma=std::numeric_limits<double>::infinity();
            gf::NodeId limiting_owner=0;
            json per_owner=json::object();
            for (gf::NodeId owner:request.mobile_ids) {
                const auto gamma=gf::solveCanonicalGammaStar(rows,owner,
                    request.acceleration_half_box);
                if (!gamma.valid||!std::isfinite(gamma.gamma))
                    throw std::runtime_error("gamma unavailable for owner "+
                        std::to_string(owner));
                per_owner[std::to_string(owner)]={
                    {"gamma_mps2",gf::task10p11w_detail::number(
                        gamma.gamma)},
                    {"feasible",gamma.gamma>=0.0}};
                if (gamma.gamma<min_gamma) {
                    min_gamma=gamma.gamma;
                    limiting_owner=owner;
                }
            }
            // limiting canonical row at the applied controls (owner-scoped).
            std::string limiting_row;
            double row_min=std::numeric_limits<double>::infinity();
            for (const auto& row:rows) {
                if (row.owner!=limiting_owner) continue;
                const double margin=row.margin(controls.at(limiting_owner));
                if (margin<row_min) {
                    row_min=margin;
                    limiting_row=row.id;
                }
            }
            point_array.push_back({{"t_s",t},
                {"source_file",path.filename().string()},
                {"controls_source",controls_source},
                {"min_gamma_mps2",gf::task10p11w_detail::number(min_gamma)},
                {"limiting_owner",limiting_owner},
                {"limiting_row_28d",limiting},
                {"limiting_row_owner_scope",limiting_row},
                {"min_residual_28d_mps2",gf::task10p11w_detail::number(
                    min_residual_28d)},
                {"topology",topologyIds(request.reference_edges)},
                {"per_owner_gamma",per_owner}});
        }
        std::sort(point_array.begin(),point_array.end(),
            [](const json& a,const json& b) {
                return a.at("t_s").get<double>()<b.at("t_s").get<double>();
            });
        const json output={{"protocol","task12-phase0-erosion-timeline-v1"},
            {"timeline_name",argv[2]},
            {"points",point_array},
            {"claim_boundary",{{"offline_diagnosis_only",true},
                {"no_new_trajectories",true},
                {"gradual_vs_sudden_reads_from_gamma_series",true}}}};
        gf::writeTask10p11vJson(argv[1],output);
        std::cout<<output.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 12 Phase 0 failed: "<<error.what()<<'\n';
        return 4;
    }
}
