#include "grand_finale/Task10p11aaMarginAudit.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <map>

namespace {

using json=nlohmann::json;

struct FirstLoss {
    std::optional<double> owner_local;
    std::optional<double> signed_transfer;
    std::optional<double> pair_component;
    std::optional<double> triplet_component;
    std::optional<double> full_pair;
    std::optional<double> successor_owner_local;
    std::optional<double> successor_signed_transfer;
    std::optional<double> successor_component;
    std::optional<double> successor_full_pair;
};

json metric(const std::optional<double>& value) {
    return value.has_value()?json{{"valid",true},{"value",*value}}
        :json{{"valid",false},{"value",nullptr},
            {"reason","not_observed_in_saved_packed_checkpoints"}};
}

bool marginFeasible(const gf::task10p11w_detail::RestrictedResult& value) {
    constexpr double tolerance=1.0e-8;
    return value.feasible && std::isfinite(value.margin) &&
        value.margin>=-tolerance && value.minimum_residual>=-tolerance;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc<3) {
        std::cerr<<"usage: GrandFinaleTask10p11aaGateA OUTPUT_JSON "
            "PACKED_CHECKPOINT...\n";
        return 2;
    }
    try {
        std::map<std::string,std::vector<gf::Task10p11aaMarginAuditFrame>> groups;
        for (int index=2;index<argc;++index) {
            const auto source=gf::readTask10p11vJson(argv[index]);
            const std::string profile=source.at("task10p11z")
                .at("profile").get<std::string>();
            auto frame=gf::task10p11aaAuditPackedCheckpoint(argv[index]);
            if (!frame.valid)
                throw std::runtime_error(std::string(argv[index])+":"+frame.reason);
            groups[profile].push_back(std::move(frame));
        }
        json profiles=json::object();
        for (auto& [profile,frames]:groups) {
            std::sort(frames.begin(),frames.end(),[](const auto& first,
                                                     const auto& second) {
                return first.time_s<second.time_s;
            });
            FirstLoss first;
            json encoded=json::array();
            for (const auto& frame:frames) {
                if (!first.owner_local.has_value() &&
                    frame.minimum_owner_local_gamma_mps2<0.0)
                    first.owner_local=frame.time_s;
                if (!first.signed_transfer.has_value() &&
                    (!frame.signed_transfer.valid ||
                     !frame.signed_transfer.shared_interval.feasible))
                    first.signed_transfer=frame.time_s;
                if (!first.pair_component.has_value() &&
                    !marginFeasible(frame.pair_component_margin))
                    first.pair_component=frame.time_s;
                if (!first.triplet_component.has_value() &&
                    frame.triplet_component_margin.status.rfind(
                        "not_applicable",0)!=0 &&
                    !marginFeasible(frame.triplet_component_margin))
                    first.triplet_component=frame.time_s;
                if (!first.full_pair.has_value() &&
                    !marginFeasible(frame.full_pair_margin))
                    first.full_pair=frame.time_s;
                if (frame.oracle_successor_performed) {
                    if (!first.successor_owner_local.has_value() &&
                        frame.oracle_successor_minimum_owner_local_gamma_mps2<0.0)
                        first.successor_owner_local=frame.time_s;
                    if (!first.successor_signed_transfer.has_value() &&
                        (!frame.oracle_successor_signed_transfer.valid ||
                         !frame.oracle_successor_signed_transfer
                              .shared_interval.feasible))
                        first.successor_signed_transfer=frame.time_s;
                    if (!first.successor_component.has_value() &&
                        !marginFeasible(
                            frame.oracle_successor_component_margin))
                        first.successor_component=frame.time_s;
                    if (!first.successor_full_pair.has_value() &&
                        !marginFeasible(frame.oracle_successor_full_pair_margin))
                        first.successor_full_pair=frame.time_s;
                }
                encoded.push_back(gf::task10p11aaMarginAuditJson(frame));
            }
            profiles[profile]={{"frames",std::move(encoded)},
                {"earliest_observed_loss_s",{
                    {"owner_local_gamma",metric(first.owner_local)},
                    {"signed_transfer",metric(first.signed_transfer)},
                    {"diagnostic_component_seed",metric(first.pair_component)},
                    {"component_triplet_2_4_6",metric(first.triplet_component)},
                    {"full_pair_global_gamma",metric(first.full_pair)},
                    {"oracle_successor_owner_local_gamma",metric(
                        first.successor_owner_local)},
                    {"oracle_successor_signed_transfer",metric(
                        first.successor_signed_transfer)},
                    {"oracle_successor_component_seed",metric(
                        first.successor_component)},
                    {"oracle_successor_full_pair_global_gamma",metric(
                        first.successor_full_pair)}}}};
        }
        const json output={{"protocol","task10p11aa-gate-a-margin-audit-v1"},
            {"checkpoint_count",argc-2},{"profiles",std::move(profiles)},
            {"terminology",{
                {"gamma_star",
                 "state_dependent_feasibility_diagnostic_and_feedback_signal_not_tunable_parameter"},
                {"tau","owner_local_predicted_gamma_feedback_threshold_mps2"},
                {"owner_local_gamma_feedback_already_used",true},
                {"full_pair_global_gamma_production_feedback_previously_used",false}}},
            {"observation_boundary",
             "earliest_among_saved_packed_checkpoints_not_continuous_crossing_time"},
            {"margin_layers_are_interchangeable",false},
            {"trajectory_run_performed",false},
            {"claim_boundary",{{"finite_saved_states_only",true},
                {"recursive_feasibility_claimed",false}}}};
        gf::writeTask10p11vJson(argv[1],output);
        std::cout<<output.dump(2)<<'\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11aa Gate A failed: "<<error.what()<<'\n';
        return 3;
    }
}
