#pragma once
#include "grand_finale/Task34FreshRoleHandoff.hpp"
#include "grand_finale/Task34RequestGeometry.hpp"
#include "grand_finale/Task34TransferLifecycle.hpp"
#include "grand_finale/Task35RegisteredRequest.hpp"

namespace gf {
class Task34RoleReconstruction {
public:
    Task34RoleReconstruction(GrandFinaleSwarmAdapter& adapter,Task10p11hSimpleCoverageController& controller,
        const nlohmann::json& specification)
        :adapter_(adapter),controller_(controller),spec_(specification),
         registry_(spec_.at("modes"),spec_.at("registered_asset")) {
        if(spec_.at("schema")!="task34-role-program-v1")throw std::invalid_argument("role_program_schema");
        targets_=spec_.at("target_modes").get<std::vector<int>>();next_=spec_.at("first_request_s").get<double>();
        if(!std::isfinite(next_)||next_<0)throw std::invalid_argument("role_program_first_request");
        if(spec_.contains("registered_paths")) {
            if(!spec_.at("registered_paths").is_object()||!spec_.contains("registered_first_request_tick")||
                std::abs(spec_.at("registered_first_request_tick").get<std::size_t>()*adapter_.config().dt_s-next_)>1e-9)
                throw std::invalid_argument("registered_request_tick_identity");
        }
        std::map<NodeId,NodeId> pi;for(auto id:adapter_.runtimeSnapshot().estimate.mobile_ids)pi[id]=id;
        current_=registry_.freeze(spec_.at("start_mode").get<int>(),pi);
        for(auto code:targets_)registry_.instantiate(code,pi);
        controller_.registerExternalRoleLibrary(registry_);
        if(Task34FrontMotionHistory(controller_.task34AppliedCoverageContract()).snapshot().at("contract")!=
            Task34FrontMotionHistory(registry_.restore(current_).contract).snapshot().at("contract"))
            throw std::invalid_argument("role_program_initial_mapping_mismatch");
    }
    void beforeStep(const std::optional<nlohmann::json>& history,
        const std::map<NodeId,Eigen::Vector2d>& applied,std::size_t control_tick) {
        const double now=adapter_.runtimeSnapshot().runtime_s;
        const bool registered_due=spec_.contains("registered_paths")&&index_==0;
        const bool due=registered_due?control_tick>=spec_.at("registered_first_request_tick").get<std::size_t>():now+1e-12>=next_;
        if(!life_&&index_<targets_.size()&&due)start(history,applied,control_tick);
        if(!life_)return;
        handoff_->advance();const bool graph=handoff_->graphCommitted();
        const auto goal=registry_.restore(goal_);
        if(graph&&(adapter_.runtimeSnapshot().adapter_transition_pending||
            task25_detail::edgeSet(adapter_.runtimeSnapshot().topology)!=task25_detail::edgeSet(goal.contract.reference_edges)))
            throw std::logic_error("completed_role_graph_changed_outside_protocol");
        life_->advance(now,graph,motionStates(),informationReady());
        controller_.setExternalReconstructionReference(life_->reference());
        auto& request=requests_.back();request["motion_applied"]=true;
        request["lifecycle"]=life_->report();request["handoff"]=handoff_->report();
        request["handoff"].erase("frozen_role_instance");
        request["formation_completed"]=life_->complete();
        if(life_->complete()) {
            request["restored_search_s"]=now;request["outcome"]="actual_registered_shape_completed";
            current_=goal_;controller_.setExternalReconstructionReference(std::nullopt);
            life_.reset();handoff_.reset();next_=now+60.;
        }
    }
    nlohmann::json report() const {
        const auto& active=handoff_&&handoff_->graphCommitted()?goal_:current_;
        return {{"schema","task34-role-reconstruction-v1"},{"implemented",true},
            {"requests",requests_},{"next_request_s",next_},{"queue_index",index_},{"target_modes",targets_},
            {"active",bool(life_)},{"current_mode",active.at("mode")},{"last_formed_mode",current_.at("mode")},
            {"current_role_to_physical",active.at("role_to_physical")},{"future_changes",future_changes_},
            {"fallback_used",false},{"completion_contract","moving-v1"}};
    }
    nlohmann::json snapshot() const {
        return {{"schema","task34-complete-role-program-v1"},{"spec",spec_},{"current",current_},
            {"source",source_},{"goal",goal_},{"requests",requests_},{"targets",targets_},{"index",index_},
            {"next",checkpoint::encode(next_)},{"future_changes",future_changes_},
            {"lifecycle",life_?life_->snapshot():nlohmann::json(nullptr)},
            {"handoff",handoff_?handoff_->snapshot():nlohmann::json(nullptr)}};
    }
    // Evidence only: save the full PRE-control state when this cycle could
    // satisfy the original 10-tick completion gate. No phase/control change.
    bool completionCheckpointDue() const {
        if(!life_)return false;
        const auto state=life_->report();
        if(state.at("second_start_s").is_null()||state.at("moving_dwell_ticks").get<std::size_t>()<9)return false;
        const double elapsed=adapter_.runtimeSnapshot().runtime_s-state.at("second_start_s").get<double>();
        return elapsed>=state.at("second_duration_s").get<double>();
    }
    void restore(const nlohmann::json& saved) {
        auto data=saved;checkpoint::Fields f{data,true,{}};
        std::string schema;nlohmann::json spec,current,source,goal,requests,changes,life,handoff;
        std::vector<int> targets;std::size_t index=0;double next=0;
        f("schema",schema);f("spec",spec);f("current",current);f("source",source);f("goal",goal);
        f("requests",requests);f("targets",targets);f("index",index);f("next",next);
        f("future_changes",changes);f("lifecycle",life);f("handoff",handoff);f.finish();
        if(schema!="task34-complete-role-program-v1"||spec!=spec_||!requests.is_array()||!changes.is_array()||
            index>targets.size()||index!=requests.size()||!std::isfinite(next)||next<0||life.is_null()!=handoff.is_null())
            throw std::invalid_argument("invalid_complete_role_checkpoint");
        const auto current_mode=registry_.restore(current);std::map<NodeId,NodeId> pi;
        for(const auto& [id,r]:current_mode.contract.member_roles)pi[id]=id;
        for(auto code:targets)registry_.instantiate(code,pi);
        std::unique_ptr<Task34TransferLifecycle> restored_life;
        std::unique_ptr<Task34FreshRoleHandoff> restored_handoff;
        if(!life.is_null()) {
            if(requests.empty()||current!=source||requests.back().at("outcome")!="in_progress")
                throw std::invalid_argument("active_role_checkpoint_request_mismatch");
            const auto start=registry_.restore(source),finish=registry_.restore(goal);
            restored_life=std::make_unique<Task34TransferLifecycle>(Task34TransferLifecycle::restore(start.contract,finish.contract,finish.fixed,life));
            restored_handoff=std::make_unique<Task34FreshRoleHandoff>(Task34FreshRoleHandoff::restore(adapter_,controller_,registry_,handoff));
            if(restored_life->complete()||!controller_.task34ReconstructionReference()||
                *controller_.task34ReconstructionReference()!=restored_life->reference()||
                life.at("path").at("final")!=requests.back().at("endpoint")||
                life.at("path").at("bridge")!=requests.back().at("bridge")||
                handoff.at("instance")!=goal)throw std::invalid_argument("active_reference_goal_checkpoint_mismatch");
        } else {
            if(controller_.task34ReconstructionReference()||adapter_.runtimeSnapshot().adapter_transition_pending||
                task25_detail::edgeSet(adapter_.runtimeSnapshot().topology)!=task25_detail::edgeSet(current_mode.contract.reference_edges)||
                Task34FrontMotionHistory(controller_.task34AppliedCoverageContract()).snapshot().at("contract")!=
                Task34FrontMotionHistory(current_mode.contract).snapshot().at("contract"))
                throw std::invalid_argument("search_role_checkpoint_mapping_mismatch");
        }
        current_=current;source_=source;goal_=goal;requests_=requests;targets_=targets;index_=index;next_=next;
        future_changes_=changes;life_=std::move(restored_life);handoff_=std::move(restored_handoff);
        if(snapshot()!=saved)throw std::invalid_argument("noncanonical_complete_role_checkpoint");
    }
    nlohmann::json replaceFutureRequests(const std::vector<int>& modes,std::optional<double> first) {
        const double now=adapter_.runtimeSnapshot().runtime_s;std::map<NodeId,NodeId> identity;
        for(const auto id:adapter_.runtimeSnapshot().estimate.mobile_ids)identity[id]=id;
        for(auto code:modes)registry_.instantiate(code,identity);
        if(first&&(!std::isfinite(*first)||*first<now||life_))
            throw std::invalid_argument("future_first_time_cannot_retime_active_request");
        const std::vector<int> previous(targets_.begin()+index_,targets_.end());
        targets_.resize(index_);targets_.insert(targets_.end(),modes.begin(),modes.end());
        if(!life_)next_=first.value_or(now+60.);
        nlohmann::json change={{"runtime_s",now},{"old_remaining",previous},{"new_remaining",modes},
            {"active_request_preserved",bool(life_)},{"past_request_count",index_},
            {"first_request_s",first?nlohmann::json(*first):nlohmann::json(nullptr)}};
        future_changes_.push_back(change);return change;
    }
private:
    GrandFinaleSwarmAdapter& adapter_;
    Task10p11hSimpleCoverageController& controller_;
    nlohmann::json spec_,current_,requests_=nlohmann::json::array();
    Task34RegisteredRoles registry_;
    std::vector<int> targets_;
    std::size_t index_=0;
    double next_=0;
    nlohmann::json source_,goal_,future_changes_=nlohmann::json::array();
    std::unique_ptr<Task34TransferLifecycle> life_;
    std::unique_ptr<Task34FreshRoleHandoff> handoff_;
    bool informationReady() {
        const auto info=adapter_.currentReferenceAudit();
        return info.minimum_effective_reference_count>=2&&info.minimum_information_edge_count>=2&&
            info.minimum_robust_fim_cone_lower_bound>=1e-6&&
            info.maximum_posterior_eigenvalue<=adapter_.config().maximum_posterior_eigenvalue_m2&&info.minimum_range_aoi_margin_s>=0;
    }
    std::map<NodeId,Task29MotionState> motionStates() const {
        const auto r=adapter_.runtimeSnapshot();std::map<NodeId,Task29MotionState> result;
        for(std::size_t i=0;i<r.estimate.mobile_ids.size();++i) {
            const auto id=r.estimate.mobile_ids[i];
            const double ps=adapter_.config().uncertainty_sigma*std::sqrt(std::max(0.,detail::maximumPositionEigenvalue(r.estimate,id)))+
                adapter_.config().certified_shadow_single_position_support_m;
            const double vs=adapter_.config().uncertainty_sigma*std::sqrt(std::max(0.,detail::maximumVelocityEigenvalue(r.estimate,id)));
            result[id]={r.estimate.mean.segment<2>(4*i),r.estimate.mean.segment<2>(4*i+2),ps,vs};
        }return result;
    }
    void start(const std::optional<nlohmann::json>& saved_history,const std::map<NodeId,Eigen::Vector2d>& applied,
        std::size_t control_tick) {
        const double now=adapter_.runtimeSnapshot().runtime_s;const int target=targets_.at(index_++);
        nlohmann::json request={{"received_s",now},{"source_mode",current_.at("mode")},{"target_mode",target},
            {"motion_applied",false},{"formation_completed",false},{"fallback_used",false}};
        try {
            const auto source=registry_.restore(current_);
            if(Task34FrontMotionHistory(controller_.task34AppliedCoverageContract()).snapshot().at("contract")!=
                Task34FrontMotionHistory(source.contract).snapshot().at("contract")||
                task25_detail::edgeSet(adapter_.runtimeSnapshot().topology)!=task25_detail::edgeSet(source.contract.reference_edges))
                throw std::logic_error("request_source_mapping_graph_mismatch");
            if(!saved_history)throw std::invalid_argument("applied_front_history_missing");
            Task34ControlFrontMotionHistory history(source.contract,adapter_.config().dt_s);history.restore(*saved_history);
            if(!history.ready())throw std::invalid_argument("applied_front_history_not_ready");
            if(control_tick==0||history.latestTick()!=control_tick-1)
                throw std::invalid_argument("stale_applied_front_control_tick");
            std::map<std::string,Eigen::Vector2d> last,canonical;
            const auto& samples=saved_history->at("history");
            double time=0;checkpoint::decode(samples.at("last"),last);checkpoint::decode(samples.at("time"),time);
            if(std::abs(now-time-adapter_.config().dt_s)>1e-8)throw std::invalid_argument("stale_applied_front_history");
            const auto velocities=history.frontReferenceVelocity();
            request["source_motion_clock"]=*saved_history;
            const auto lifted=task20LiftTargets(source.contract,source.fixed,last);
            if(!lifted.valid||lifted.targets.size()!=applied.size())throw std::invalid_argument("applied_reference_front_domain");
            for(const auto& [id,p]:lifted.targets)if(!applied.count(id)||(p-applied.at(id)).norm()>1e-8)
                throw std::invalid_argument("applied_reference_not_source_front_lifting");
            for(const auto& item:spec_.at("canonical_fronts").at(std::to_string(target)).items())
                canonical[item.key()]={item.value().at(0).get<double>(),item.value().at(1).get<double>()};
            std::map<NodeId,double> supports;for(const auto& [id,state]:motionStates())supports[id]=state.position_support;
            const bool registered=spec_.contains("registered_paths");
            nlohmann::json registered_asset;
            if(registered) {
                const auto key=std::to_string(current_.at("mode").get<int>())+":"+std::to_string(target);
                if(!spec_.at("registered_paths").contains(key))throw std::invalid_argument("unregistered_directed_spatial_path");
                registered_asset=spec_.at("registered_paths").at(key);
            }
            const auto geometry=registered?task35RegisteredRequest(registry_,source.contract,target,applied,canonical,registered_asset):
                task34RequestGeometry(registry_,source.contract,target,applied,canonical,supports,spec_.at("registered_asset"));
            if(!geometry.valid)throw std::invalid_argument(geometry.reason);
            const auto goal=registry_.restore(geometry.frozen_goal);
            Task34SharedTransferPath path=registered?Task34SharedTransferPath::registered(source.contract,registered_asset,source.fixed):
                Task34SharedTransferPath(source.contract,applied,geometry.bridge,geometry.endpoint,velocities);
            auto lifecycle=std::make_unique<Task34TransferLifecycle>(path,goal.contract,goal.fixed,now);
            auto handoff=std::make_unique<Task34FreshRoleHandoff>(adapter_,controller_,registry_,geometry.frozen_goal);
            request["geometry"]=geometry.metadata;request["role_to_physical"]=geometry.frozen_goal.at("role_to_physical");
            request["source_role_to_physical"]=current_.at("role_to_physical");
            request["bridge"]=checkpoint::encode(geometry.bridge);request["endpoint"]=checkpoint::encode(geometry.endpoint);
            request["endpoint_fronts"]=checkpoint::encode(geometry.endpoint_fronts);
            request["target_speed_bound_mps"]=path.speedBound();request["target_acceleration_bound_mps2"]=path.accelerationCertificate();
            if(registered) {
                request["spatial_path_schema"]="task35-registered-arclength-v1";
                request["request_control_tick"]=control_tick;
                request["nominal_acceleration_certificate"]=false;
                request["nominal_velocity_jumps_allowed"]=true;
                request["incoming_velocity_displacement_added"]=false;
            }
            request["real_task_ledger"]=nlohmann::json::object();
            for(const auto& [id,cell]:controller_.committedTargets())request["real_task_ledger"][std::to_string(id)]=cell.id();
            request["outcome"]="in_progress";
            source_=current_;goal_=geometry.frozen_goal;life_=std::move(lifecycle);handoff_=std::move(handoff);
        } catch(const std::invalid_argument& error) {
            request["outcome"]="request_rejected_before_motion";request["reason"]=error.what();next_=now+60.;
        }
        requests_.push_back(std::move(request));
    }
};
} // namespace gf
