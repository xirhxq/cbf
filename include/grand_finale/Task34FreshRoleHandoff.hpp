#pragma once
#include "grand_finale/Task26ExternalReconstruction.hpp"
#include "grand_finale/Task34RegisteredRoles.hpp"

namespace gf {
// Graph handoff only. Completion here is never moving-v1 or task completion.
class Task34FreshRoleHandoff {
public:
    Task34FreshRoleHandoff(GrandFinaleSwarmAdapter& adapter,Task10p11hSimpleCoverageController& controller,
        const Task34RegisteredRoles& registry,nlohmann::json instance)
        :Task34FreshRoleHandoff(adapter,controller,registry,std::move(instance),
            adapter.runtimeSnapshot().topology,adapter.runtimeSnapshot().runtime_s) {
        if(adapter.runtimeSnapshot().adapter_transition_pending)
            throw std::invalid_argument("role_handoff_already_pending");
    }
    void advance() {
        if(committed_)return;
        const auto runtime=adapter_.runtimeSnapshot();
        if(runtime.runtime_s<=last_s_)throw std::logic_error("role_handoff_requires_fresh_control_cycle");
        last_s_=runtime.runtime_s;
        auto actual_expected=expected_;
        if(made_)actual_expected.push_back(plan_.replacements.at(index_).first);
        if(runtime.adapter_transition_pending!=made_||
            task25_detail::edgeSet(runtime.topology)!=task25_detail::edgeSet(actual_expected))
            throw std::logic_error("role_handoff_graph_changed_outside_frozen_plan");
        if(made_) {
            if(adapter_.finishReplacementAfterFreshCycle(true)) {
                const auto& pair=plan_.replacements.at(index_);
                expected_.push_back(pair.first);
                expected_=transition_certifier_detail::without(expected_,pair.second);
                event("edge_break",pair.second.id());++index_;made_=false;
            } else reject("fresh_break:"+adapter_.lastCertificationReason());
            return;
        }
        if(index_==plan_.replacements.size()) {
            if(!informationReady()){reject("information_not_ready_at_full_graph");return;}
            controller_.commitExternalRoleCoverageMode(registry_,instance_);
            committed_=true;event("graph_committed","registered_roles_lifting_bound_not_shape_completion");return;
        }
        if(!admitted_) {
            ++whole_plan_audits_;
            const auto certificates=adapter_.auditReplacementPlan(plan_.replacements);
            for(const auto& c:certificates)if(!c.valid){reject("whole_plan:"+reason(c));return;}
            if(certificates.size()!=plan_.replacements.size()||!informationReady()) {
                reject("whole_plan_information_or_count_not_ready");return;
            }
            admitted_=true;event("snapshot_plan_qualified","not_a_future_safety_guarantee");
        }
        if(runtime.runtime_s-adapter_.supervisor().lastTransitionS()+1e-12<adapter_.supervisor().minimumDwellS()) {
            ++dwell_blocks_;return;
        }
        const auto& pair=plan_.replacements.at(index_);
        const auto certificate=adapter_.auditReplacement(pair.first,pair.second);++fresh_audits_;
        if(!certificate.valid){reject("fresh_make:"+reason(certificate));return;}
        if(!informationReady()){reject("information_not_ready_at_make");return;}
        if(adapter_.beginReplacement(pair.first,pair.second,false)) {
            made_=true;event("edge_make",pair.first.id());
        } else reject("begin:"+adapter_.lastCertificationReason());
    }
    bool graphCommitted() const {return committed_;}
    nlohmann::json snapshot() const {
        using checkpoint::encode;
        nlohmann::json edges=nlohmann::json::array();
        for(const auto& e:initial_)edges.push_back({e.reference,e.owner});
        return {{"schema","task34-fresh-role-handoff-v1"},{"instance",instance_},{"initial_graph",edges},
            {"index",index_},{"made",made_},{"admitted",admitted_},{"committed",committed_},
            {"received_s",encode(received_s_)},{"last_s",encode(last_s_)},
            {"whole_plan_audits",whole_plan_audits_},{"fresh_audits",fresh_audits_},{"dwell_blocks",dwell_blocks_},
            {"rejections",encode(rejections_)},{"events",events_}};
    }
    static Task34FreshRoleHandoff restore(GrandFinaleSwarmAdapter& adapter,
        Task10p11hSimpleCoverageController& controller,const Task34RegisteredRoles& registry,
        const nlohmann::json& saved) {
        auto data=saved;checkpoint::Fields f{data,true,{}};
        std::string schema;nlohmann::json instance,edges,events;
        std::size_t index=0,whole=0,fresh=0,dwell=0;bool made=false,admitted=false,committed=false;
        double received=0,last=0;std::map<std::string,std::size_t> rejections;
        f("schema",schema);f("instance",instance);f("initial_graph",edges);f("index",index);
        f("made",made);f("admitted",admitted);f("committed",committed);f("received_s",received);
        f("last_s",last);f("whole_plan_audits",whole);f("fresh_audits",fresh);f("dwell_blocks",dwell);
        f("rejections",rejections);f("events",events);f.finish();
        if(schema!="task34-fresh-role-handoff-v1"||!edges.is_array()||!events.is_array()||
            !std::isfinite(received)||received<0||!std::isfinite(last)||
            (last!=-1&&last<received)||last>adapter.runtimeSnapshot().runtime_s)
            throw std::invalid_argument("invalid_role_handoff_checkpoint");
        std::vector<DirectedEdge> initial;
        for(const auto& e:edges) {
            if(!e.is_array()||e.size()!=2||!e[0].is_number_integer()||!e[1].is_number_integer())
                throw std::invalid_argument("invalid_checkpoint_graph_edge");
            initial.push_back({e[0].get<NodeId>(),e[1].get<NodeId>()});
        }
        Task34FreshRoleHandoff result(adapter,controller,registry,instance,initial,received);
        const auto count=result.plan_.replacements.size();
        if(index>count||(made&&(index==count||!admitted))||(index&&!admitted)||
            (committed&&(index!=count||made)))throw std::invalid_argument("inconsistent_handoff_phase");
        for(std::size_t k=0;k<index;++k) {
            result.expected_.push_back(result.plan_.replacements[k].first);
            result.expected_=transition_certifier_detail::without(result.expected_,result.plan_.replacements[k].second);
        }
        auto expected=result.expected_;if(made)expected.push_back(result.plan_.replacements[index].first);
        const auto runtime=adapter.runtimeSnapshot();
        if(runtime.adapter_transition_pending!=made||task25_detail::edgeSet(runtime.topology)!=task25_detail::edgeSet(expected))
            throw std::invalid_argument("checkpoint_actual_graph_or_pending_mismatch");
        if(committed&&Task34FrontMotionHistory(controller.task34AppliedCoverageContract()).snapshot().at("contract")!=
            Task34FrontMotionHistory(result.goal_.contract).snapshot().at("contract"))
            throw std::invalid_argument("checkpoint_committed_role_mapping_mismatch");
        result.index_=index;result.made_=made;result.admitted_=admitted;result.committed_=committed;
        result.last_s_=last;result.whole_plan_audits_=whole;result.fresh_audits_=fresh;
        result.dwell_blocks_=dwell;result.rejections_=rejections;result.events_=events;
        if(result.snapshot()!=saved)throw std::invalid_argument("noncanonical_role_handoff_checkpoint");
        return result;
    }
    nlohmann::json report() const {
        nlohmann::json pairs=nlohmann::json::array();
        for(const auto& [add,remove]:plan_.replacements)pairs.push_back({{"make",{add.reference,add.owner}},
            {"break",{remove.reference,remove.owner}}});
        return {{"graph_committed",committed_},{"formation_completed",false},{"implemented",true},
            {"received_s",received_s_},{"replacement_count",plan_.replacements.size()},{"edge_index",index_},
            {"whole_plan_audits",whole_plan_audits_},{"fresh_make_audits",fresh_audits_},
            {"dwell_blocks",dwell_blocks_},{"rejections",rejections_},{"events",events_},
            {"replacement_plan",pairs},{"frozen_role_instance",instance_}};
    }
private:
    Task34FreshRoleHandoff(GrandFinaleSwarmAdapter& adapter,Task10p11hSimpleCoverageController& controller,
        const Task34RegisteredRoles& registry,nlohmann::json instance,std::vector<DirectedEdge> initial,double received)
        :adapter_(adapter),controller_(controller),registry_(registry),instance_(std::move(instance)),
         initial_(std::move(initial)),received_s_(received) {
        goal_=registry_.restore(instance_);
        if(adapter_.runtimeSnapshot().estimate.fixed_positions!=goal_.fixed)
            throw std::invalid_argument("role_handoff_scene_mismatch");
        expected_=initial_;
        plan_=task26ReplacementPlan(expected_,goal_.contract.reference_edges,goal_.contract.fixed_anchor_ids);
        if(!plan_.valid)throw std::invalid_argument("role_handoff_structural_plan:"+plan_.reason);
    }
    GrandFinaleSwarmAdapter& adapter_;
    Task10p11hSimpleCoverageController& controller_;
    Task34RegisteredRoles registry_;
    nlohmann::json instance_,events_=nlohmann::json::array();
    Task32ModeContract goal_;
    Task26ReplacementPlan plan_;
    std::vector<DirectedEdge> initial_,expected_;
    std::size_t index_=0,whole_plan_audits_=0,fresh_audits_=0,dwell_blocks_=0;
    bool made_=false,admitted_=false,committed_=false;
    double received_s_=0,last_s_=-1;
    std::map<std::string,std::size_t> rejections_;
    bool informationReady() {
        const auto info=adapter_.currentReferenceAudit();
        return info.minimum_effective_reference_count>=2&&info.minimum_information_edge_count>=2&&
            info.minimum_robust_fim_cone_lower_bound>=1e-6&&
            info.maximum_posterior_eigenvalue<=adapter_.config().maximum_posterior_eigenvalue_m2&&
            info.minimum_range_aoi_margin_s>=0;
    }
    template<class Certificate>static std::string reason(const Certificate& c) {
        return c.reason+":"+c.old_state.reason+":"+c.union_state.reason+":"+c.successor_state.reason;
    }
    void reject(const std::string& why){++rejections_[why];}
    void event(const std::string& kind,const std::string& reason) {
        const auto r=adapter_.runtimeSnapshot();
        events_.push_back({{"kind",kind},{"reason",reason},{"runtime_s",r.runtime_s},
            {"edge_index",index_},{"topology_version",r.topology_token},{"estimator_version",r.estimator_token}});
    }
};
} // namespace gf
