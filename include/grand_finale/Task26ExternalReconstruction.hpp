#pragma once

#include "grand_finale/Task25P0MultiDag.hpp"
#include "grand_finale/Task28TransitionPath.hpp"
#include "grand_finale/TransitionCertifier.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"

namespace gf {

inline bool task26ValidDag(const std::vector<DirectedEdge>& edges,std::size_t max_refs) {
    const TopologyRequest request{task10p10MobileIds(14),{100,101,102},
        edges,edges,2,max_refs,{},{},{}};
    return TopologyModel(request).evaluate(edges).valid;
}

struct Task26ReplacementPlan {
    bool valid=false;
    std::string reason;
    std::vector<std::pair<DirectedEdge,DirectedEdge>> replacements;
};

// Fixed finite library action, no topology optimiser. Every intermediate
// union is explicitly checked; a cyclic union is never executed.
inline Task26ReplacementPlan task26ReplacementPlan(
    std::vector<DirectedEdge> current,const std::vector<DirectedEdge>& goal) {
    Task26ReplacementPlan result;
    if (!task26ValidDag(current,2)||!task26ValidDag(goal,2)) {
        result.reason="invalid_endpoint_dag";return result;
    }
    const auto target=task25_detail::edgeSet(goal);
    while (task25_detail::edgeSet(current)!=target) {
        std::vector<std::pair<DirectedEdge,DirectedEdge>> candidates;
        for (const auto& add:goal)
            if (!transition_certifier_detail::contains(current,add))
                for (const auto& remove:current)
                    if (remove.owner==add.owner&&!target.count(remove.id()))
                        candidates.emplace_back(add,remove);
        std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b) {
            return std::make_tuple(a.first.owner,a.first.reference,a.second.reference)<
                std::make_tuple(b.first.owner,b.first.reference,b.second.reference);
        });
        bool progressed=false;
        for (const auto& pair:candidates) {
            auto united=current;united.push_back(pair.first);
            auto next=transition_certifier_detail::without(united,pair.second);
            if (!task26ValidDag(united,3)||!task26ValidDag(next,2)) continue;
            result.replacements.push_back(pair);current=std::move(next);
            progressed=true;break;
        }
        if (!progressed) {result.reason="no_acyclic_single_replacement_order";return result;}
    }
    result.valid=true;return result;
}

inline double task26SmoothStep(double t) {
    const double s=std::clamp(t,0.0,1.0);return s*s*(3.0-2.0*s);
}

// Compare the actual affine target mapping, not DAG labels or edge names.
inline bool task27SameTargetMapping(const Task20DagLatticeContract& a,
    const Task20DagLatticeContract& b) {
    if (!a.valid||!b.valid||a.coverage_units.size()!=b.coverage_units.size()||
        a.member_roles.size()!=b.member_roles.size()) return false;
    for (std::size_t i=0;i<a.coverage_units.size();++i) {
        const auto& u=a.coverage_units[i];const auto& v=b.coverage_units[i];
        if (u.id!=v.id||u.members!=v.members||u.base_anchors!=v.base_anchors||
            u.leader!=v.leader||u.front_members!=v.front_members) return false;
    }
    for (const auto& [id,u]:a.member_roles) {
        const auto p=b.member_roles.find(id);if (p==b.member_roles.end()) return false;
        const auto& v=p->second;
        if (u.member!=v.member||u.coverage_unit!=v.coverage_unit||
            u.axial_fraction!=v.axial_fraction||u.triangular_fraction!=v.triangular_fraction)
            return false;
    }
    return true;
}
inline bool task27SameMotionReference(const std::map<NodeId,Eigen::Vector2d>& a,
    const std::map<NodeId,Eigen::Vector2d>& b) {
    if (a.empty()||a.size()!=b.size()) return false;
    for (const auto& [id,p]:a) {
        const auto q=b.find(id);
        if (q==b.end()||!p.allFinite()||!q->second.allFinite()||
            (p.array()!=q->second.array()).any()) return false;
    }
    return true;
}

inline std::map<std::string,Eigen::Vector2d> task26CompactFronts(
    const Task20DagLatticeContract& contract,
    const std::map<NodeId,Eigen::Vector2d>& fixed) {
    std::map<std::string,Eigen::Vector2d> result;
    const auto n=contract.coverage_units.size();
    for (std::size_t i=0;i<n;++i)
        result[contract.coverage_units[i].id]=fixed.at(101)+Eigen::Vector2d(
            300.0*(static_cast<double>(i)-0.5*static_cast<double>(n-1)),n==1?1000.0:500.0);
    return result;
}

// External-command coordinator only: no efficiency signal, no autonomous
// candidate selection, no estimator reset, no normal-search governor.
class Task26ExternalReconstructor {
public:
    Task26ExternalReconstructor(GrandFinaleSwarmAdapter& adapter,
        Task10p11hSimpleCoverageController& controller,const std::string& action,
        double request_s=60.0)
        :adapter_(adapter),controller_(controller),action_(action),next_request_s_(request_s) {
        if (action_=="pinball-qualified-layered-centeredframe"||action_=="cross-roundtrip-qualified-layered-centeredframe") {
            expansion_kind_=Task28LayerPath::Kind::CenteredFrame;
            action_.erase(action_.size()-std::string("-centeredframe").size());
        } else if (action_=="pinball-qualified-layered-commonfinish"||action_=="cross-roundtrip-qualified-layered-commonfinish") {
            expansion_kind_=Task28LayerPath::Kind::CommonFinish;
            action_.erase(action_.size()-std::string("-commonfinish").size());
        }
        if (action_=="pinball-qualified-layered"||action_=="cross-roundtrip-qualified-layered") {
            layered_expansion_=true;
            action_.erase(action_.size()-std::string("-layered").size());
        }
        if (action_=="pinball-qualified"||action_=="cross-roundtrip-qualified") {
            qualified_contraction_=true;
            action_.erase(action_.size()-std::string("-qualified").size());
        }
        if ((action_!="pinball"&&action_!="cross-roundtrip")||
            !adapter.config().target_policy_task20_dag_lattice||
            adapter.config().task20_lattice_mode!=0||
            adapter.config().task20_target_policy!=0)
            throw std::invalid_argument("Task26 requires frozen H0/P0 entry");
    }

    void beforeStep() {
        const auto runtime=adapter_.runtimeSnapshot();
        const double now=runtime.runtime_s;
        if (adapter_.coverage().certifiedFraction()>=1.0-1e-12) return;
        if (stage_=="search" && now+1e-9>=next_request_s_) start(runtime);
        if (stage_=="search"||stage_=="blocked_partial") return;
        if (now-request_started_>=360.0) {
            if (edge_index_==0&&!runtime.adapter_transition_pending) {
                event("rejected","compact_or_first_edge_deadline");
                requests_.back()["outcome"]="rejected";
                controller_.setExternalReconstructionReference(std::nullopt);
                stage_="search";next_request_s_=std::numeric_limits<double>::infinity();
            } else {
                event("incomplete","partial_handoff_or_shape_deadline");
                requests_.back()["outcome"]="incomplete";stage_="blocked_partial";
            }
            return;
        }
        if (stage_=="contracting") {
            fraction_=task26SmoothStep((now-request_started_)/60.0);
            auto fronts=from_fronts_;
            for (auto& [unit,p]:fronts) p=(1.0-fraction_)*p+fraction_*compact_fronts_.at(unit);
            reference_=task20LiftTargets(old_contract_,runtime.estimate.fixed_positions,fronts).targets;
            controller_.setExternalReconstructionReference(reference_);
            tracking(runtime);
            if (qualified_contraction_) {
                const auto certificates=adapter_.auditReplacementPlan(plan_.replacements);
                ++plan_audits_;
                plan_prefix_=0;
                for (const auto& c:certificates) {
                    if (!c.valid) {
                        plan_reason_=c.reason+":"+c.old_state.reason+":"+
                            c.union_state.reason+":"+c.successor_state.reason;break;
                    }
                    ++plan_prefix_;
                }
                if (plan_prefix_==plan_.replacements.size()&&informationReady()) {
                    plan_reason_="all_plan_states_certified_at_current_snapshot";
                    // Freeze the CURRENT common-front reference, not the
                    // unfinished compact endpoint. No target discontinuity.
                    old_compact_targets_=reference_;
                    if (task27SameTargetMapping(old_contract_,new_contract_))
                        new_compact_targets_=task20LiftTargets(new_contract_,
                            runtime.estimate.fixed_positions,fronts).targets;
                    stage_="handoff";
                    event("contracted",plan_reason_);
                    requests_.back()["contraction_fraction_at_admission"]=fraction_;
                }
            } else if (fraction_>=1.0&&shape_ready_) {
                stage_="handoff";event("contracted","tracking_and_information_ready");
            }
        } else if (stage_=="handoff") {
            tracking(runtime);
            if (runtime.adapter_transition_pending) {
                // A failed refreshed qualification must defer the break,
                // not mutate the supervisor into HOLD as the legacy helper did.
                if (adapter_.finishReplacementAfterFreshCycle(true)) {
                    event("edge_break",plan_.replacements.at(edge_index_).second.id());
                    ++edge_index_;
                } else last_reason_=adapter_.lastCertificationReason();
                return;
            }
            if (edge_index_==plan_.replacements.size()) {
                controller_.commitExternalCoverageMode(pending_mode_);
                active_mode_=pending_mode_;
                stage_="expanding";expansion_started_=now;shape_dwell_=0;
                if (layered_expansion_)
                    expansion_path_=std::make_unique<Task28LayerPath>(
                        new_contract_,old_compact_targets_,new_compact_targets_,expansion_kind_);
                event("graph_handoff","full_dag_roles_lifting_pair_committed");
                if (qualified_contraction_&&
                    task27SameTargetMapping(old_contract_,new_contract_)&&
                    task27SameMotionReference(reference_,new_compact_targets_)&&informationReady()) {
                    // No geometric change exists to demonstrate. The normal
                    // flight speed gate remains enforced by every plant step;
                    // <=3 m/s is not an admission condition for unchanged motion.
                    restore(now,"identical_mapping_and_motion_reference_no_interpolation");
                }
                return;
            }
            const auto& pair=plan_.replacements.at(edge_index_);
            // Respect existing supervisor dwell; do not call begin in a
            // state where requestReformation is required to be deferred.
            if (now-adapter_.supervisor().lastTransitionS()+1e-12<
                adapter_.supervisor().minimumDwellS()) return;
            const auto certificate=adapter_.auditReplacement(pair.first,pair.second);
            last_reason_=certificate.reason+":"+certificate.old_state.reason+":"+
                certificate.union_state.reason+":"+certificate.successor_state.reason;
            ++qualification_attempts_;
            if (!certificate.valid||!informationReady()) {
                ++rejections_[last_reason_];return;
            }
            event("union_qualified",pair.first.id(),certificate.minimum_gamma);
            if (adapter_.beginReplacement(pair.first,pair.second,false))
                event("edge_make",pair.first.id(),certificate.minimum_gamma);
            else {last_reason_=adapter_.lastCertificationReason();++rejections_[last_reason_];}
        } else if (stage_=="expanding") {
            fraction_=task26SmoothStep((now-expansion_started_)/60.0);
            if (layered_expansion_) reference_=expansion_path_->evaluate(fraction_);
            else for (const auto& [id,p]:old_compact_targets_)
                reference_[id]=(1.0-fraction_)*p+fraction_*new_compact_targets_.at(id);
            controller_.setExternalReconstructionReference(reference_);
            tracking(runtime);
            if (fraction_>=1.0&&shape_ready_) {
                restore(now,"new_actual_shape_tracking_and_information_ready");
            }
        }
    }

    nlohmann::json telemetry() const {
        const auto runtime=adapter_.runtimeSnapshot();
        nlohmann::json edges=nlohmann::json::array(),targets=nlohmann::json::object();
        for (const auto& edge:runtime.topology) edges.push_back({edge.reference,edge.owner});
        for (const auto& [id,p]:reference_) targets[std::to_string(id)]={p.x(),p.y()};
        nlohmann::json result={{"enabled",true},{"stage",stage_},{"active_mode",active_mode_},
            {"pending_mode",pending_mode_},{"request_count",requests_.size()},
            {"edge_index",edge_index_},{"replacement_count",plan_.replacements.size()},
            {"topology_version",runtime.topology_token},{"estimator_version",runtime.estimator_token},
            {"actual_reference_edges",edges},{"motion_reference",targets},
            {"interpolation_fraction",fraction_},{"tracking_rms_bound_m",rms_},
            {"tracking_max_bound_m",max_error_},{"speed_bound_mps",max_speed_},
            {"shape_dwell_ticks",shape_dwell_},{"task_ledger_active",stage_=="search"},
            {"qualification_attempts",qualification_attempts_},{"last_reason",last_reason_}};
        if (qualified_contraction_) result["task27"]={{"qualified_contraction",true},
            {"plan_audits",plan_audits_},{"qualified_plan_prefix",plan_prefix_},
            {"plan_reason",plan_reason_}};
        if (layered_expansion_) result["task28"]={{"expansion_path",expansion_kind_==Task28LayerPath::Kind::CenteredFrame?"terminal_first_centered_frame":
                (expansion_kind_==Task28LayerPath::Kind::CommonFinish?"terminal_first_common_finish":"terminal_first_dag_layers")},
            {"layers",expansion_path_?expansion_path_->layerCount():0},
            {"search_governor",false},{"expansion_duration_s",60.0}};
        return result;
    }
    nlohmann::json report() const {
        return {{"action",action_},{"requests",requests_},{"events",events_},
            {"qualification_rejection_counts",rejections_},{"final",telemetry()},
            {"planned_request_count",action_=="cross-roundtrip"?2:1},
            {"untriggered_request_count",(action_=="cross-roundtrip"?2:1)-requests_.size()}};
    }
private:
    void restore(double now,const std::string& reason) {
        event("restored",reason);requests_.back()["outcome"]="realized";
        requests_.back()["restored_s"]=now;
        controller_.setExternalReconstructionReference(std::nullopt);stage_="search";
        next_request_s_=action_=="cross-roundtrip"&&requests_.size()==1
            ?now+60.0:std::numeric_limits<double>::infinity();
    }
    bool informationReady() {
        const auto info=adapter_.currentReferenceAudit();
        return info.minimum_effective_reference_count>=2&&info.minimum_information_edge_count>=2&&
            info.minimum_robust_fim_cone_lower_bound>=1e-6&&
            info.maximum_posterior_eigenvalue<=adapter_.config().maximum_posterior_eigenvalue_m2&&
            info.minimum_range_aoi_margin_s>=0.0;
    }
    void tracking(const GrandFinaleRuntimeSnapshot& r) {
        rms_=0;max_error_=0;max_speed_=0;
        for (std::size_t i=0;i<r.estimate.mobile_ids.size();++i) {
            const auto id=r.estimate.mobile_ids[i];
            const double error=(r.estimate.mean.segment<2>(4*i)-reference_.at(id)).norm()+
                adapter_.config().uncertainty_sigma*std::sqrt(std::max(0.0,detail::maximumPositionEigenvalue(r.estimate,id)))+
                adapter_.config().certified_shadow_single_position_support_m;
            rms_+=error*error;max_error_=std::max(max_error_,error);
            max_speed_=std::max(max_speed_,r.estimate.mean.segment<2>(4*i+2).norm()+
                adapter_.config().uncertainty_sigma*std::sqrt(std::max(0.0,detail::maximumVelocityEigenvalue(r.estimate,id))));
        }
        rms_=std::sqrt(rms_/14.0);
        const bool ok=rms_<=100.0&&max_error_<=180.0&&max_speed_<=3.0&&informationReady();
        shape_dwell_=ok?shape_dwell_+1:0;shape_ready_=shape_dwell_>=10;
    }
    void start(const GrandFinaleRuntimeSnapshot& r) {
        old_contract_=task25DagContractFromCode(active_mode_);
        pending_mode_=action_=="pinball"?12:(active_mode_==0?11:0);
        new_contract_=task25DagContractFromCode(pending_mode_);
        plan_=task26ReplacementPlan(r.topology,new_contract_.reference_edges);
        if (!plan_.valid) throw std::logic_error(plan_.reason);
        from_fronts_.clear();
        nlohmann::json ledger=nlohmann::json::object();
        for (const auto& [id,cell]:controller_.committedTargets())
            ledger[std::to_string(id)]=cell.id();
        for (const auto& unit:old_contract_.coverage_units) {
            const auto inverse=task20FrontForMemberPose(old_contract_,r.estimate.fixed_positions,
                unit.leader,controller_.committedTargets().at(unit.leader).center);
            if (!inverse.valid) throw std::logic_error(inverse.reason);
            from_fronts_[unit.id]=inverse.front;
        }
        compact_fronts_=task26CompactFronts(old_contract_,r.estimate.fixed_positions);
        old_compact_targets_=task20LiftTargets(old_contract_,r.estimate.fixed_positions,compact_fronts_).targets;
        new_compact_targets_=task20LiftTargets(new_contract_,r.estimate.fixed_positions,
            task26CompactFronts(new_contract_,r.estimate.fixed_positions)).targets;
        request_started_=r.runtime_s;edge_index_=0;shape_dwell_=0;stage_="contracting";
        requests_.push_back({{"from_mode",active_mode_},{"to_mode",pending_mode_},
            {"received_s",r.runtime_s},{"outcome","pending"},{"history_ledger",ledger},
            {"estimator_version_at_request",r.estimator_token},{"topology_version_at_request",r.topology_token}});
        event("accepted","external_pre_registered_request");
    }
    void event(const std::string& kind,const std::string& reason,double gamma=0) {
        const auto r=adapter_.runtimeSnapshot();
        events_.push_back({{"request",requests_.size()},{"kind",kind},{"reason",reason},
            {"runtime_s",r.runtime_s},{"topology_version",r.topology_token},
            {"estimator_version",r.estimator_token},{"edge_index",edge_index_},
            {"minimum_certificate_gamma",gamma},{"rms_bound_m",rms_},
            {"max_error_bound_m",max_error_},{"max_speed_bound_mps",max_speed_}});
    }
    GrandFinaleSwarmAdapter& adapter_;
    Task10p11hSimpleCoverageController& controller_;
    std::string action_,stage_="search",last_reason_;
    int active_mode_=0,pending_mode_=0;
    double next_request_s_,request_started_=0,expansion_started_=0,fraction_=0;
    double rms_=0,max_error_=0,max_speed_=0;
    std::size_t edge_index_=0,shape_dwell_=0,qualification_attempts_=0;
    bool shape_ready_=false;
    bool qualified_contraction_=false;
    bool layered_expansion_=false;
    Task28LayerPath::Kind expansion_kind_=Task28LayerPath::Kind::Serial;
    std::unique_ptr<Task28LayerPath> expansion_path_;
    std::size_t plan_audits_=0,plan_prefix_=0;
    std::string plan_reason_;
    Task20DagLatticeContract old_contract_,new_contract_;
    Task26ReplacementPlan plan_;
    std::map<std::string,Eigen::Vector2d> from_fronts_,compact_fronts_;
    std::map<NodeId,Eigen::Vector2d> reference_,old_compact_targets_,new_compact_targets_;
    nlohmann::json requests_=nlohmann::json::array(),events_=nlohmann::json::array();
    std::map<std::string,std::size_t> rejections_;
};

} // namespace gf
