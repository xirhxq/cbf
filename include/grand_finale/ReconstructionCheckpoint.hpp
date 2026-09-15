#pragma once
#include "grand_finale/FullStateCheckpoint.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"

namespace gf::checkpoint {
#define F(x) f(#x,v.x)
#define RECORD(type,body) template<>struct Codec<type>:RecordCodec<type>{static void fields(Fields& f,type& v){body}};
RECORD(Task31LatticeCell,F(row);F(slot);)
RECORD(Task31AnchorScene,F(valid);F(reason);F(mode_code);F(width_m);F(height_m);F(bridge_spacing_m);F(ranking_span_m);F(front_similarity_gain);F(front_port_binding);F(terminal_ports);F(frame_origin);F(direction);F(final_front_offset);F(fixed);F(goal);F(unscaled_goal);F(cells);F(identity);)
RECORD(Task32ModeContract,F(valid);F(reason);F(mode_code);F(width_m);F(height_m);F(fixed);F(contract);F(initialization_version);F(mapping_kind);F(identity);F(runtime_qualified);F(strip_role_reassignment_pending);)
RECORD(Task32ModeTransitionPlan,F(valid);F(reason);F(arm);F(start_mode);F(target_modes);F(first_request_s);F(after_restore_s);F(modes);F(pinball_scene);F(identity);)
RECORD(Task26ReplacementPlan,F(valid);F(reason);F(replacements);)
#undef RECORD
#undef F
} // namespace gf::checkpoint

namespace gf {
class ReconstructionCheckpoint {
    using Json=checkpoint::Json;
    using Fields=checkpoint::Fields;
    using Targets=Task28LayerPath::Targets;
    template<class T,class Fn>static Json component(T& v,Fn fn){Json j=Json::object();Fields f{j,false,{}};fn(f,v);f.finish();return j;}
    template<class T,class Fn>static void loadComponent(T& v,const Json& j,Fn fn){Json copy=j;Fields f{copy,true,{}};fn(f,v);f.finish();}
    // Compile against the committed coordinator as well as the independently
    // frozen endpoint overlay, without importing that overlay into production.
    // A compiled-in extension is always captured; absent source fields are not
    // invented. Exact binary identity still prohibits cross-version restoration.
    template<class T,class=void>struct HasSlotOrder:std::false_type{};
    template<class T>struct HasSlotOrder<T,std::void_t<decltype(std::declval<T&>().goal_lateral_bridge_ties_)>>:std::true_type{};
    template<class T,class=void>struct HasEndpoint:std::false_type{};
    template<class T>struct HasEndpoint<T,std::void_t<decltype(std::declval<T&>().reference_legal_endpoint_)>>:std::true_type{};
    template<class T>static void extensionConfig(Fields& f,T& v){
        if constexpr(HasSlotOrder<T>::value)f("goal_lateral_bridge_ties_",v.goal_lateral_bridge_ties_);
        if constexpr(HasEndpoint<T>::value){
            f("registered_shape_endpoint_",v.registered_shape_endpoint_);
            f("reference_legal_endpoint_",v.reference_legal_endpoint_);
        }
    }
    template<class T>static void extensionState(Fields& f,T& v){
        if constexpr(HasEndpoint<T>::value){
            f("formation_endpoint_telemetry_",v.formation_endpoint_telemetry_);
            f("registered_endpoint_fronts_",v.registered_endpoint_fronts_);
        }
    }
    template<class T>static bool referenceLegalEndpoint(const T& v){
        if constexpr(HasEndpoint<T>::value)return v.reference_legal_endpoint_;
        return false;
    }
    static void layer(Fields& f,Task28LayerPath& v){
#define F(x) f(#x,v.x)
        F(from_);F(to_);F(depth_);F(layers_);F(kind_);
#undef F
    }
    static void role(Fields& f,Task29RoleCenterPath& v){
#define F(x) f(#x,v.x)
        F(from_);F(to_);F(units_);F(unit_names_);F(weights_);F(role_matrices_);
        Json raw=f.loading?f.object.at("raw"):component(v.raw_,layer);
        if(f.loading)loadComponent(v.raw_,raw,layer);
        f.names.insert("raw");if(!f.loading)f.object["raw"]=std::move(raw);
#undef F
    }
    static void contraction(Fields& f,Task32ContractionPath& v){f("from",v.from_);f("to",v.to_);}
    static void sequence(Fields& f,Task32RequestSequence& v){
#define F(x) f(#x,v.x)
        F(targets_);F(next_s_);F(after_s_);F(accepted_s_);F(started_);F(completed_);F(active_);F(stopped_);
#undef F
        if(v.started_>v.targets_.size()||v.completed_>v.started_)throw std::invalid_argument("checkpoint request cursor");
    }
    template<class T,class Fn,class Factory>static void pointer(Fields& f,const char* name,std::unique_ptr<T>& value,Fn fn,Factory factory){
        f.names.insert(name);
        if(!f.loading){f.object[name]=value?component(*value,fn):Json(nullptr);return;}
        const auto& j=f.object.at(name);if(j.is_null()){value.reset();return;}
        auto next=factory(j);loadComponent(*next,j,fn);value=std::move(next);
    }
    static Targets targets(const Json& j,const char* key){Targets t;checkpoint::decode(j.at(key),t);return t;}
#ifdef GF_CHECKPOINT_R2_OVERLAY
    static void clock(Fields& f,Task33SharedPhase& v){
        f("knots",v.knots_);f("bounds",v.bounds_);
        Json d=f.loading?f.object.at("derivatives"):Json::array();
        if(f.loading){v.derivatives_.clear();for(const auto& row:d){std::vector<Task33SharedPhase::Derivative> r;for(const auto& item:row){Task33SharedPhase::Derivative x;checkpoint::decode(item.at("controls"),x.controls);checkpoint::decode(item.at("pad"),x.pad);r.push_back(x);}v.derivatives_.push_back(std::move(r));}}
        else for(const auto& row:v.derivatives_){Json r=Json::array();for(const auto& x:row)r.push_back({{"controls",checkpoint::encode(x.controls)},{"pad",checkpoint::encode(x.pad)}});d.push_back(std::move(r));}
        f.names.insert("derivatives");if(!f.loading)f.object["derivatives"]=std::move(d);
        if(v.knots_.size()!=v.bounds_.size()+1||v.derivatives_.size()!=v.bounds_.size())throw std::invalid_argument("checkpoint phase clock dimensions");
    }
#endif
    static void config(Fields& f,Task26ExternalReconstructor& v){
#define F(x) f(#x,v.x)
        F(action_);F(bridge_scale_);F(old_bridge_only_);F(triangular_final_);F(common_bridge_);
        extensionConfig(f,v);
        F(anchor_scene_);F(fixed_mode_observer_);F(fixed_mode_code_);F(mode_plan_);
        F(qualified_contraction_);F(layered_expansion_);F(centered_bow_contraction_);F(moving_completion_);
        F(projected_compact_);F(linear_expansion_phase_);F(role_center_expansion_);F(continuing_front_);F(expansion_kind_);
#ifdef GF_CHECKPOINT_R2_OVERLAY
        F(shared_phase_speed_);
#endif
#undef F
    }
    static void state(Fields& f,Task26ExternalReconstructor& v){
#define F(x) f(#x,v.x)
        F(stage_);F(last_reason_);F(active_mode_);F(pending_mode_);F(next_request_s_);F(request_started_);F(expansion_started_);F(fraction_);F(motion_phase_);
        extensionState(f,v);F(common_bridge_metadata_);F(triangular_roles_);F(rms_);F(max_error_);F(max_speed_);
        F(edge_index_);F(shape_dwell_);F(qualification_attempts_);F(shape_ready_);F(compact_telemetry_);F(legacy_shadow_dwell_);F(moving_telemetry_);
        F(plan_audits_);F(plan_prefix_);F(plan_reason_);F(old_contract_);F(new_contract_);F(plan_);
        F(from_fronts_);F(compact_fronts_);F(canonical_compact_fronts_);F(task31_continuation_rates_);
        F(reference_);F(old_compact_targets_);F(new_compact_targets_);F(continuation_from_);F(initial_motion_targets_);
        F(requests_);F(events_);F(rejections_);
        pointer(f,"sequence",v.sequence_,sequence,[&](const Json&){
            if(!v.mode_plan_)throw std::invalid_argument("checkpoint sequence requires its sealed startup library");
            // Construct from the valid original plan, then restore the actual
            // queue/cursor. A legitimate future cancellation can leave it empty.
            return std::make_unique<Task32RequestSequence>(v.mode_plan_->target_modes,
                v.mode_plan_->first_request_s,v.mode_plan_->after_restore_s);});
        pointer(f,"contraction_path",v.contraction_path_,contraction,[](const Json& j){return std::make_unique<Task32ContractionPath>(targets(j,"from"),targets(j,"to"));});
        pointer(f,"expansion_path",v.expansion_path_,layer,[&](const Json& j){return std::make_unique<Task28LayerPath>(v.new_contract_,targets(j,"from_"),targets(j,"to_"));});
        pointer(f,"role_center_path",v.role_center_path_,role,[&](const Json& j){return std::make_unique<Task29RoleCenterPath>(v.new_contract_,targets(j,"from_"),targets(j,"to_"));});
#ifdef GF_CHECKPOINT_R2_OVERLAY
        F(recovery_started_);F(phase_clock_s_);F(continuation_bound_);
        auto clock_factory=[&](const Json& j){std::vector<double> knots;checkpoint::decode(j.at("knots"),knots);return std::make_unique<Task33SharedPhase>([&](double){return v.initial_motion_targets_;},knots);};
        pointer(f,"contraction_clock",v.contraction_clock_,clock,clock_factory);
        pointer(f,"expansion_clock",v.expansion_clock_,clock,clock_factory);
#endif
#undef F
    }
public:
    static Json boundaryKey(const Task26ExternalReconstructor& r) {
        return {r.stage_,r.edge_index_,r.adapter_.supervisor().topologyVersion(),r.adapter_.runtimeSnapshot().adapter_transition_pending,r.requests_.size()};
    }
    static bool requestDue(const Task26ExternalReconstructor& r) {
        const double now=r.adapter_.runtimeSnapshot().runtime_s;
        return !r.fixed_mode_observer_&&r.stage_=="search"&&
            (r.sequence_?r.sequence_->due(now):now+1e-9>=r.next_request_s_);
    }
    // Only unaccepted external requests may change. No plant, estimator,
    // controller, pending certificate, active path or completed ledger is reset.
    // The sealed library and all existing protocol admission restrictions apply.
    static Json replaceFutureRequests(Task26ExternalReconstructor& r,
        const std::vector<int>& future,std::optional<double> first_request_s=std::nullopt) {
        if(!r.mode_plan_||!r.sequence_||r.fixed_mode_observer_||r.sequence_->stopped_)
            throw std::invalid_argument("branch requires an active startup-sealed request library");
        auto& q=*r.sequence_;const auto& plan=*r.mode_plan_;
        const double now=r.adapter_.runtimeSnapshot().runtime_s;
        if(q.started_+future.size()>3)throw std::invalid_argument("branch exceeds frozen finite request contract");
        if(first_request_s&&(!std::isfinite(*first_request_s)||*first_request_s<now-1e-9||q.active_))
            throw std::invalid_argument("cannot retime the past or an active request; pending future retains after-restore dwell");
        if(!q.active_&&r.stage_!="search")throw std::invalid_argument("coordinator is not at an external-request boundary");
        int prior=q.active_?r.pending_mode_:r.active_mode_;
        for(int mode:future) {
            if(!plan.modes.count(mode)||mode==prior)
                throw std::invalid_argument("future target is unregistered or repeats the active mode");
            const auto& from=plan.mode(prior);const auto& to=plan.mode(mode);
            if(from.fixed!=r.adapter_.runtimeSnapshot().estimate.fixed_positions||to.fixed!=from.fixed||
               !from.valid||!to.valid)
                throw std::invalid_argument("future target changes physical scene or invalid contract");
            if(referenceLegalEndpoint(r)&&!task27SameTargetMapping(from.contract,to.contract))
                throw std::invalid_argument("future target violates frozen zero-continuation endpoint admission");
            prior=mode;
        }
        const double next=first_request_s.value_or(q.next_s_);
        if(!q.active_&&!future.empty()&&(!std::isfinite(next)||next<now-1e-9))
            throw std::invalid_argument("new future queue requires a nonpast finite first request");
        const auto original=component(q,sequence);
        auto targets=q.targets_;targets.resize(q.started_);targets.insert(targets.end(),future.begin(),future.end());
        q.targets_=std::move(targets);
        if(!q.active_)q.next_s_=future.empty()?std::numeric_limits<double>::infinity():next;
        const Json receipt={{"event","checkpoint_future_requests_replaced"},{"runtime_s",now},
            {"before",original},{"after",component(q,sequence)},
            {"admission","sealed library only; dynamic old/union/successor qualification still required"},
            {"shared_prefix",true},{"independent_noise_realization",false}};
        r.events_.push_back(receipt);return receipt;
    }
    static Json capture(Swarm& s,GrandFinaleSwarmAdapter& a,Task10p11hSimpleCoverageController& c,
        Task26ExternalReconstructor& r,const Json& identity){
        return {{"schema","grand-finale-reconstruction-checkpoint-v1"},
            {"core",FullStateCheckpoint::capture(s,a,c,identity)},
            {"reconstruction_config",component(r,config)},{"reconstruction",component(r,state)}};
    }
    static void restore(Swarm& s,GrandFinaleSwarmAdapter& a,Task10p11hSimpleCoverageController& c,
        Task26ExternalReconstructor& r,const Json& saved,const Json& identity){
        if(saved.size()!=4||saved.at("schema")!="grand-finale-reconstruction-checkpoint-v1"||saved.at("reconstruction_config")!=component(r,config))
            throw std::invalid_argument("checkpoint reconstruction configuration mismatch");
        const auto before=capture(s,a,c,r,identity);
        try{FullStateCheckpoint::restore(s,a,c,saved.at("core"),identity);loadComponent(r,saved.at("reconstruction"),state);
            if(capture(s,a,c,r,identity)!=saved)throw std::runtime_error("checkpoint reconstruction readback mismatch");}
        catch(...){FullStateCheckpoint::restore(s,a,c,before.at("core"),identity);loadComponent(r,before.at("reconstruction"),state);throw;}
    }
};
} // namespace gf
