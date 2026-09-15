#pragma once
#include "grand_finale/Task32ModeContract.hpp"
#include <optional>
#include <limits>

namespace gf {

// A finite, startup-sealed library. It selects whole contracts, never just
// topology labels. Parsing is not initialization, safety or noise admission.
struct Task32ModeTransitionPlan {
    bool valid=false;
    std::string reason;
    std::string arm;
    int start_mode=0;
    std::vector<int> target_modes;
    double first_request_s=60,after_restore_s=60;
    std::map<int,Task32ModeContract> modes;
    Task31AnchorScene pinball_scene;
    nlohmann::json identity;
    const Task32ModeContract& mode(int code) const {return modes.at(code);}
    const Task32ModeContract& start() const {return mode(start_mode);}
    int targetMechanism() const {return arm=="C"?2:0;}
};

inline Task32ModeTransitionPlan task32ModeTransitionPlan(const nlohmann::json& j,
    const nlohmann::json& independently_registered_pinball) {
    Task32ModeTransitionPlan p;
    try {
        using task32_mode_contract_detail::fields;
        if(!fields(j,{"schema","arm","start_mode","target_modes","first_request_s",
            "after_restore_s","modes"})||j.at("schema")!="task32-mode-transition-plan-v1")
            throw std::invalid_argument("transition_plan_schema");
        p.arm=j.at("arm").get<std::string>();
        if(p.arm!="A"&&p.arm!="C")throw std::invalid_argument("transition_arm");
        if(!j.at("start_mode").is_number_integer())throw std::invalid_argument("integer_start_mode_required");
        p.start_mode=j.at("start_mode").get<int>();
        p.first_request_s=j.at("first_request_s").get<double>();
        p.after_restore_s=j.at("after_restore_s").get<double>();
        if(p.first_request_s!=60||p.after_restore_s!=60)throw std::invalid_argument("frozen_request_timing");
        p.pinball_scene=task31AnchorScene(independently_registered_pinball);
        if(!p.pinball_scene.valid)throw std::invalid_argument("invalid_pinball_registration");
        if(!j.at("modes").is_array()||j.at("modes").empty())throw std::invalid_argument("empty_mode_registry");
        for(const auto& payload:j.at("modes")) {
            auto a=task32ModeContract(payload,&independently_registered_pinball);
            if(!a.valid||(a.mode_code!=0&&a.mode_code!=11&&a.mode_code!=12&&a.mode_code!=13))
                throw std::invalid_argument("invalid_registered_mode:"+a.reason);
            if(a.width_m!=4500||a.height_m!=2250||a.fixed!=p.pinball_scene.fixed||
                a.contract.member_roles.size()!=14)
                throw std::invalid_argument("shared_physical_scene_mismatch");
            if(!p.modes.emplace(a.mode_code,std::move(a)).second)throw std::invalid_argument("duplicate_mode");
        }
        if(!p.modes.count(p.start_mode))throw std::invalid_argument("unregistered_start_mode");
        if(!j.at("target_modes").is_array()||j.at("target_modes").empty()||j.at("target_modes").size()>3)
            throw std::invalid_argument("finite_nonempty_sequence_required");
        int prior=p.start_mode;
        for(const auto& value:j.at("target_modes")) {
            if(!value.is_number_integer())throw std::invalid_argument("integer_target_required");
            const int code=value.get<int>();
            if(!p.modes.count(code)||code==prior)throw std::invalid_argument("unregistered_or_repeated_target");
            p.target_modes.push_back(code);prior=code;
        }
        p.identity=j;p.valid=true;p.reason="parsed_not_runtime_qualified";
    }catch(const std::exception& e){p.reason=e.what();}
    return p;
}

// The only sequence clock: graph handoff alone cannot complete a request.
// stop() records interruption at the coordinator; untriggered requests remain
// in the denominator. No completion timer fires after the finite queue ends.
class Task32RequestSequence {
    friend class ReconstructionCheckpoint;
public:
    Task32RequestSequence(std::vector<int> targets,double first,double after)
        :targets_(std::move(targets)),next_s_(first),after_s_(after) {
        if(targets_.empty()||!std::isfinite(first)||first<0||!std::isfinite(after)||after<0)
            throw std::invalid_argument("invalid_request_sequence");
    }
    bool due(double now) const {
        if(!std::isfinite(now))throw std::invalid_argument("nonfinite_sequence_time");
        return !stopped_&&!active_&&started_<targets_.size()&&now+1e-9>=next_s_;
    }
    int begin(double now) {
        if(!due(now))throw std::logic_error("request_not_due");
        active_=true;accepted_s_=now;return targets_.at(started_++);
    }
    void complete(double now) {
        if(!active_||stopped_||!std::isfinite(now)||now<accepted_s_)
            throw std::logic_error("request_not_active");
        active_=false;++completed_;
        next_s_=started_<targets_.size()?now+after_s_:std::numeric_limits<double>::infinity();
    }
    void stop(){stopped_=true;active_=false;next_s_=std::numeric_limits<double>::infinity();}
    bool pending() const {return active_;}
    std::size_t planned() const {return targets_.size();}
    std::size_t completed() const {return completed_;}
    std::size_t untriggered() const {return targets_.size()-started_;}
private:
    std::vector<int> targets_;
    double next_s_,after_s_,accepted_s_=0;
    std::size_t started_=0,completed_=0;
    bool active_=false,stopped_=false;
};

// Preserve the legacy canonical H0 front endpoint. Pinball alone uses its
// registered reparameterized front; native target modes never inherit it.
inline std::map<std::string,Eigen::Vector2d> task32ModeCompactFronts(
    const Task32ModeTransitionPlan& plan,int mode) {
    const auto& c=plan.mode(mode).contract;
    std::map<std::string,Eigen::Vector2d> fronts;
    for(std::size_t i=0;i<c.coverage_units.size();++i) {
        Eigen::Vector2d f=plan.start().fixed.at(101)+Eigen::Vector2d(
            300.0*(double(i)-.5*double(c.coverage_units.size()-1)),c.coverage_units.size()==1?1000.:500.);
        if(mode==plan.pinball_scene.mode_code)
            f=plan.pinball_scene.frame_origin+plan.pinball_scene.final_front_offset;
        fronts[c.coverage_units[i].id]=f;
    }
    return fronts;
}

inline void task32ValidateTransitionStartup(const Task32ModeTransitionPlan& p,
    int configured_mode,const std::vector<DirectedEdge>& actual_topology,
    const std::map<NodeId,Eigen::Vector2d>& physical_anchors,int mechanism) {
    if(!p.valid||configured_mode!=p.start_mode||mechanism!=p.targetMechanism()||
        physical_anchors!=p.start().fixed||
        task25_detail::edgeSet(actual_topology)!=task25_detail::edgeSet(p.start().contract.reference_edges))
        throw std::invalid_argument("transition_startup_graph_mapping_arm_scene_mismatch");
}
} // namespace gf
