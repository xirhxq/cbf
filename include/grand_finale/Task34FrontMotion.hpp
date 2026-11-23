#pragma once
#include "grand_finale/Task29MovingCompletion.hpp"
#include "grand_finale/CheckpointCodec.hpp"

namespace gf {
// Passive history of applied common-front samples. Never advances a task,
// fits actual states, changes yaw, or modifies a control. A secant is the
// measured discrete reference slope, not a continuous plant acceleration.
class Task34FrontMotionHistory {
public:
    using Fronts=std::map<std::string,Eigen::Vector2d>;
    explicit Task34FrontMotionHistory(Task20DagLatticeContract contract)
        :contract_(std::move(contract)) {
        if(!contract_.valid)throw std::invalid_argument("invalid_motion_contract");
    }
    void observe(double time_s,const Fronts& applied_fronts) {
        if(!std::isfinite(time_s)||time_s<0||(samples_&&time_s<=time_)||
            applied_fronts.size()!=contract_.coverage_units.size())
            throw std::invalid_argument("invalid_motion_sample");
        for(const auto& unit:contract_.coverage_units)
            if(!applied_fronts.count(unit.id)||!applied_fronts.at(unit.id).allFinite())
                throw std::invalid_argument("incomplete_motion_fronts");
        if(samples_)for(const auto& [id,role]:contract_.member_roles) {
            const Eigen::Vector2d slope=task29RoleMatrix(role)*
                ((applied_fronts.at(role.coverage_unit)-last_.at(role.coverage_unit))/(time_s-time_));
            if(!slope.allFinite())throw std::invalid_argument("nonfinite_motion_slope");
        }
        if(samples_) {previous_=last_;previous_time_=time_;}
        last_=applied_fronts;time_=time_s;
        samples_=std::min<std::size_t>(2,samples_+1);
    }
    bool ready() const {return samples_==2;}
    nlohmann::json snapshot() const {
        using checkpoint::encode;
        return {{"schema","task34-applied-front-history-v1"},{"contract",contractKey()},
            {"samples",samples_},{"time",encode(time_)},{"previous_time",encode(previous_time_)},
            {"last",encode(last_)},{"previous",encode(previous_)}};
    }
    void restore(const nlohmann::json& saved) {
        auto data=saved;
        checkpoint::Fields f{data,true,{}};
        std::string schema; nlohmann::json key;
        std::size_t samples=0;double time=0,previous_time=0;Fronts last,previous;
        f("schema",schema);f("contract",key);f("samples",samples);f("time",time);
        f("previous_time",previous_time);f("last",last);f("previous",previous);f.finish();
        if(schema!="task34-applied-front-history-v1"||key!=contractKey()||samples>2)
            throw std::invalid_argument("motion_checkpoint_contract");
        Task34FrontMotionHistory candidate(contract_);
        if(samples==2)candidate.observe(previous_time,previous);
        if(samples)candidate.observe(time,last);
        if(candidate.snapshot()!=saved)throw std::invalid_argument("noncanonical_motion_checkpoint");
        *this=std::move(candidate);
    }
    std::map<NodeId,Eigen::Vector2d> memberReferenceVelocity() const {
        if(!ready())throw std::logic_error("motion_history_not_ready");
        std::map<NodeId,Eigen::Vector2d> result;
        for(const auto& [id,role]:contract_.member_roles)
            result[id]=task29RoleMatrix(role)*
                ((last_.at(role.coverage_unit)-previous_.at(role.coverage_unit))/(time_-previous_time_));
        return result;
    }
private:
    Task20DagLatticeContract contract_;
    Fronts last_,previous_;
    double time_=0,previous_time_=0;
    std::size_t samples_=0;
    nlohmann::json contractKey() const {
        using checkpoint::encode;
        nlohmann::json edges=nlohmann::json::array(),roles=nlohmann::json::array(),units=nlohmann::json::array();
        for(const auto& e:contract_.reference_edges)edges.push_back({e.reference,e.owner});
        for(const auto& [id,r]:contract_.member_roles)roles.push_back({id,r.member,r.coverage_unit,
            encode(r.axial_fraction),encode(r.triangular_fraction)});
        for(const auto& u:contract_.coverage_units)units.push_back({u.id,u.members,u.base_anchors,
            u.leader,u.front_members,u.search_observation_members,encode(u.frame_origin)});
        return {contract_.id,contract_.structural_signature,contract_.fixed_anchor_ids,
            contract_.topological_order,edges,roles,units};
    }
};
// Distinct clocked format; legacy timestamp samples never acquire tick identity.
class Task34ControlFrontMotionHistory {
public:
    using Fronts=Task34FrontMotionHistory::Fronts;
    Task34ControlFrontMotionHistory(Task20DagLatticeContract contract,double dt)
        :contract_(std::move(contract)),history_(contract_),dt_(dt) {
        if(!std::isfinite(dt_)||dt_<=0)throw std::invalid_argument("invalid_control_clock_dt");
    }
    void observeAdvanced(std::size_t tick,double time,const Fronts& fronts,bool advanced) {
        const auto old=history_.snapshot();
        if(!advanced)throw std::invalid_argument("control_sample_not_advanced");
        if(old.at("samples").get<std::size_t>()) {
            if(tick<=tick_||tick-tick_!=1)throw std::invalid_argument("nonadjacent_control_samples");
            double prior;checkpoint::decode(old.at("time"),prior);checkInterval(prior,time);
        }
        auto candidate=history_;candidate.observe(time,fronts);
        if(old.at("samples").get<std::size_t>())previous_tick_=tick_;
        tick_=tick;history_=std::move(candidate);
    }
    bool ready() const {return history_.ready();}
    std::size_t latestTick() const {return tick_;}
    Fronts frontReferenceVelocity() const {
        if(!ready())throw std::logic_error("control_history_not_ready");
        Fronts last,previous,result;const auto saved=history_.snapshot();
        checkpoint::decode(saved.at("last"),last);checkpoint::decode(saved.at("previous"),previous);
        for(const auto& [unit,p]:last)result[unit]=(p-previous.at(unit))/dt_;
        return result;
    }
    nlohmann::json snapshot() const {
        return {{"schema","task34-control-front-history-v1"},{"dt",checkpoint::encode(dt_)},
            {"tick",tick_},{"previous_tick",previous_tick_},{"history",history_.snapshot()}};
    }
    void restore(const nlohmann::json& saved) {
        auto data=saved;checkpoint::Fields f{data,true,{}};
        std::string schema;double dt;std::size_t tick,previous;nlohmann::json history;
        f("schema",schema);f("dt",dt);f("tick",tick);f("previous_tick",previous);f("history",history);f.finish();
        if(schema!="task34-control-front-history-v1"||checkpoint::encode(dt)!=checkpoint::encode(dt_))
            throw std::invalid_argument("control_checkpoint_clock_identity");
        Task34ControlFrontMotionHistory candidate(contract_,dt_);candidate.history_.restore(history);
        const auto count=history.at("samples").get<std::size_t>();
        if((count==0&&(tick||previous))||(count==1&&previous)||
            (count==2&&(tick<=previous||tick-previous!=1)))
            throw std::invalid_argument("control_checkpoint_tick_identity");
        if(count==2) {
            double a,b;checkpoint::decode(history.at("previous_time"),a);checkpoint::decode(history.at("time"),b);
            candidate.checkInterval(a,b);
        }
        candidate.tick_=tick;candidate.previous_tick_=previous;
        if(candidate.snapshot()!=saved)throw std::invalid_argument("noncanonical_control_checkpoint");
        *this=std::move(candidate);
    }
private:
    Task20DagLatticeContract contract_;
    Task34FrontMotionHistory history_;
    double dt_;
    std::size_t tick_=0,previous_tick_=0;
    void checkInterval(double previous,double time) const {
        // Only a consistency check on represented timestamps, never a speed allowance.
        const double roundoff=8*std::numeric_limits<double>::epsilon()*(std::abs(previous)+std::abs(time)+dt_);
        if(std::abs((time-previous)-dt_)>roundoff)
            throw std::invalid_argument("control_timestamp_interval_mismatch");
    }
};
} // namespace gf
