#pragma once
#include "grand_finale/Task34SharedTransferPath.hpp"
namespace gf {
// Shared reference phase and original moving-v1 completion; graph authority
// remains in the fresh-MBB adapter. No plant, task, or estimator mutation here.
class Task34TransferLifecycle {
public:
    Task34TransferLifecycle(Task34SharedTransferPath path,Task20DagLatticeContract goal,
        std::map<NodeId,Eigen::Vector2d> fixed,double start)
        :path_(std::move(path)),goal_(std::move(goal)),fixed_(std::move(fixed)),start_(start) {
        if(!goal_.valid||!std::isfinite(start_)||start_<0)throw std::invalid_argument("invalid_transfer_start");
        reference_=path_.position(0);
    }
    void advance(double now,bool graph,const std::map<NodeId,Task29MotionState>& states,bool information) {
        if(!std::isfinite(now)||now<start_||now<=last_)throw std::invalid_argument("transfer_requires_fresh_time");
        last_=now;graph_handed_=graph;
        if(formed_)return;
        const double first=path_.firstDuration();
        // rr2 (2026-10-09): the second segment previously started on duration and
        // graph alone; actual bridge arrival was never checked, so members still
        // hundreds of meters behind began chasing a departing reference and the
        // formation gate could never be met (pentagon DL->SS await_shape stall).
        // Unfold now additionally requires the existing legacy instant criteria
        // (same position thresholds RMS<=100m / max<=180m, plus ABSOLUTE speed
        // <=3m/s) evaluated against the held static bridge reference. The
        // legacy variant is used because the bridge is a stationary hold point:
        // moving_instant_ready only bounds residuals to unit-coordinated motion,
        // which a coordinated high-speed pass could still satisfy. This is an
        // engineering arrival condition reusing existing thresholds, not a new
        // theoretical tracking guarantee; the goal keeps its original moving-v1
        // and 10-tick formation audit. A held bridge that never satisfies
        // arrival remains an honest stall to be diagnosed, not bypassed.
        if(!second_start_&&now-start_>=first&&graph) {
            // rr3 (2026-10-09): evaluate arrival against the exact held bridge
            // reference path_.position(first). The rr2 version checked the
            // not-yet-updated reference_, which the constructor initializes to
            // the path START -- a fresh lifecycle could therefore release the
            // second segment while every member was still at the start
            // (Manager regression: 400 m from bridge, stale_reference_started).
            const auto bridge_reference=path_.position(first);
            const auto arrival=task29MovingCompletion(goal_,fixed_,bridge_reference,states,graph,information);
            if(arrival.valid&&arrival.legacy_instant_ready)second_start_=now;
        }
        if(!second_start_) {
            reference_=path_.position(std::min(now-start_,first));
            stage_=now-start_<first?"to_bridge":(graph?"await_arrival":"await_graph");return;
        }
        const double elapsed=now-*second_start_;
        reference_=path_.position(first+elapsed);
        const auto audit=task29MovingCompletion(goal_,fixed_,reference_,states,graph,information);
        moving_ticks_=audit.valid&&audit.moving_instant_ready?moving_ticks_+1:0;
        legacy_ticks_=audit.valid&&audit.legacy_instant_ready?legacy_ticks_+1:0;
        audit_={{"valid",audit.valid},{"reason",audit.reason},{"information_ready",information},{"graph_handed_off",graph},
            {"max_position_m",audit.maximum_position_bound},{"rms_position_m",audit.rms_position_bound},
            {"max_coordinated_speed_mps",audit.maximum_coordinated_speed_bound},
            {"max_absolute_speed_mps",audit.maximum_absolute_speed_bound},
            {"moving_instant_ready",audit.moving_instant_ready},{"legacy_instant_ready",audit.legacy_instant_ready}};
        formed_=elapsed>=path_.secondDuration()&&moving_ticks_>=10;
        stage_=formed_?"formed":elapsed>=path_.secondDuration()?"await_shape":"to_endpoint";
    }
    bool complete() const{return formed_;}
    nlohmann::json snapshot() const {
        using checkpoint::encode;
        return {{"schema","task34-transfer-lifecycle-v1"},{"path",path_.snapshot()},
            {"goal",Task34FrontMotionHistory(goal_).snapshot().at("contract")},{"fixed",encode(fixed_)},
            {"start",encode(start_)},{"last",encode(last_)},{"second_start",encode(second_start_)},
            {"moving_ticks",moving_ticks_},{"legacy_ticks",legacy_ticks_},{"formed",formed_},
            {"stage",stage_},
            {"graph_handed",graph_handed_},
            {"reference",encode(reference_)},{"audit",audit_}};
    }
    static Task34TransferLifecycle restore(const Task20DagLatticeContract& source,
        const Task20DagLatticeContract& goal,const std::map<NodeId,Eigen::Vector2d>& fixed,const nlohmann::json& saved) {
        auto data=saved;checkpoint::Fields f{data,true,{}};
        std::string schema,stage;nlohmann::json path,key,audit;
        std::map<NodeId,Eigen::Vector2d> stored_fixed,reference;
        double start=0,last=-1;std::optional<double> second;std::size_t moving=0,legacy=0;bool formed=false;
        bool graph_handed=false;
        f("schema",schema);f("path",path);f("goal",key);f("fixed",stored_fixed);f("start",start);f("last",last);
        f("second_start",second);f("moving_ticks",moving);f("legacy_ticks",legacy);f("formed",formed);
        f("stage",stage);f("graph_handed",graph_handed);f("reference",reference);f("audit",audit);f.finish();
        if(schema!="task34-transfer-lifecycle-v1"||key!=Task34FrontMotionHistory(goal).snapshot().at("contract")||
            stored_fixed!=fixed||!std::isfinite(last)||(last!=-1&&last<start)||
            (second&&(!std::isfinite(*second)||*second>last)))throw std::invalid_argument("invalid_lifecycle_checkpoint");
        Task34TransferLifecycle result(Task34SharedTransferPath::restore(source,path),goal,fixed,start);
        const double first=result.path_.firstDuration(),duration=result.path_.secondDuration();
        if(second&&*second<start+first)throw std::invalid_argument("premature_second_segment");
        if(!second&&(moving||legacy||formed))throw std::invalid_argument("completion_without_final_segment");
        if(formed&&(!second||last-*second<duration||moving<10))throw std::invalid_argument("premature_shape_completion");
        const auto expected=!second?result.path_.position(last==-1?0:std::min(last-start,first)):
            result.path_.position(first+(last-*second));
        // rr3 (2026-10-09): beyond the first duration without a second segment the
        // wait splits into await_arrival (graph already handed off, arrival gate
        // pending) and await_graph; the snapshot's graph_handed selects the
        // expected sub-stage, so checkpoints taken while awaiting arrival
        // restore canonically instead of failing lifecycle_reference_phase_mismatch.
        const std::string expected_stage=formed?"formed":second?(last-*second>=duration?"await_shape":"to_endpoint"):
            last-start<first?"to_bridge":(graph_handed?"await_arrival":"await_graph");
        if(reference!=expected||stage!=expected_stage)throw std::invalid_argument("lifecycle_reference_phase_mismatch");
        result.last_=last;result.second_start_=second;result.moving_ticks_=moving;result.legacy_ticks_=legacy;
        result.formed_=formed;result.stage_=stage;result.graph_handed_=graph_handed;
        result.reference_=reference;result.audit_=audit;
        if(result.snapshot()!=saved)throw std::invalid_argument("noncanonical_lifecycle_checkpoint");
        return result;
    }
    std::map<NodeId,Eigen::Vector2d> reference() const{return reference_;}
    std::string stage() const{return stage_;}
    nlohmann::json report() const {
        return {{"stage",stage_},{"formation_completed",formed_},{"received_s",start_},
            {"second_start_s",second_start_?nlohmann::json(*second_start_):nlohmann::json(nullptr)},
            {"first_duration_s",path_.firstDuration()},{"second_duration_s",path_.secondDuration()},
            {"moving_dwell_ticks",moving_ticks_},{"legacy_dwell_ticks",legacy_ticks_},{"moving_v1",audit_}};
    }
private:
    Task34SharedTransferPath path_;
    Task20DagLatticeContract goal_;
    std::map<NodeId,Eigen::Vector2d> fixed_;
    std::map<NodeId,Eigen::Vector2d> reference_;
    double start_=0,last_=-1;
    std::optional<double> second_start_;
    std::size_t moving_ticks_=0,legacy_ticks_=0;
    bool formed_=false;
    bool graph_handed_=false;
    std::string stage_="to_bridge";
    nlohmann::json audit_;
};
} // namespace gf
