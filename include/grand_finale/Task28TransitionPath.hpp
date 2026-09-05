#pragma once

#include "grand_finale/Task20DagLatticeContract.hpp"

namespace gf {

// A geometric path in the full labelled configuration space.  One external
// phase drives every member.  This does not advance a controller, gate safety,
// assign tasks, change roles, or introduce a per-member governor.
class Task28LayerPath {
public:
    using Targets=std::map<NodeId,Eigen::Vector2d>;
    Task28LayerPath(const Task20DagLatticeContract& goal,Targets from,Targets to,bool common_finish=false)
        :from_(std::move(from)),to_(std::move(to)),common_finish_(common_finish) {
        if (!goal.valid||from_.empty()||from_.size()!=goal.member_roles.size()||
            to_.size()!=from_.size()) throw std::invalid_argument("invalid path endpoint contract");
        for (const auto& [id,role]:goal.member_roles) {
            if (!from_.count(id)||!to_.count(id)||!from_.at(id).allFinite()||!to_.at(id).allFinite())
                throw std::invalid_argument("missing or nonfinite path member");
        }
        for (NodeId id:goal.topological_order) {
            if (!goal.member_roles.count(id)) continue;
            std::size_t depth=1;
            for (const auto& e:goal.reference_edges) if (e.owner==id&&goal.member_roles.count(e.reference)) {
                if (!depth_.count(e.reference)) throw std::invalid_argument("invalid path topological order");
                depth=std::max(depth,depth_.at(e.reference)+1);
            }
            depth_[id]=depth;layers_=std::max(layers_,depth);
        }
        if (depth_.size()!=from_.size()) throw std::invalid_argument("incomplete path topology");
    }
    Targets evaluate(double phase) const {
        if (!std::isfinite(phase)) throw std::invalid_argument("nonfinite path phase");
        if (phase<=0) return from_;
        if (phase>=1) return to_;
        Targets out;
        for (const auto& [id,p]:from_) {
            // Terminal layers clear the channel first. Internal joins have
            // zero tangent; total expansion duration remains the caller's.
            const double raw=common_finish_
                ? (phase-(1-depth_.at(id)/double(layers_)))/(depth_.at(id)/double(layers_))
                : layers_*phase-(layers_-depth_.at(id));
            const double u=std::clamp(raw,0.0,1.0);
            const double a=u*u*(3-2*u);
            out[id]=(1-a)*p+a*to_.at(id);
        }
        return out;
    }
    std::size_t layerCount() const {return layers_;}
    const std::map<NodeId,std::size_t>& depths() const {return depth_;}
private:
    Targets from_,to_;
    std::map<NodeId,std::size_t> depth_;
    std::size_t layers_=0;
    bool common_finish_=false;
};

} // namespace gf
