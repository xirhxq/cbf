#pragma once

#include "grand_finale/Task28TransitionPath.hpp"
#include "grand_finale/Task29MovingCompletion.hpp"

namespace gf {

// One fixed rank-two correction per coverage unit, derived only from the
// final labelled contract. No state fitting, extra phase, target assignment,
// online optimiser, or per-member governor. Actual feasibility is external.
class Task29RoleCenterPath {
public:
    using Targets=Task28LayerPath::Targets;
    Task29RoleCenterPath(const Task20DagLatticeContract& goal,Targets from,Targets to)
        :from_(std::move(from)),to_(std::move(to)),raw_(goal,from_,to_,Task28LayerPath::Kind::Serial) {
        std::set<NodeId> seen;
        std::set<std::string> unit_ids;
        for(const auto& unit:goal.coverage_units) {
            if(unit.members.empty()||!unit_ids.insert(unit.id).second)
                throw std::invalid_argument("invalid role-center unit");
            Eigen::Matrix2d mean=Eigen::Matrix2d::Zero();
            for(auto id:unit.members) {
                if(!goal.member_roles.count(id)||!seen.insert(id).second||
                    goal.member_roles.at(id).coverage_unit!=unit.id||goal.member_roles.at(id).member!=id)
                    throw std::invalid_argument("invalid role-center membership");
                const auto m=task29RoleMatrix(goal.member_roles.at(id));
                if(!m.allFinite())throw std::invalid_argument("nonfinite role-center matrix");
                mean+=m;
            }
            mean/=unit.members.size();
            if(!mean.allFinite()||std::abs(mean.determinant())<1e-12)
                throw std::invalid_argument("singular role-center mean");
            const Eigen::Matrix2d inverse=mean.inverse();
            for(auto id:unit.members) {
                weights_[id]=task29RoleMatrix(goal.member_roles.at(id))*inverse;
                if(!weights_.at(id).allFinite())throw std::invalid_argument("nonfinite role-center weight");
            }
            units_.push_back(unit.members);
        }
        if(seen.size()!=from_.size())throw std::invalid_argument("incomplete role-center units");
    }
    Targets evaluate(double phase) const {
        auto out=raw_.evaluate(phase); // Also rejects nonfinite phase.
        if(phase<=0||phase>=1)return out;
        for(const auto& members:units_) {
            Eigen::Vector2d correction=Eigen::Vector2d::Zero();
            for(auto id:members)correction+=(1-phase)*from_.at(id)+phase*to_.at(id)-out.at(id);
            correction/=members.size();
            for(auto id:members)out.at(id)+=weights_.at(id)*correction;
        }
        return out;
    }
    Targets evaluateContinuingFront(double phase) const {
        if(!std::isfinite(phase))throw std::invalid_argument("nonfinite continuing-front phase");
        if(phase<=1)return evaluate(phase);
        auto out=to_;
        for(const auto& members:units_) {
            Eigen::Vector2d displacement=Eigen::Vector2d::Zero();
            for(auto id:members)displacement+=to_.at(id)-from_.at(id);
            displacement*=(phase-1)/members.size();
            for(auto id:members) {
                out.at(id)+=weights_.at(id)*displacement;
                if(!out.at(id).allFinite())throw std::invalid_argument("nonfinite continuing-front reference");
            }
        }
        return out;
    }
    std::size_t layerCount() const {return raw_.layerCount();}
private:
    Targets from_,to_;
    Task28LayerPath raw_;
    std::vector<std::vector<NodeId>> units_;
    std::map<NodeId,Eigen::Matrix2d> weights_;
};

} // namespace gf
