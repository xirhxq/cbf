#pragma once
#include <Eigen/Dense>
#include <map>
#include <stdexcept>
#include "grand_finale/Types.hpp"

namespace gf {
// External motion-reference geometry only; no controller or task selection.
class Task32ContractionPath {
    friend class ReconstructionCheckpoint;
public:
    using Targets=std::map<NodeId,Eigen::Vector2d>;
    Task32ContractionPath(Targets from,Targets to):from_(std::move(from)),to_(std::move(to)) {
        if(from_.empty()||from_.size()!=to_.size())
            throw std::invalid_argument("incomplete contraction endpoints");
        for(const auto& [id,p]:from_)
            if(!to_.count(id)||!p.allFinite()||!to_.at(id).allFinite())
                throw std::invalid_argument("invalid contraction member");
    }
    Targets evaluate(double phase) const {
        if(!std::isfinite(phase))throw std::invalid_argument("nonfinite contraction phase");
        if(phase<=0)return from_;
        if(phase>=1)return to_;
        Targets out;
        Eigen::Vector2d mean=Eigen::Vector2d::Zero();
        for(const auto& [id,p]:from_)mean+=to_.at(id)-p;
        mean/=from_.size();
        for(const auto& [id,p]:from_) {
            const Eigen::Vector2d relative=to_.at(id)-p-mean;
            out[id]=(1-phase)*p+phase*to_.at(id)+
                2*phase*(1-phase)*Eigen::Vector2d(-relative.y(),relative.x());
            if(!out.at(id).allFinite())throw std::invalid_argument("nonfinite contraction target");
        }
        return out;
    }
private:
    Targets from_,to_;
};
}
