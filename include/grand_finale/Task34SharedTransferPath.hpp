#pragma once
#include "grand_finale/Task29MovingCompletion.hpp"
#include "grand_finale/Task34FrontMotion.hpp"
#include "grand_finale/Task35RegisteredArcPath.hpp"
#include <array>
#include <limits>

namespace gf {
// One request-frozen path for every member, with a common physical clock.
// Shape and identities come from contracts; incoming motion from unit fronts.
class Task34SharedTransferPath {
public:
    using Targets=std::map<NodeId,Eigen::Vector2d>;
    Task34SharedTransferPath(const Task20DagLatticeContract& source,Targets from,Targets bridge,Targets final,
        const std::map<std::string,Eigen::Vector2d>& incoming_front_velocity)
        :source_(source),front_velocity_(incoming_front_velocity),
         from_(std::move(from)),bridge_(std::move(bridge)),final_(std::move(final)) {
        if(!source.valid||from_.size()!=source.member_roles.size()||bridge_.size()!=from_.size()||
            final_.size()!=from_.size()||incoming_front_velocity.size()!=source.coverage_units.size())
            throw std::invalid_argument("incomplete_shared_path");
        for(const auto& unit:source.coverage_units)
            if(!incoming_front_velocity.count(unit.id)||!incoming_front_velocity.at(unit.id).allFinite())
                throw std::invalid_argument("incomplete_common_front_velocity");
        for(const auto& [id,role]:source.member_roles) {
            if(!from_.count(id)||!bridge_.count(id)||!final_.count(id)||!from_.at(id).allFinite()||
                !bridge_.at(id).allFinite()||!final_.at(id).allFinite())
                throw std::invalid_argument("invalid_shared_path_members");
            incoming_[id]=task29RoleMatrix(role)*incoming_front_velocity.at(role.coverage_unit);
            if(!incoming_.at(id).allFinite()||incoming_.at(id).norm()>budget+tolerance())
                throw std::invalid_argument("inherited_reference_velocity_exceeds_budget");
            zero_[id]=Eigen::Vector2d::Zero();
            if(!(bridge_.at(id)-from_.at(id)).allFinite()||!(final_.at(id)-bridge_.at(id)).allFinite())
                throw std::invalid_argument("nonfinite_path_displacement");
        }
        first_s_=duration(from_,bridge_,incoming_);
        second_s_=duration(bridge_,final_,zero_);
    }
    double firstDuration() const {return first_s_;}
    double secondDuration() const {return second_s_;}
    static Task34SharedTransferPath registered(const Task20DagLatticeContract& source,
        const nlohmann::json& asset,const Targets& fixed) {
        std::set<NodeId> ids;for(const auto& [id,r]:source.member_roles)ids.insert(id);
        return Task34SharedTransferPath(source,Task35RegisteredArcPath(asset,ids,fixed,budget));
    }
    nlohmann::json snapshot() const {
        using checkpoint::encode;
        if(registered_)return {{"schema","task35-shared-registered-path-v1"},{"contract",contractKey(source_)},
            {"from",encode(from_)},{"bridge",encode(bridge_)},{"final",encode(final_)},{"registered",registered_->snapshot()}};
        return {{"schema","task34-shared-transfer-v1"},{"contract",contractKey(source_)},
            {"from",encode(from_)},{"bridge",encode(bridge_)},{"final",encode(final_)},
            {"front_velocity",encode(front_velocity_)},{"first_s",encode(first_s_)},
            {"second_s",encode(second_s_)}};
    }
    static Task34SharedTransferPath restore(const Task20DagLatticeContract& source,const nlohmann::json& saved) {
        if(saved.at("schema")=="task35-shared-registered-path-v1") {
            Task34SharedTransferPath out(source,Task35RegisteredArcPath::restore(saved.at("registered")));
            if(out.snapshot()!=saved)throw std::invalid_argument("registered_path_contract_checkpoint");
            return out;
        }
        auto data=saved;checkpoint::Fields fields{data,true,{}};
        std::string schema;nlohmann::json key;Targets from,bridge,final;
        std::map<std::string,Eigen::Vector2d> fronts;double first=0,second=0;
        fields("schema",schema);fields("contract",key);fields("from",from);
        fields("bridge",bridge);fields("final",final);fields("front_velocity",fronts);
        fields("first_s",first);fields("second_s",second);fields.finish();
        if(schema!="task34-shared-transfer-v1"||key!=contractKey(source))
            throw std::invalid_argument("shared_path_checkpoint_contract");
        Task34SharedTransferPath restored(source,from,bridge,final,fronts);
        // Recompute the certificate; never trust serialized times to relax it.
        if(restored.snapshot()!=saved)throw std::invalid_argument("noncanonical_shared_path_checkpoint");
        return restored;
    }
    Targets position(double elapsed_s) const {return registered_?registered_->position(elapsed_s):evaluate(elapsed_s,0);}
    Targets velocity(double elapsed_s) const {return registered_?registered_->velocity(elapsed_s):evaluate(elapsed_s,1);}
    Targets acceleration(double elapsed_s) const {
        if(registered_)throw std::invalid_argument("registered_path_no_global_acceleration_certificate");
        return evaluate(elapsed_s,2);
    }
    double speedBound() const {return registered_?registered_->speedBudget():std::max(bound(from_,bridge_,incoming_,first_s_,false),bound(bridge_,final_,zero_,second_s_,false));}
    double accelerationBound() const {return registered_?std::numeric_limits<double>::infinity():std::max(bound(from_,bridge_,incoming_,first_s_,true),bound(bridge_,final_,zero_,second_s_,true));}
    nlohmann::json accelerationCertificate()const {
        // No finite bound is established for registered phase junctions.
        // Explicit absence must survive CBOR too, not only JSON text output.
        return registered_?nlohmann::json(nullptr):nlohmann::json(accelerationBound());
    }
    static double tolerance(){return 256*std::numeric_limits<double>::epsilon()*budget;}
private:
    Task34SharedTransferPath(const Task20DagLatticeContract& source,Task35RegisteredArcPath path)
        :source_(source),from_(path.start()),bridge_(path.bridge()),final_(path.goal()),
        first_s_(path.firstDuration()),second_s_(path.secondDuration()),registered_(std::move(path)) {
        if(!source_.valid||source_.member_roles.size()!=from_.size())throw std::invalid_argument("registered_path_source_domain");
        for(const auto& [id,r]:source_.member_roles)if(!from_.count(id))throw std::invalid_argument("registered_path_source_member");
    }
    static constexpr double budget=29.9;
    Task20DagLatticeContract source_;
    std::map<std::string,Eigen::Vector2d> front_velocity_;
    Targets from_,bridge_,final_,incoming_,zero_;
    double first_s_=60,second_s_=60;
    std::optional<Task35RegisteredArcPath> registered_;
    static nlohmann::json contractKey(const Task20DagLatticeContract& source) {
        return Task34FrontMotionHistory(source).snapshot().at("contract");
    }
    template<std::size_t N>static double refinedBound(std::array<Eigen::Vector2d,N> c,int depth=6) {
        if(depth==0){double b=0;for(const auto& v:c)b=std::max(b,v.norm());return b;}
        std::array<Eigen::Vector2d,N> left,right;
        left[0]=c[0];right[N-1]=c[N-1];
        for(std::size_t k=1;k<N;++k){
            for(std::size_t j=0;j<N-k;++j)c[j]=.5*c[j]+.5*c[j+1];
            left[k]=c[0];right[N-1-k]=c[N-1-k];
        }
        return std::max(refinedBound(left,depth-1),refinedBound(right,depth-1));
    }
    static double bound(const Targets& a,const Targets& b,const Targets& v,double duration,bool acceleration) {
        double result=0;
        for(const auto& [id,p]:a) {
            const Eigen::Vector2d middle=5*((b.at(id)-p)/duration)-2*v.at(id);
            const std::array<Eigen::Vector2d,5> controls{v.at(id),v.at(id),middle,Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
            double value;
            if(acceleration){
                std::array<Eigen::Vector2d,4> d;
                for(std::size_t k=0;k<4;++k)d[k]=4*(controls[k+1]-controls[k])/duration;
                value=refinedBound(d);
            } else value=refinedBound(controls);
            if(!std::isfinite(value))throw std::invalid_argument("nonfinite_path_derivative_bound");
            result=std::max(result,value);
        }
        return result;
    }
    static double duration(const Targets& a,const Targets& b,const Targets& v) {
        auto legal=[&](double t){return bound(a,b,v,t,false)<=budget+tolerance();};
        double low=60,high=60;
        if(legal(high))return high;
        bool found=false;
        for(int k=0;k<16;++k){high*=2;if(legal(high)){found=true;break;}}
        if(!found)throw std::invalid_argument("no_duration_in_finite_shared_time_family");
        for(int k=0;k<48;++k){const double mid=low+.5*(high-low);if(legal(mid))high=mid;else low=mid;}
        return high;
    }
    Targets evaluate(double time,int derivative) const {
        if(!std::isfinite(time)||time<0)throw std::invalid_argument("invalid_path_time");
        if(time==0)return derivative==0?from_:(derivative==1?incoming_:zero_);
        if(time==first_s_)return derivative==0?bridge_:zero_;
        if(time>=first_s_+second_s_)return derivative==0?final_:zero_;
        const bool first=time<first_s_;
        const auto& a=first?from_:bridge_;const auto& b=first?bridge_:final_;
        const auto& v=first?incoming_:zero_;
        const double T=first?first_s_:second_s_,s=(first?time:time-first_s_)/T;
        const double s2=s*s,s3=s2*s,s4=s3*s,s5=s4*s;
        Targets result;
        for(const auto& [id,p]:a) {
            const Eigen::Vector2d delta=b.at(id)-p;
            if(derivative==0)result[id]=p+(10*s3-15*s4+6*s5)*delta+T*(s-6*s3+8*s4-3*s5)*v.at(id);
            else if(derivative==1)result[id]=30*s2*(1-s)*(1-s)*delta/T+(1-18*s2+32*s3-15*s4)*v.at(id);
            else result[id]=(60*s-180*s2+120*s3)*delta/(T*T)+(-36*s+96*s2-60*s3)*v.at(id)/T;
        }
        return result;
    }
};
} // namespace gf
