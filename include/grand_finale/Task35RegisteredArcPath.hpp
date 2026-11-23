#pragma once
#include "grand_finale/Types.hpp"
#include <Eigen/Dense>
#include <json.hpp>
#include <map>
#include <set>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace gf {
// Immutable registered spatial path. No plant, measurement or task mutation.
// This clock is a nominal Euclidean speed budget, not a safety certificate.
class Task35RegisteredArcPath {
public:
    using Targets=std::map<NodeId,Eigen::Vector2d>;
    Task35RegisteredArcPath(const nlohmann::json& asset,std::set<NodeId> members,Targets fixed,double budget=29.9)
        :asset_(asset),members_(std::move(members)),fixed_(std::move(fixed)),budget_(budget) {
        if(members_.empty()||!std::isfinite(budget_)||budget_<=0)throw std::invalid_argument("registered_arc_domain");
        for(const auto& [id,p]:fixed_)if(members_.count(id)||!p.allFinite())throw std::invalid_argument("registered_arc_fixed_domain");
        Targets previous=readTargets(asset_.at("start"));
        const auto bridge=readTargets(asset_.at("bridge")),goal=readTargets(asset_.at("goal"));
        int previous_leg=0;bool seen_first=false,seen_second=false;
        for(const auto& entry:asset_.at("stages")) {
            Segment s;s.leg=entry.at("leg").get<int>();
            if(s.leg<0||s.leg>1||s.leg<previous_leg||(!seen_first&&s.leg!=0))throw std::invalid_argument("registered_arc_leg_order");
            if(s.leg==1&&!seen_second)requireSame(previous,bridge);
            seen_first|=s.leg==0;seen_second|=s.leg==1;previous_leg=s.leg;
            for(const auto& item:entry.at("coeff").items()) {
                auto& coeff=s.coeff[std::stoi(item.key())];
                for(const auto& v:item.value())coeff.emplace_back(v.at(0).get<double>(),v.at(1).get<double>());
                if(coeff.empty()||coeff.size()>32)throw std::invalid_argument("registered_arc_polynomial_domain");
                for(const auto& v:coeff)if(!v.allFinite())throw std::invalid_argument("registered_arc_nonfinite");
            }
            if(s.coeff.size()!=members_.size()+fixed_.size())throw std::invalid_argument("registered_arc_members");
            for(const auto& [id,p]:fixed_) {
                const auto& c=s.coeff.at(id);
                if(c[0]!=p)throw std::invalid_argument("registered_arc_anchor_changed");
                for(std::size_t k=1;k<c.size();++k)if(c[k]!=Eigen::Vector2d::Zero())throw std::invalid_argument("registered_arc_anchor_motion");
            }
            Targets start,end;
            for(auto id:members_){start[id]=evaluate(s.coeff.at(id),0,false);end[id]=evaluate(s.coeff.at(id),1,false);}
            requireSame(previous,start);previous=end;
            for(auto id:members_)if(s.coeff.at(id).size()>2)s.linear=false;
            s.cumulative.push_back(0);
            if(s.linear){s.length=rate(s,0);s.cumulative.push_back(s.length);}
            else for(int k=0;k<bins;++k) {
                s.length+=integral(s,double(k)/bins,double(k+1)/bins);
                s.cumulative.push_back(s.length);
            }
            s.duration=s.length/budget_;
            (s.leg==0?first_:second_)+=s.duration;
            segments_.push_back(std::move(s));
        }
        if(!seen_first||!seen_second||!std::isfinite(first_+second_)||first_+second_<=0)throw std::invalid_argument("registered_arc_incomplete_legs");
        requireSame(previous,goal);
    }
    double firstDuration()const{return first_;}
    double secondDuration()const{return second_;}
    double speedBudget()const{return budget_;}
    const nlohmann::json& asset()const{return asset_;}
    Targets start()const{return readTargets(asset_.at("start"));}
    Targets bridge()const{return readTargets(asset_.at("bridge"));}
    Targets goal()const{return readTargets(asset_.at("goal"));}
    nlohmann::json snapshot()const {
        nlohmann::json fixed=nlohmann::json::object();
        for(const auto& [id,p]:fixed_)fixed[std::to_string(id)]={p.x(),p.y()};
        return {{"schema","task35-registered-arclength-v1"},{"asset",asset_},{"members",members_},
            {"fixed",fixed},{"budget",budget_},{"first_s",first_},{"second_s",second_},
            {"quadrature","gauss8-fixed512-bisection44-v1"}};
    }
    static Task35RegisteredArcPath restore(const nlohmann::json& saved) {
        Targets fixed;for(const auto& v:saved.at("fixed").items())
            fixed[std::stoi(v.key())]={v.value().at(0).get<double>(),v.value().at(1).get<double>()};
        Task35RegisteredArcPath out(saved.at("asset"),saved.at("members").get<std::set<NodeId>>(),fixed,saved.at("budget").get<double>());
        if(out.snapshot()!=saved)throw std::invalid_argument("registered_arc_checkpoint_identity");
        return out;
    }
    Targets position(double time)const {
        const auto [index,s]=locate(time);Targets out;
        for(auto id:members_) {
            const auto& c=segments_.at(index).coeff.at(id);
            out[id]=evaluate(c,s,false);
        }return out;
    }
    Targets velocity(double time)const {
        const auto [index,s]=locate(time);Targets out;
        const auto& stage=segments_.at(index);
        for(auto id:members_) {
            out[id]=Eigen::Vector2d::Zero();const auto& c=stage.coeff.at(id);
            const double m=rate(stage,s);
            if(time<first_+second_&&m>0)out[id]=evaluate(c,s,true)*(budget_/m);
        }return out;
    }
private:
    struct Segment {int leg=0;std::map<NodeId,std::vector<Eigen::Vector2d>> coeff;double length=0,duration=0;
        bool linear=true;std::vector<double> cumulative;};
    static constexpr int bins=512;
    nlohmann::json asset_;std::set<NodeId> members_;Targets fixed_;double budget_,first_=0,second_=0;
    std::vector<Segment> segments_;
    Targets readTargets(const nlohmann::json& j)const {
        if(j.size()!=members_.size()+fixed_.size())throw std::invalid_argument("registered_arc_endpoint_domain");
        Targets out;
        for(auto id:members_) {
            const auto& v=j.at(std::to_string(id));out[id]={v.at(0).get<double>(),v.at(1).get<double>()};
            if(!out[id].allFinite())throw std::invalid_argument("registered_arc_endpoint_nonfinite");
        }
        for(const auto& [id,p]:fixed_) {
            const auto& v=j.at(std::to_string(id));
            if(v.at(0).get<double>()!=p.x()||v.at(1).get<double>()!=p.y())throw std::invalid_argument("registered_arc_endpoint_anchor");
        }return out;
    }
    static void requireSame(const Targets& a,const Targets& b) {
        if(a.size()!=b.size())throw std::invalid_argument("registered_arc_continuity_domain");
        for(const auto& [id,p]:a)if((p-b.at(id)).norm()>1e-8)throw std::invalid_argument("registered_arc_discontinuous");
    }
    std::pair<std::size_t,double> locate(double time)const {
        if(!std::isfinite(time)||time<0||segments_.empty())throw std::invalid_argument("registered_arc_time");
        double start=0;
        for(std::size_t i=0;i<segments_.size();++i) {
            const auto& s=segments_[i];
            if(time<start+s.duration) {
                const double distance=(time-start)*budget_;
                if(s.linear)return {i,distance/s.length};
                auto upper=std::upper_bound(s.cumulative.begin(),s.cumulative.end(),distance);
                const std::size_t k=std::min(std::size_t(bins-1),std::size_t(upper-s.cumulative.begin()-1));
                const double a=double(k)/bins;double lo=a,hi=double(k+1)/bins;
                for(int j=0;j<44;++j) {
                    const double mid=(lo+hi)/2;
                    if(s.cumulative[k]+integral(s,a,mid)<distance)lo=mid;else hi=mid;
                }return {i,(lo+hi)/2};
            }
            start+=s.duration;
        }return {segments_.size()-1,1};
    }
    static Eigen::Vector2d evaluate(const std::vector<Eigen::Vector2d>& c,double s,bool derivative) {
        Eigen::Vector2d v=Eigen::Vector2d::Zero();
        for(std::size_t k=c.size();k-->std::size_t(derivative);)v=v*s+c[k]*(derivative?double(k):1.);
        return v;
    }
    double rate(const Segment& stage,double phase)const {
        double m=0;for(auto id:members_)m=std::max(m,evaluate(stage.coeff.at(id),phase,true).norm());return m;
    }
    double integral(const Segment& stage,double a,double b)const {
        // Fixed Gauss-8 on fixed bins. Numerical timing only; never changes the spatial polynomial.
        static constexpr double x[]={.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363};
        static constexpr double w[]={.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763};
        const double mid=(a+b)/2,half=(b-a)/2;double sum=0;
        for(int i=0;i<4;++i)sum+=w[i]*(rate(stage,mid-half*x[i])+rate(stage,mid+half*x[i]));
        return half*sum;
    }
};
} // namespace gf
