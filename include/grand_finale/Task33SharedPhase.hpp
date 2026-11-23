#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>
#include "grand_finale/Types.hpp"

namespace gf {
// Reconstruction-only clock for a declared piecewise polynomial path of
// degree <=3. This is not a generic sampled-path speed certificate. Knots
// must contain every polynomial join. The original path is never replaced.
class Task33SharedPhase {
    friend class ReconstructionCheckpoint;
public:
    using Targets=std::map<NodeId,Eigen::Vector2d>;
    using Path=std::function<Targets(double)>;
    Task33SharedPhase(const Path& path,std::vector<double> knots)
        :knots_(std::move(knots)) {
        if(knots_.size()<2||knots_.front()!=0||knots_.back()!=1)
            throw std::invalid_argument("shared-phase knots must span [0,1]");
        for(std::size_t k=1;k<knots_.size();++k) {
            const double a=knots_[k-1],b=knots_[k],h=b-a;
            if(!std::isfinite(a)||!std::isfinite(b)||!(h>0))
                throw std::invalid_argument("invalid shared-phase interval");
            const Targets q0=path(a),q1=path(a+h/3),q2=path(a+2*h/3),q3=path(b);
            if(q0.empty()||q1.size()!=q0.size()||q2.size()!=q0.size()||q3.size()!=q0.size())
                throw std::invalid_argument("inconsistent shared-phase members");
            double bound=0;std::vector<Derivative> segment;
            for(const auto& [id,p0]:q0) {
                if(!q1.count(id)||!q2.count(id)||!q3.count(id)||!p0.allFinite()||
                    !q1.at(id).allFinite()||!q2.at(id).allFinite()||!q3.at(id).allFinite())
                    throw std::invalid_argument("nonfinite shared-phase target");
                // Exact cubic interpolation in Bernstein form. Derivative
                // lies in the convex hull of its three control vectors.
                using V=Eigen::Matrix<long double,2,1>;
                const V p=p0.cast<long double>(),u=q1.at(id).cast<long double>(),
                    v=q2.at(id).cast<long double>(),z=q3.at(id).cast<long double>();
                if((p==u)&&(p==v)&&(p==z))continue;
                const V c1=(-5*p+18*u-9*v+2*z)/6;
                const V c2=(2*p-9*u+18*v-5*z)/6;
                const long double d=3*std::max({(c1-p).norm(),(c2-c1).norm(),(z-c2).norm()})/h;
                const double scale=std::max({1.,p0.norm(),q1.at(id).norm(),q2.at(id).norm(),q3.at(id).norm()});
                const double pad=512*std::numeric_limits<double>::epsilon()*scale/h;
                segment.push_back({{(3*(c1-p)/h).cast<double>(),(3*(c2-c1)/h).cast<double>(),
                    (3*(z-c2)/h).cast<double>()},pad});
                bound=std::max(bound,std::nextafter(static_cast<double>(d)+pad,
                    std::numeric_limits<double>::infinity()));
            }
            bounds_.push_back(bound);
            derivatives_.push_back(std::move(segment));
        }
    }
    double advance(double phase,double dt,double budget) const {
        if(!std::isfinite(phase)||phase<0||phase>1||!std::isfinite(dt)||dt<0||
            !std::isfinite(budget)||budget<=0)
            throw std::invalid_argument("invalid shared-phase clock input");
        double s=phase,remaining=dt;
        if(dt==0)return s;
        while(s<1) {
            const auto it=std::upper_bound(knots_.begin(),knots_.end(),s);
            const std::size_t k=std::distance(knots_.begin(),it)-1;
            const double end=knots_.at(k+1),L=bounds_.at(k);
            if(L==0){s=end;continue;}
            const double cost=(end-s)*L/budget;
            if(cost<=remaining){s=end;remaining-=cost;}
            else {s=std::min(end,s+remaining*budget/L);break;}
            if(remaining<=0)break;
        }
        return s;
    }
    const std::vector<double>& bounds() const {return bounds_;}
    // R2: same polynomial, same common budget; enclose the derivative only
    // over this candidate step rather than using the entire layer's hull.
    // Fixed48 scalar bisections do not select targets or optimise a plant.
    double advanceLocal(double phase,double dt,double budget) const {
        advance(phase,0,budget); // Identical finite/range validation.
        if(!std::isfinite(dt)||dt<0)throw std::invalid_argument("invalid local phase dt");
        if(dt==0)return phase;
        double s=phase,remaining=dt;
        while(s<1) {
            const auto it=std::upper_bound(knots_.begin(),knots_.end(),s);
            const std::size_t k=std::distance(knots_.begin(),it)-1;
            const double end=knots_.at(k+1),L=localBound(k,s,end);
            if(L==0){s=end;continue;}
            const double cost=(end-s)*L/budget;
            if(cost<=remaining){s=end;remaining-=cost;if(remaining<=0)break;continue;}
            double lo=s,hi=end;
            for(int iteration=0;iteration<48;++iteration) {
                const double mid=lo+(hi-lo)/2;
                if((mid-s)*localBound(k,s,mid)<=remaining*budget)lo=mid;
                else hi=mid;
            }
            return lo;
        }
        return s;
    }
private:
    struct Derivative {std::array<Eigen::Vector2d,3> controls;double pad;};
    double localBound(std::size_t k,double start,double end) const {
        const double h=knots_[k+1]-knots_[k];
        const double a=std::clamp((start-knots_[k])/h,0.,1.);
        const double b=std::clamp((end-knots_[k])/h,0.,1.);
        double out=0;
        for(const auto& d:derivatives_[k]) {
            const auto& p=d.controls;
            const Eigen::Vector2d first=(1-a)*(1-a)*p[0]+2*a*(1-a)*p[1]+a*a*p[2];
            const Eigen::Vector2d last=(1-b)*(1-b)*p[0]+2*b*(1-b)*p[1]+b*b*p[2];
            const Eigen::Vector2d middle=first+(b-a)*((1-a)*(p[1]-p[0])+a*(p[2]-p[1]));
            const double L=std::max({first.norm(),middle.norm(),last.norm()})+d.pad;
            if(!std::isfinite(L))throw std::invalid_argument("nonfinite local derivative enclosure");
            out=std::max(out,std::nextafter(L,std::numeric_limits<double>::infinity()));
        }
        return out;
    }
    std::vector<double> knots_,bounds_;
    std::vector<std::vector<Derivative>> derivatives_;
};
} // namespace gf
