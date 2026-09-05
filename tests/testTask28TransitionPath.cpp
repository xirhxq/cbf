#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task28TransitionPath.hpp"
#include "grand_finale/Task26ExternalReconstruction.hpp"

namespace {
auto endpoints(const gf::Task20DagLatticeContract& c) {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    return gf::task20LiftTargets(c,fixed,gf::task26CompactFronts(c,fixed)).targets;
}
}

TEST_CASE("Task28 complete contract derives layers and preserves every endpoint role") {
    const auto old=gf::task25DagContractFromCode(0), goal=gf::task25DagContractFromCode(12);
    auto a=endpoints(old),b=endpoints(goal);
    gf::Task28LayerPath path(goal,a,b);
    CHECK(path.layerCount()==4);
    CHECK(path.depths().at(1)==1);CHECK(path.depths().at(14)==4);
    for (const auto& [id,p]:a) {
        CHECK((path.evaluate(0).at(id)-p).norm()==0);
        CHECK((path.evaluate(1).at(id)-b.at(id)).norm()==0);
        CHECK((path.evaluate(-1).at(id)-p).norm()==0);
        CHECK((path.evaluate(2).at(id)-b.at(id)).norm()==0);
    }
    CHECK((path.evaluate(.125).at(1)-a.at(1)).norm()==0);
    CHECK((path.evaluate(.125).at(14)-.5*(a.at(14)+b.at(14))).norm()<1e-9);
    for (double h:{0.,.25,.5,.75,1.}) {
        const auto l=path.evaluate(h-1e-7),r=path.evaluate(h+1e-7);
        for (const auto& [id,p]:l) CHECK((p-r.at(id)).norm()<1e-8);
    }
    a.erase(1);CHECK_THROWS(gf::Task28LayerPath(goal,a,b));
}

TEST_CASE("Task28 geometry covers all 14 mobile and fixed pairs and commutes with a rigid frame") {
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto a=endpoints(old),b=endpoints(goal);gf::Task28LayerPath path(goal,a,b);
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    double minimum=1e9;
    for (int k=0;k<=10000;++k) {
        const auto q=path.evaluate(k/10000.);
        for (auto i=q.begin();i!=q.end();++i) {
            for (auto j=std::next(i);j!=q.end();++j) minimum=std::min(minimum,(i->second-j->second).norm());
            for (const auto& [id,p]:fixed) minimum=std::min(minimum,(i->second-p).norm());
        }
    }
    // Independent scalar oracle uses analytic stationary points, not this grid.
    double analytic=1e9;
    for (std::size_t layer=0;layer<path.layerCount();++layer) {
        auto begin=path.evaluate(double(layer)/path.layerCount()),end=path.evaluate(double(layer+1)/path.layerCount());
        for (const auto& [id,p]:fixed) {begin[id]=p;end[id]=p;}
        for (const auto& [id,p]:a) for (const auto& [other,q]:begin) if (other>id) {
            const Eigen::Vector2d r=begin.at(id)-q,dr=end.at(id)-end.at(other)-r;
            const double h=dr.squaredNorm()>0?std::clamp(-r.dot(dr)/dr.squaredNorm(),0.,1.):0.;
            analytic=std::min(analytic,(r+h*dr).norm());
        }
    }
    CHECK(analytic==doctest::Approx(12.9613649733).epsilon(1e-10));
    CHECK(minimum>=analytic-1e-9);
    CHECK(minimum-analytic<.01); // Sampling is an upper approximation, not an exact minimizer.
    std::cout<<"TASK28_SCALAR_ALL_PHASE_MIN_M "<<std::setprecision(16)<<analytic<<'\n';
    auto c=a,d=b;const Eigen::Rotation2Dd rotation(.71);const Eigen::Vector2d offset(113,-27);
    for (auto& [id,p]:c) p=rotation*p+offset;
    for (auto& [id,p]:d) p=rotation*p+offset;
    gf::Task28LayerPath moved(goal,c,d);
    for (double h:{.07,.32,.71,.94}) for (const auto& [id,p]:path.evaluate(h))
        CHECK((moved.evaluate(h).at(id)-(rotation*p+offset)).norm()<1e-9);
}

TEST_CASE("Task28 layer path has no Pinball member-number dependency") {
    for (int code:{0,11,12,13,2}) {
        const auto c=gf::task25DagContractFromCode(code);const auto a=endpoints(c);
        gf::Task28LayerPath same(c,a,a);
        for (double h:{0.,.1,.4,.9,1.}) for (const auto& [id,p]:a)
            CHECK((same.evaluate(h).at(id)-p).norm()<1e-9);
    }
}
