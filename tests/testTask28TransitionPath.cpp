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
    double analytic=1e9,maximum_reference=0;
    for (std::size_t layer=0;layer<path.layerCount();++layer) {
        auto begin=path.evaluate(double(layer)/path.layerCount()),end=path.evaluate(double(layer+1)/path.layerCount());
        for (const auto& [id,p]:fixed) {begin[id]=p;end[id]=p;}
        for (const auto& edge:goal.reference_edges) {
            maximum_reference=std::max(maximum_reference,(begin.at(edge.reference)-begin.at(edge.owner)).norm());
            maximum_reference=std::max(maximum_reference,(end.at(edge.reference)-end.at(edge.owner)).norm());
        }
        for (const auto& [id,p]:a) for (const auto& [other,q]:begin) if (other>id) {
            const Eigen::Vector2d r=begin.at(id)-q,dr=end.at(id)-end.at(other)-r;
            const double h=dr.squaredNorm()>0?std::clamp(-r.dot(dr)/dr.squaredNorm(),0.,1.):0.;
            analytic=std::min(analytic,(r+h*dr).norm());
        }
    }
    CHECK(analytic==doctest::Approx(12.9613649733).epsilon(1e-10));
    CHECK(minimum>=analytic-1e-9);
    CHECK(minimum-analytic<.01); // Sampling is an upper approximation, not an exact minimizer.
    CHECK(maximum_reference==doctest::Approx(790.0206928648051).epsilon(1e-11));
    std::cout<<"TASK28_SCALAR_ALL_PHASE_MIN_M "<<std::setprecision(16)<<analytic<<'\n';
    std::cout<<"TASK28_SCALAR_ALL_PHASE_MAX_NOMINAL_REFERENCE_M "<<maximum_reference<<'\n';
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

TEST_CASE("Task28 common-finish path preserves layer starts without early terminal hold") {
    const auto old=gf::task25DagContractFromCode(0),goal=gf::task25DagContractFromCode(12);
    const auto a=endpoints(old),b=endpoints(goal);
    gf::Task28LayerPath path(goal,a,b,true);
    CHECK((path.evaluate(.5).at(14)-b.at(14)).norm()>1);
    CHECK((path.evaluate(.5).at(1)-a.at(1)).norm()==0);
    for (const auto& [id,p]:a) {
        CHECK((path.evaluate(0).at(id)-p).norm()==0);
        CHECK((path.evaluate(1).at(id)-b.at(id)).norm()==0);
    }
    for (double h:{.25,.5,.75}) {
        const auto l=path.evaluate(h-1e-6),c=path.evaluate(h),r=path.evaluate(h+1e-6);
        for (const auto& [id,p]:a) CHECK(((c.at(id)-l.at(id))-(r.at(id)-c.at(id))).norm()/1e-6<.1);
    }
    // Independent scalar grid: not a continuous certificate; compare with
    // Python's piecewise-polynomial stationary points at dh=1e-5 resolution.
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    double minimum=1e9,maximum_reference=0;
    for (int tick=0;tick<=100000;++tick) {
        const double h=tick/100000.;auto q=fixed;
        for (const auto& [id,p]:a) {
            const double start=1-path.depths().at(id)/double(path.layerCount());
            const double u=std::clamp((h-start)/(1-start),0.,1.),s=u*u*(3-2*u);
            q[id]=p+s*(b.at(id)-p);
        }
        for (const auto& [id,p]:a) for (const auto& [other,r]:q) if (other>id)
            minimum=std::min(minimum,(q.at(id)-r).norm());
        for (const auto& e:goal.reference_edges)
            maximum_reference=std::max(maximum_reference,(q.at(e.reference)-q.at(e.owner)).norm());
        if (tick%1000==0) for (const auto& [id,p]:path.evaluate(h)) CHECK((p-q.at(id)).norm()<1e-9);
    }
    CHECK(minimum==doctest::Approx(11.2179554165421).epsilon(1e-5));
    CHECK(maximum_reference==doctest::Approx(754.75922466535).epsilon(1e-10));
    std::cout<<"TASK28_COMMON_FINISH_SCALAR_MIN "<<minimum<<" MAX_NOMINAL_REF "<<maximum_reference<<'\n';
}
