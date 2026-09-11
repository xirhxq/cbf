#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task32ContractionPath.hpp"

TEST_CASE("Shared contraction phase separates a labelled head-on exchange and preserves center") {
    const gf::Task32ContractionPath path({{1,{-1,0}},{2,{1,0}}},{{1,{1,0}},{2,{-1,0}}});
    const auto middle=path.evaluate(.5);
    CHECK((middle.at(1)-Eigen::Vector2d(0,1)).norm()<1e-12);
    CHECK((middle.at(2)-Eigen::Vector2d(0,-1)).norm()<1e-12);
    CHECK((middle.at(1)+middle.at(2)).norm()<1e-12);
    CHECK((middle.at(1)-middle.at(2)).norm()>1.);
}

TEST_CASE("External contraction rejects incomplete identities and nonfinite geometry") {
    using P=gf::Task32ContractionPath;
    const P::Targets a{{1,{0,0}},{2,{2,0}}};
    CHECK_THROWS(P({},{}));
    CHECK_THROWS(P(a,{{1,{1,0}}}));
    CHECK_THROWS(P(a,{{1,{1,0}},{3,{2,0}}}));
    auto bad=a;bad.at(1).x()=std::numeric_limits<double>::infinity();
    CHECK_THROWS(P(bad,a));
    CHECK_THROWS(P(a,bad));
    const P path(a,a);
    CHECK_THROWS(path.evaluate(std::numeric_limits<double>::quiet_NaN()));
    CHECK_THROWS(path.evaluate(std::numeric_limits<double>::infinity()));
    CHECK(path.evaluate(-1).at(1)==a.at(1));
    CHECK(path.evaluate(2).at(2)==a.at(2));
}

TEST_CASE("All fourteen labels preserve endpoints center and equivariance under one common phase") {
    using P=gf::Task32ContractionPath;
    P::Targets a,b,ar,br;
    Eigen::Matrix2d rotation;rotation<<0,-1,1,0;
    const Eigen::Vector2d offset(500,-200);
    for(int i=1;i<=14;++i) {
        a[i]={100.*i,40.*i*i};b[i]={70.*i+20,300.+90.*i};
        ar[100+i]=rotation*a.at(i)+offset;br[100+i]=rotation*b.at(i)+offset;
    }
    const P path(a,b),transformed(ar,br),stationary(a,a);
    for(int k=0;k<=100;++k) {
        const double s=k/100.;const auto q=path.evaluate(s),qr=transformed.evaluate(s);
        Eigen::Vector2d centroid=Eigen::Vector2d::Zero(),expected=Eigen::Vector2d::Zero();
        for(int i=1;i<=14;++i) {
            CHECK((path.evaluate(0).at(i)-a.at(i)).norm()==0);
            CHECK((path.evaluate(1).at(i)-b.at(i)).norm()==0);
            CHECK((qr.at(100+i)-rotation*q.at(i)-offset).norm()<1e-9);
            CHECK((stationary.evaluate(s).at(i)-a.at(i)).norm()<1e-9);
            centroid+=q.at(i);expected+=(1-s)*a.at(i)+s*b.at(i);
        }
        CHECK((centroid-expected).norm()<1e-8);
    }
    const double dt=1e-4,u=dt/60.,s=u*u*(3-2*u);
    for(int i=1;i<=14;++i)CHECK((path.evaluate(s).at(i)-a.at(i)).norm()/dt<1e-3);
}
