#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task13UnifiedCoveragePolicy.hpp"

namespace {

gf::FrontierCell cell(int x,int y) {
    return {x,y,{10.0*x+5.0,10.0*y+5.0}};
}

std::map<gf::NodeId,Eigen::Vector2d> fixed() {
    return {{100,{1200.0,-50.0}},{101,{1500.0,-50.0}},
            {102,{1800.0,-50.0}}};
}

}  // namespace

TEST_CASE("Independent C++ full-map scan matches the H2 Python oracle") {
    const auto squads=gf::task13UnifiedCoverageSquads();
    const auto fixed_positions=fixed();
    gf::Task13UnifiedCoverageConfig config;
    config.minimum_half_width_m=7.0;
    config.fan_ratio=0.0075;

    std::size_t service_a=0,service_b=0,only_a=0,only_b=0,both=0;
    std::size_t witness_count=0,no_service=0;
    double capability_reference=0.0;
    double capability_separation=std::numeric_limits<double>::infinity();
    double capability_fim=std::numeric_limits<double>::infinity();

    for (int x=0;x<300;++x) for (int y=0;y<300;++y) {
        const auto current=cell(x,y);
        std::array<bool,2> serviced{false,false};
        double best_reference=std::numeric_limits<double>::infinity();
        double best_separation=0.0;
        double best_fim=0.0;
        for (std::size_t squad_index=0;squad_index<squads.size();++squad_index)
            for (gf::NodeId member:squads[squad_index].members) {
                const auto witness=gf::task13TaperedWitness(
                    squads[squad_index],member,current,fixed_positions,config);
                if (!witness.has_value()) continue;
                serviced[squad_index]=true;
                ++witness_count;
                best_reference=std::min(
                    best_reference,witness->maximum_reference_edge_m);
                best_separation=std::max(
                    best_separation,witness->minimum_target_separation_m);
                best_fim=std::max(best_fim,witness->nominal_fim_proxy);
            }
        service_a+=serviced[0];
        service_b+=serviced[1];
        only_a+=serviced[0]&&!serviced[1];
        only_b+=serviced[1]&&!serviced[0];
        both+=serviced[0]&&serviced[1];
        no_service+=!serviced[0]&&!serviced[1];
        if (serviced[0]||serviced[1]) {
            capability_reference=std::max(capability_reference,best_reference);
            capability_separation=std::min(capability_separation,best_separation);
            capability_fim=std::min(capability_fim,best_fim);
        }
    }

    CHECK(service_a==84775);
    CHECK(service_b==84775);
    CHECK(only_a==5225);
    CHECK(only_b==5225);
    CHECK(both==79550);
    CHECK(no_service==0);
    CHECK(witness_count==578188);
    CHECK(capability_reference==doctest::Approx(849.2870188838726).epsilon(1e-12));
    CHECK(capability_separation==doctest::Approx(14.820910567181649).epsilon(1e-12));
    CHECK(capability_fim==doctest::Approx(0.0004957316924231499).epsilon(1e-11));
}

TEST_CASE("Independent C++ full-map scan matches service-pose H2 oracle") {
    const auto squads=gf::task13UnifiedCoverageSquads();
    const auto fixed_positions=fixed();
    gf::Task13UnifiedCoverageConfig config;
    config.certified_service_standoff_m=350.0;
    std::size_t service_a=0,service_b=0,only_a=0,only_b=0,both=0;
    std::size_t witness_count=0,no_service=0;
    double capability_reference=0.0;
    double capability_separation=std::numeric_limits<double>::infinity();
    double capability_fim=std::numeric_limits<double>::infinity();
    for (int x=0;x<300;++x) for (int y=0;y<300;++y) {
        const auto current=cell(x,y);
        std::array<bool,2> serviced{false,false};
        double best_reference=std::numeric_limits<double>::infinity();
        double best_separation=0.0,best_fim=0.0;
        for (std::size_t squad_index=0;squad_index<squads.size();++squad_index)
            for (gf::NodeId member:squads[squad_index].members) {
                const auto witness=gf::task13TaperedWitness(
                    squads[squad_index],member,current,fixed_positions,config);
                if (!witness.has_value()) continue;
                serviced[squad_index]=true;
                ++witness_count;
                best_reference=std::min(best_reference,
                    witness->maximum_reference_edge_m);
                best_separation=std::max(best_separation,
                    witness->minimum_target_separation_m);
                best_fim=std::max(best_fim,witness->nominal_fim_proxy);
            }
        service_a+=serviced[0]; service_b+=serviced[1];
        only_a+=serviced[0]&&!serviced[1];
        only_b+=serviced[1]&&!serviced[0];
        both+=serviced[0]&&serviced[1];
        no_service+=!serviced[0]&&!serviced[1];
        if (serviced[0]||serviced[1]) {
            capability_reference=std::max(capability_reference,best_reference);
            capability_separation=std::min(capability_separation,best_separation);
            capability_fim=std::min(capability_fim,best_fim);
        }
    }
    CHECK(service_a==88786);
    CHECK(service_b==88786);
    CHECK(only_a==1214);
    CHECK(only_b==1214);
    CHECK(both==87572);
    CHECK(no_service==0);
    CHECK(witness_count==649348);
    CHECK(capability_reference==doctest::Approx(766.9566265006717).epsilon(1e-12));
    CHECK(capability_separation==doctest::Approx(20.036734772733684).epsilon(1e-12));
    CHECK(capability_fim==doctest::Approx(0.000553059397243866).epsilon(1e-11));
}
