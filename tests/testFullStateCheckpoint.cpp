#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/FullStateCheckpoint.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include "grand_finale/ReconstructionCheckpoint.hpp"
#include "grand_finale/CheckpointFile.hpp"
#include "grand_finale/RunnerCheckpoint.hpp"

static auto continuousFixture(){
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(scenario,gf::SolverProfile::Gurobi);
    auto config=gf::task19ProductionAdapterConfig(0.5,0.0,305003);
    config.target_policy_task18_cbf2026_outer=false;
    config.target_policy_task20_dag_lattice=true;
    config.task32_target_mechanism=2;
    config.distance_range_availability.enabled=true;
    config.distance_range_availability.link_seed=306003;
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(scenario,settings,config);
}

TEST_CASE("runner checkpoint restores a noisy control boundary without changing its successor") {
    auto a=gf::makeTask19ProductionFixture(0.5,0.0,305001);
    REQUIRE(a->adapter.initializeStageZero().initialized);
    for(int k=0;k<3;++k)REQUIRE(a->controller.advance().step.advanced);
    const nlohmann::json identity={{"binary","engineering-same-binary"},{"config","noise-305001"}};
    const auto saved=gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity);
    const auto bytes=nlohmann::json::to_cbor(saved);
    const auto loaded=nlohmann::json::from_cbor(bytes);
    auto b=gf::makeTask19ProductionFixture(0.5,0.0,305001);
    REQUIRE(b->adapter.initializeStageZero().initialized);
    gf::FullStateCheckpoint::restore(b->swarm,b->adapter,b->controller,loaded,identity);
    CHECK(gf::FullStateCheckpoint::capture(b->swarm,b->adapter,b->controller,identity)==saved);
    for(int k=0;k<3;++k) {
        REQUIRE(a->controller.advance().step.advanced);
        REQUIRE(b->controller.advance().step.advanced);
        CHECK(a->controller.lastNominalControls()==b->controller.lastNominalControls());
        CHECK(gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity)==
              gf::FullStateCheckpoint::capture(b->swarm,b->adapter,b->controller,identity));
    }
}

TEST_CASE("C front and an external request resume without restarting their clocks") {
    auto a=continuousFixture();REQUIRE(a->adapter.initializeStageZero().initialized);
    gf::Task26ExternalReconstructor ra(a->adapter,a->controller,"cross-roundtrip-qualified",0.2);
    for(int k=0;k<6;++k){ra.beforeStep();REQUIRE(a->controller.advance().step.advanced);}
    const nlohmann::json identity={{"binary","engineering-same-binary"},{"config","C-distance-305003"}};
    const auto saved=gf::ReconstructionCheckpoint::capture(a->swarm,a->adapter,a->controller,ra,identity);
    auto b=continuousFixture();REQUIRE(b->adapter.initializeStageZero().initialized);
    gf::Task26ExternalReconstructor rb(b->adapter,b->controller,"cross-roundtrip-qualified",0.2);
    gf::ReconstructionCheckpoint::restore(b->swarm,b->adapter,b->controller,rb,
        nlohmann::json::from_cbor(nlohmann::json::to_cbor(saved)),identity);
    CHECK(ra.report()==rb.report());
    for(int k=0;k<8;++k){
        ra.beforeStep();rb.beforeStep();REQUIRE(a->controller.advance().step.advanced);REQUIRE(b->controller.advance().step.advanced);
        CHECK(ra.report()==rb.report());
        CHECK(gf::ReconstructionCheckpoint::capture(a->swarm,a->adapter,a->controller,ra,identity)==
              gf::ReconstructionCheckpoint::capture(b->swarm,b->adapter,b->controller,rb,identity));
    }
}

TEST_CASE("checkpoint files reject corruption and never overwrite an existing snapshot") {
    char directory[]="/tmp/gf-checkpoint-test-XXXXXX";
    REQUIRE(::mkdtemp(directory)!=nullptr);
    const auto path=std::filesystem::path(directory)/"checkpoint.cbor.gz";
    auto a=continuousFixture();REQUIRE(a->adapter.initializeStageZero().initialized);
    REQUIRE(a->controller.advance().step.advanced);
    const nlohmann::json identity={{"binary","same"},{"config","same"}};
    const auto saved=gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity);
    gf::CheckpointFile::writeNew(path,saved);
    CHECK(gf::CheckpointFile::read(path)==saved);
    CHECK_THROWS(gf::CheckpointFile::writeNew(path,saved));
    auto wrong=identity;wrong["binary"]="other";
    CHECK_THROWS(gf::FullStateCheckpoint::restore(a->swarm,a->adapter,a->controller,saved,wrong));
    auto missing=saved;missing["controller"].erase("task32_unit_front_ledger_");
    CHECK_THROWS(gf::FullStateCheckpoint::restore(a->swarm,a->adapter,a->controller,missing,identity));
    CHECK(gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,identity)==saved);
    {std::fstream file(path,std::ios::binary|std::ios::in|std::ios::out);file.seekg(17);char byte=0;file.get(byte);file.seekp(17);file.put(byte^1);}
    CHECK_THROWS(gf::CheckpointFile::read(path));
    // Only this test's newly created temporary files are removed.
    std::filesystem::remove(path);std::filesystem::remove(directory);
}

TEST_CASE("arbitrary-point resume accepts control boundaries but never interpolates a sub-tick state") {
    CHECK(gf::RunnerCheckpoint::controlTick(420.0,0.1)==4200);
    CHECK(gf::RunnerCheckpoint::controlTick(17.3,0.1)==173);
    CHECK_THROWS(gf::RunnerCheckpoint::controlTick(17.35,0.1));
    CHECK_THROWS(gf::RunnerCheckpoint::controlTick(-0.1,0.1));
    CHECK(gf::RunnerCheckpoint::precedingTick({0,100,200,300},273)==200);
    CHECK_THROWS(gf::RunnerCheckpoint::precedingTick({100,200},99));
}

TEST_CASE("future request branching refuses an unregistered protocol without changing its state") {
    auto a=continuousFixture();REQUIRE(a->adapter.initializeStageZero().initialized);
    gf::Task26ExternalReconstructor r(a->adapter,a->controller,"cross-roundtrip-qualified",0.2);
    const nlohmann::json identity={{"binary","test"}};
    const auto before=gf::ReconstructionCheckpoint::capture(a->swarm,a->adapter,a->controller,r,identity);
    CHECK_THROWS(gf::ReconstructionCheckpoint::replaceFutureRequests(r,{11},0.1));
    CHECK(gf::ReconstructionCheckpoint::capture(a->swarm,a->adapter,a->controller,r,identity)==before);
}

TEST_CASE("checkpoint options never silently truncate a fractional control tick") {
    char directory[]="/tmp/gf-checkpoint-options-XXXXXX";REQUIRE(::mkdtemp(directory)!=nullptr);
    const auto path=std::filesystem::path(directory)/"options.json";
    const char* original=std::getenv("GRAND_FINALE_CHECKPOINT_SPEC");
    const std::optional<std::string> previous=original?std::optional<std::string>(original):std::nullopt;
    {std::ofstream out(path);out<<nlohmann::json({{"schema","grand-finale-checkpoint-options-v1"},
        {"directory",directory},{"interval_ticks",1.5}});}
    ::setenv("GRAND_FINALE_CHECKPOINT_SPEC",path.c_str(),1);
    CHECK_THROWS(gf::RunnerCheckpoint(std::vector<std::string>{"/usr/bin/true","origin"}));
    if(previous)::setenv("GRAND_FINALE_CHECKPOINT_SPEC",previous->c_str(),1);else ::unsetenv("GRAND_FINALE_CHECKPOINT_SPEC");
    std::filesystem::remove(path);std::filesystem::remove(directory);
}

TEST_CASE("scientific floating values and control tick types retain exact semantics") {
    const auto positive=gf::checkpoint::encode(0.0),negative=gf::checkpoint::encode(-0.0);
    CHECK(positive!=negative);
    double value=0;gf::checkpoint::decode(negative,value);CHECK(std::signbit(value));
    const double infinity=std::numeric_limits<double>::infinity();
    gf::checkpoint::decode(gf::checkpoint::encode(infinity),value);CHECK(value==infinity);
    CHECK_THROWS(gf::checkpoint::decode(nlohmann::json(2.5),value));
    CHECK(gf::RunnerCheckpoint::exactTickValue(nlohmann::json(4200))==4200);
    CHECK_THROWS(gf::RunnerCheckpoint::exactTickValue(nlohmann::json(4200.5)));
    CHECK_THROWS(gf::RunnerCheckpoint::exactTickValue(nlohmann::json(-1)));
    CHECK_THROWS(gf::RunnerCheckpoint::exactTickValue(nlohmann::json("4200")));
    CHECK_THROWS(gf::RunnerCheckpoint::exactTickValue(nlohmann::json(1000000001ULL)));
}

TEST_CASE("configuration mismatches fail before altering a live fixture") {
    auto a=continuousFixture();REQUIRE(a->adapter.initializeStageZero().initialized);
    const nlohmann::json id={{"binary","fixed-fixture"}};
    const auto original=gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,id);
    auto mismatched=original;
    mismatched["immutable"]["adapter_config"]["range_random_seed"]=123;
    CHECK_THROWS(gf::FullStateCheckpoint::restore(a->swarm,a->adapter,a->controller,mismatched,id));
    CHECK(gf::FullStateCheckpoint::capture(a->swarm,a->adapter,a->controller,id)==original);
}
