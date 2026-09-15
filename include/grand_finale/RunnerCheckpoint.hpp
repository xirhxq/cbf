#pragma once
#include "grand_finale/ReconstructionCheckpoint.hpp"
#include "grand_finale/CheckpointFile.hpp"
#include "grand_finale/Task19ProductionBaseline.hpp"
#include <cstdlib>

namespace gf {
class RunnerCheckpoint {
    using Json=checkpoint::Json;
    using Fields=checkpoint::Fields;
    bool enabled_=false;
    std::filesystem::path directory_;
    Json identity_,loaded_,future_,ancestry_=Json::object();
    bool future_applied_=false;
    std::size_t interval_=100,seek_tick_=0;
    std::optional<std::size_t> last_written_;
public:
    static std::size_t exactTickValue(const Json& value){
        if(!value.is_number_integer()||value<Json(0)||value>Json(1000000000ULL))
            throw std::invalid_argument("checkpoint tick must be a nonnegative bounded integer");
        return value.get<std::size_t>();
    }
    static std::size_t controlTick(double time,double dt){
        if(!std::isfinite(time)||time<0||!std::isfinite(dt)||dt<=0||time/dt>1.e9)throw std::invalid_argument("invalid resume time");
        const auto tick=static_cast<std::size_t>(std::llround(time/dt));
        if(std::abs(time-tick*dt)>1.e-8)throw std::invalid_argument("resume requires an exact control boundary, no interpolation");return tick;
    }
    static std::size_t precedingTick(const std::vector<std::size_t>& ticks,std::size_t target){
        std::optional<std::size_t> out;for(auto t:ticks)if(t<=target&&(!out||t>*out))out=t;
        if(!out)throw std::invalid_argument("no preceding complete checkpoint");return *out;
    }
    static std::vector<std::string> originalArguments(int argc,char** argv){
        std::vector<std::string> out;for(int k=0;k<argc;++k)out.emplace_back(argv[k]);return out;
    }
    explicit RunnerCheckpoint(const std::vector<std::string>& argv){
        const char* spec=std::getenv("GRAND_FINALE_CHECKPOINT_SPEC");if(!spec)return;
        if(std::string(spec).empty())throw std::invalid_argument("empty checkpoint specification");
        if(std::getenv("GRAND_FINALE_TASK20_CVT_ABLATION"))throw std::invalid_argument("unregistered environment-controlled search ablation cannot be checkpointed");
        std::ifstream in(spec);Json option;if(!in)throw std::runtime_error("checkpoint specification missing");in>>option;
        const std::set<std::string> allowed{"schema","directory","interval_ticks","resume_file","seek_tick","future_requests"};
        for(const auto& item:option.items())if(!allowed.count(item.key()))throw std::invalid_argument("unknown checkpoint option");
        if(option.at("schema")!="grand-finale-checkpoint-options-v1")throw std::invalid_argument("checkpoint option schema");
        directory_=option.at("directory").get<std::string>();interval_=exactTickValue(option.value("interval_ticks",Json(100)));
        if(!directory_.is_absolute()||!std::filesystem::is_directory(directory_)||interval_==0)throw std::invalid_argument("checkpoint output directory/interval");
        Json args=Json::array(),files=Json::object();
        for(std::size_t k=1;k<argv.size();++k){if(k==2||k==3||k==4||k==6)continue;std::string value=argv[k];args.push_back(value);
            const auto eq=value.find('=');const auto path=eq==std::string::npos?value:value.substr(eq+1);
            if(std::filesystem::is_regular_file(path))files[path]=CheckpointFile::fileSha256(path);}
        if(argv.empty())throw std::invalid_argument("missing original binary identity");
        identity_={{"binary_sha256",CheckpointFile::fileSha256(argv[0])},{"scientific_argv",args},{"input_sha256",files}};
        if(option.contains("resume_file")){
            const std::filesystem::path path=option.at("resume_file").get<std::string>();loaded_=CheckpointFile::read(path);
            if(loaded_.at("schema")!="grand-finale-runner-checkpoint-v1"||loaded_.at("identity")!=identity_)throw std::invalid_argument("runner checkpoint binary/config mismatch");
            const auto tick=exactTickValue(loaded_.at("tick"));seek_tick_=exactTickValue(option.value("seek_tick",Json(tick)));
            if(seek_tick_<tick)throw std::invalid_argument("checkpoint is later than requested resume tick");
            ancestry_={{"parent_checkpoint",path.string()},{"parent_checkpoint_sha256",CheckpointFile::fileSha256(path)},
                {"parent_tick",tick},{"requested_tick",seek_tick_},{"kind","exact-continuation"},
                {"parent_ancestry",loaded_.at("ancestry")},
                {"boundary","shared prefix; not an independent noise realization"}};
        }else if(option.contains("seek_tick")||option.contains("future_requests"))throw std::invalid_argument("seek/branch requires a preceding checkpoint");
        if(option.contains("future_requests")) {
            future_=option.at("future_requests");
            if(!future_.is_object()||!future_.contains("target_modes"))throw std::invalid_argument("future request schema");
            for(const auto& item:future_.items())if(item.key()!="target_modes"&&item.key()!="first_request_s")throw std::invalid_argument("future requests cannot change scientific configuration");
            for(const auto& code:future_.at("target_modes"))if(!code.is_number_integer())throw std::invalid_argument("integer mode identity required");
            if(!future_.at("target_modes").is_array())throw std::invalid_argument("future modes must be an array");
        }
        enabled_=true;
    }
    bool enabled()const{return enabled_;}
    bool restoring()const{return !loaded_.is_null();}
    const Json& identity()const{return identity_;}
    const Json& loaded()const{return loaded_;}
    const Json& ancestry()const{return ancestry_;}
    std::size_t seekTick()const{return seek_tick_;}
    void applyFutureAtBoundary(std::size_t tick,Task26ExternalReconstructor* r){
        if(future_.is_null()||future_applied_||tick!=seek_tick_)return;
        if(!r)throw std::invalid_argument("no external request coordinator to branch");
        std::optional<double> first;if(future_.contains("first_request_s"))first=future_.at("first_request_s").get<double>();
        ancestry_["branch"]=ReconstructionCheckpoint::replaceFutureRequests(*r,future_.at("target_modes").get<std::vector<int>>(),first);
        ancestry_["kind"]="future-request-branch";future_applied_=true;
    }
    bool shouldSave(std::size_t tick,bool force=false)const{return enabled_&&(!last_written_||*last_written_!=tick)&&(force||tick%interval_==0||tick==seek_tick_);}
    static Json captureEngine(Swarm& s,GrandFinaleSwarmAdapter& a,Task10p11hSimpleCoverageController& c,Task26ExternalReconstructor* r,const Json& id){
        return r?ReconstructionCheckpoint::capture(s,a,c,*r,id):FullStateCheckpoint::capture(s,a,c,id);
    }
    static void restoreEngine(Swarm& s,GrandFinaleSwarmAdapter& a,Task10p11hSimpleCoverageController& c,Task26ExternalReconstructor* r,const Json& value,const Json& id){
        if(r)ReconstructionCheckpoint::restore(s,a,c,*r,value,id);else FullStateCheckpoint::restore(s,a,c,value,id);
    }
    void save(std::size_t tick,double runtime,const Json& engine,const Json& metrics,bool terminal){
        if(!enabled_)return;
        Json payload={{"schema","grand-finale-runner-checkpoint-v1"},{"identity",identity_},{"tick",tick},
            {"runtime_s",runtime},{"engine",engine},{"metrics",metrics},{"terminal",terminal},{"ancestry",ancestry_}};
        const auto path=directory_/("tick-"+std::to_string(tick)+".cbor.gz");CheckpointFile::writeNew(path,payload);last_written_=tick;
        const auto receipt=directory_/("tick-"+std::to_string(tick)+".json");
        if(std::filesystem::exists(receipt))throw std::runtime_error("checkpoint receipt exists");
        CheckpointFile::writeJsonNew(receipt,Json({{"schema","checkpoint-index-entry-v1"},{"tick",tick},{"runtime_s",runtime},
            {"file",path.string()},{"bytes",std::filesystem::file_size(path)},{"sha256",CheckpointFile::fileSha256(path)},
            {"terminal",terminal},{"identity",identity_},{"ancestry",ancestry_}}));
    }
    static void efficiency(Fields& f,Task19EfficiencyAccumulator& v){
#define F(x) f(#x,v.x)
        F(valid_cells_);F(dt_s_);F(observations_);F(j_uncovered_cell_seconds_);F(t50_tick_);F(t95_tick_);F(t99_tick_);F(t100_tick_);
#undef F
    }
    static void profile(Fields& f,Task10p11ComputeProfile& v){
        std::map<Task10p11ComputePhase,std::vector<std::pair<double,bool>>> samples;
        if(!f.loading)for(const auto& [phase,items]:v.samples_)for(const auto& item:items)samples[phase].push_back({item.seconds,item.steady});
        f("samples",samples);
        if(f.loading){v.samples_.clear();for(const auto& [phase,items]:samples)for(const auto& item:items)v.samples_[phase].push_back({item.first,item.second});}
    }
};
} // namespace gf
