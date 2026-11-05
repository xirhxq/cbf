#ifndef SHARED_M1_CONSUMER_HPP
#define SHARED_M1_CONSUMER_HPP
#include "CanonicalReplayInterface.hpp"
#include <CommonCrypto/CommonDigest.h>
#include <cfenv>
#include <filesystem>
#include <fcntl.h>
#include <unistd.h>
namespace shared_m1 {
inline void evidenceNeed(bool ok,const char* label){if(!ok)throw ConsumerEvidenceError(label);}
using canonical::schemaNeed;using canonical::nativeInt;using canonical::nativeReal;using canonical::nativeBitsEqual;
inline std::string digest(const std::string& data){
    schemaNeed(data.size()<=16u*1024u*1024u,"registered input hash maximum16MiB");unsigned char out[CC_SHA256_DIGEST_LENGTH];
    schemaNeed(CC_SHA256(data.data(),static_cast<CC_LONG>(data.size()),out)!=nullptr,"SHA256 returned");
    const char* hex="0123456789abcdef";std::string value;for(auto x:out){value+=hex[x>>4];value+=hex[x&15];}return value;
}
inline json nativeBitTree(const json& value){
    if(value.is_number_float()){nativeReal(value);static_assert(sizeof(double)==sizeof(std::uint64_t),"binary64 codec");const double number=value.get<double>();std::uint64_t bits;std::memcpy(&bits,&number,sizeof(bits));const char* hex="0123456789abcdef";std::string text;for(int shift=60;shift>=0;shift-=4)text+=hex[(bits>>shift)&15];return {{"binary64_be_hex",text}};}
    if(value.is_array()){json out=json::array();for(const auto& child:value)out.push_back(nativeBitTree(child));return out;}
    if(value.is_object()){json out=json::object();for(auto it=value.begin();it!=value.end();++it)out[it.key()]=nativeBitTree(it.value());return out;}
    return value;
}
inline std::string nativeDigest(const json& value){return digest(nativeBitTree(value).dump());}
inline std::string bytes(const std::filesystem::path& path,std::size_t cap){
    schemaNeed(!std::filesystem::is_symlink(path)&&std::filesystem::is_regular_file(path),"read-only regular registered input");
    const auto count=std::filesystem::file_size(path);schemaNeed(count<=cap,"registered input byte cap before read");
    std::ifstream in(path,std::ios::binary);std::string data(static_cast<std::size_t>(count),'\0');in.read(data.data(),data.size());
    schemaNeed(in.gcount()==static_cast<std::streamsize>(count)&&in.peek()==EOF,"exact whole registered input bytes");return data;
}
inline json strictParse(const std::string& data){
    std::vector<std::set<std::string>> objectKeys;
    auto callback=[&](int,json::parse_event_t event,json& value){
        if(event==json::parse_event_t::object_start)objectKeys.emplace_back();
        else if(event==json::parse_event_t::key){schemaNeed(!objectKeys.empty(),"object key stack");schemaNeed(objectKeys.back().insert(value.get<std::string>()).second,"duplicate JSON key refused");}
        else if(event==json::parse_event_t::object_end){schemaNeed(!objectKeys.empty(),"object end stack");objectKeys.pop_back();}
        return true;
    };
    json value=json::parse(data,callback);schemaNeed(objectKeys.empty(),"complete strict JSON parse");return value;
}
inline json readBound(const std::filesystem::path& path,const std::string& expected,std::size_t cap){const auto data=bytes(path,cap);schemaNeed(digest(data)==expected,"consumed exact input SHA256");return strictParse(data);}
inline const std::set<std::string>& required(){static const std::set<std::string> names={"plan-at-generation.json","generation-release.json","exact-dispatch.json","source-files.json","source.tar","environment.json","position-template.json","base-config.json","generation-entry.py","capture-binary","capture-config.json","position-recipe-observed.json","capture-child-release.json","capture-spawn.json","capture-exit.json","native-capture.json.constructor.json","native-capture.json","physical-native-bit-tree.json","common-native-replay-view.json","latent700.json","frame0-common-payload.json","generation-provenance.json","capture.stdout","capture.stderr"};return names;}
inline void validateMethod(const json& cfg,const std::string& method){
    schemaNeed(method=="M1"||method=="M1-origin","only two adapted methods");canonical::validateCommonSettings(cfg);tvt2026::validateLocalSymmetricExecution(cfg);
    const auto& t=cfg.at("tvt-experiment");schemaNeed(t.at("target-origin")== (method=="M1"?"base-midpoint":"first-base"),"frozen method origin");
    schemaNeed(t.at("implementation-identity")=="symmetric-l1-main-target-v3"&&t.at("target-protocol")=="continuous-halfplane-grid-v3-exact-dyadic","original target implementation");
    nativeReal(cfg.at("execute").at("time-step"));schemaNeed(nativeBitsEqual(cfg.at("execute").at("time-step"),json(.5)),"original .5s time step");nativeInt(cfg.at("execute").at("time-total"),350,350);
    schemaNeed(!cfg.at("execute").value("terminate-on-coverage-complete",false),"no early result selection");
    schemaNeed(t.at("row-mode")=="symmetric-l1"&&t.at("controller-identity")=="symmetric-l1-main-v1"&&t.at("information-contract")=="estimated-task-v1","same original local QP/controller");
    schemaNeed(!t.value("m2-exact-finite-centroid",false)&&!t.value("cano-pcg",false),"no other method module");
}
inline void validateLatent(const json& value,int seed){
    canonical::keys(value,{"schema","seed","error_seed","generator_class","frames","independent_of_trajectory","terminal350_draw"});
    schemaNeed(value.at("schema")=="shared-latent700-v1"&&value.at("generator_class")=="PCG64","stored original noise class");nativeInt(value.at("seed"),seed,seed);nativeInt(value.at("error_seed"),seed+100000,seed+100000);
    schemaNeed(value.at("independent_of_trajectory").is_boolean()&&value.at("independent_of_trajectory").get<bool>()&&value.at("terminal350_draw").is_boolean()&&!value.at("terminal350_draw").get<bool>(),"no new terminal350 estimate");
    const auto& frames=value.at("frames");schemaNeed(frames.is_array()&&frames.size()==700,"exact700 stored frames");
    for(int k=0;k<700;++k){const auto& f=frames.at(k);canonical::keys(f,{"frame_index","time_s","errors_xy_id_order"});nativeInt(f.at("frame_index"),k,k);nativeReal(f.at("time_s"));schemaNeed(nativeBitsEqual(f.at("time_s"),json(.5*k)),"exact0..349.5 input times");const auto& errors=f.at("errors_xy_id_order");schemaNeed(errors.is_array()&&errors.size()==14,"ordered14 latent vectors");for(const auto& error:errors){canonical::nativeVector(error,2);schemaNeed(std::hypot(error.at(0).get<double>(),error.at(1).get<double>())<=.5+1e-12,"unchanged .5m disk definition");}}
}
inline json commonView(const json& receipt){
    json view={{"schema","shared-common-native-replay-view-v1"},{"nodes",json::array()}};
    for(const char* key:{"world","bases","grid","true_evaluation_map_phase","true_valid_hex","true_visited_hex","reset_history","charge_policy"})view[key]=receipt.at(key);
    for(auto node:receipt.at("nodes")){node["common_settings_copy"]=canonical::commonSettingsProjection(node.at("settings_copy"));node.erase("settings_copy");view["nodes"].push_back(node);}return view;
}
inline json physicalRequest(Swarm& swarm,std::uint64_t frame,const std::string& bundleSHA){
    schemaNeed(frame<700&&swarm.robots.size()==14,"physical frame scope");json robots=json::array();
    for(std::size_t i=0;i<14;++i){auto& robot=*swarm.robots.at(i);schemaNeed(robot.id==static_cast<int>(i)+1&&robot.runtime==.5*frame,"same actual frame/ID/time");
        const auto state=robot.model->state2Json();for(const auto& value:state)nativeReal(value);
        robots.push_back({{"id",robot.id},{"state",state},{"X_x_y_battery_yaw",canonical::vector(robot.model->getX())},{"u_before",robot.model->control2Json()},{"velocity",canonical::vector(robot.model->getVelocity())},{"runtime_s",robot.runtime},{"formation",robot.myFormation}});
    }
    return {{"schema","shared-M1-physical-request-v1"},{"frame_index",frame},{"time_s",.5*frame},{"shared_bundle_sha256",bundleSHA},{"phase","pre-common-estimates/sensor/target/QP/hold"},{"robots",robots}};
}
inline json payload(const json& request,const json& latent,std::uint64_t frame){
    schemaNeed(frame<700&&request.at("frame_index").get<std::uint64_t>()==frame,"current input frame");schemaNeed(std::fegetround()==FE_TONEAREST,"original binary64 nearest addition");
    json robots=json::array();const auto& errors=latent.at("frames").at(frame).at("errors_xy_id_order");
    for(int i=0;i<14;++i){const auto& r=request.at("robots").at(i);nativeInt(r.at("id"),i+1,i+1);const auto& state=r.at("state");const double x=state.at("x").get<double>()+errors.at(i).at(0).get<double>(),y=state.at("y").get<double>()+errors.at(i).at(1).get<double>();schemaNeed(std::isfinite(x)&&std::isfinite(y),"finite truth plus same stored noise");robots.push_back({{"id",i+1},{"estimate",{x,y}},{"epsilon",.5}});}
    json value={{"frame_index",frame},{"robots",robots}};tvt2026::parseEstimatorSnapshot(value,frame,14);return value;
}
inline json installedPayload(Swarm& swarm,std::uint64_t frame){json value={{"frame_index",frame},{"robots",json::array()}};for(auto& r:swarm.robots)value["robots"].push_back({{"id",r->id},{"estimate",{r->estimatorPosition.x,r->estimatorPosition.y}},{"epsilon",r->estimatorEpsilon}});return value;}
inline json estimatedMap(Swarm& swarm){json nodes=json::array();for(auto& r:swarm.robots)nodes.push_back({{"id",r->id},{"valid_hex",canonical::bitmap(r->gridWorld.valid)},{"visited_hex",canonical::bitmap(r->gridWorld.vis)}});return {{"schema","shared-estimated-sensor-map-v1"},{"phase","post-estimated-sensor/pre-target-QP-hold"},{"nodes",nodes}};}
class EventLog {
    int fd=-1;std::size_t spent=0;json context;
public:
    explicit EventLog(const std::string& path,bool guardedFIFO){
        evidenceNeed(guardedFIFO&&!std::filesystem::is_symlink(path)&&std::filesystem::is_fifo(path),"reviewed event FIFO required");fd=::open(path.c_str(),O_WRONLY);evidenceNeed(fd>=0,"guarded event FIFO open failed");
    }
    void setContext(const json& value){context=value;}
    ~EventLog(){if(fd>=0)::close(fd);}
    void append(const json& value){try{json record=value;record["evidence_context"]=context;record["hash_codec"]="native-binary64-tree-compact-JSON-v1";record["evidence_body_sha256"]=nativeDigest(record);const std::string data=record.dump()+"\n";evidenceNeed(data.size()<=4u*1024u*1024u&&spent+data.size()<=1024u*1024u*1024u,"registered consumer record/cumulative cap beforewrite");spent+=data.size();std::size_t offset=0;while(offset<data.size()){auto n=::write(fd,data.data()+offset,data.size()-offset);if(n<0&&errno==EINTR)continue;evidenceNeed(n>0,"consumer evidence full write");offset+=static_cast<std::size_t>(n);}evidenceNeed(::fsync(fd)==0||errno==EINVAL,"FIFO full write, parent persists/fsyncs exact stream");}catch(const ConsumerEvidenceError&){throw;}catch(const std::exception& error){throw ConsumerEvidenceError(std::string("evidence serialization/hash/persistence: ")+std::string(error.what()).substr(0,16384));}}
};
}
#endif
