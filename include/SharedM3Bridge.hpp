#pragma once
#include "SharedM1Consumer.hpp"
#include "cbf/CanoPcgNominal.hpp"
#include "M3NominalTransaction.hpp"
#include <poll.h>
#include <chrono>
namespace shared_m3 {
inline constexpr const char* coreSHA="03bf370c3f763f8f68cd7a7d928fa91e20b617b115ff28ef8432cd36330cacc1";
inline constexpr const char* producerSHA="b6d2ca88c702c56d479dc1fa41076bca7993336a757bcaa691f6221c6c55d714";
inline constexpr const char* protocolSHA="0cf04a474a028dab9a4856edaeeb8d8795f2e8354bbc67f36e4e32a059c001df";
inline void validateMethod(const json& cfg,const std::string& method){
 canonical::schemaNeed(method=="M3","M3 own scientific consumer scope");json common=cfg;common.erase("cano-pcg");common["tvt-experiment"]["cano-pcg"]=false;shared_m1::validateMethod(common,"M1");
 canonical::schemaNeed(cfg.at("tvt-experiment").at("cano-pcg").is_boolean()&&cfg.at("tvt-experiment").at("cano-pcg").get<bool>(),"explicit M3 nominal only");const auto& p=cfg.at("cano-pcg");canonical::keys(p,{"protocol-sha256","core-source-sha256","producer-source-sha256","consumer-source-sha256","identity","arithmetic-decimal-digits","transport-codec-version","transport-layout","nominal-rule","shared700-interface"});canonical::schemaNeed(p.at("protocol-sha256")==protocolSHA&&p.at("core-source-sha256")==coreSHA&&p.at("producer-source-sha256")==producerSHA&&p.at("consumer-source-sha256")=="4d2751c96c33cfc7c47ee6242f5aaf63b75493471ce8ad96af271c2b411aef67"&&p.at("identity")==tvt_cano::identity&&p.at("arithmetic-decimal-digits")==50&&p.at("transport-codec-version")==2&&p.at("transport-layout")==2&&p.at("nominal-rule")=="minus5*g/max(1-per-metre,norm(g)),yaw0"&&p.at("shared700-interface")=="shared-M3-snapshot-sidecar-v1","complete frozen M3 science/codec policy");
}
class Bridge {
 int request=-1,response=-1;json context;std::uint64_t next=0;std::chrono::steady_clock::time_point deadline;
 void ready(int fd,short events){canonical::schemaNeed(std::chrono::steady_clock::now()<deadline,"M3 sidecar IPC deadline");pollfd p{fd,events,0};int status=::poll(&p,1,250);if(status<0&&errno==EINTR)return;canonical::schemaNeed(status>=0&&!(p.revents&(POLLERR|POLLNVAL)),"M3 bounded FIFO poll");}
 void send(const json& value){const std::string raw=value.dump()+"\n";canonical::schemaNeed(raw.size()<=65536,"whole M3 request64KiB");std::size_t off=0;while(off<raw.size()){ready(request,POLLOUT);auto n=::write(request,raw.data()+off,raw.size()-off);if(n<0&&(errno==EAGAIN||errno==EINTR))continue;canonical::schemaNeed(n>0,"M3 request full write");off+=static_cast<std::size_t>(n);}}
 json receive(){std::string raw;while(true){ready(response,POLLIN);char c;auto n=::read(response,&c,1);if(n<0&&(errno==EAGAIN||errno==EINTR))continue;canonical::schemaNeed(n==1,"M3 response whole line");raw.push_back(c);canonical::schemaNeed(raw.size()<=65536,"whole M3 response64KiB");if(c=='\n')return shared_m1::strictParse(raw);}}
public:
 explicit Bridge(const json& gate):context(gate.at("binding")),deadline(std::chrono::steady_clock::now()+std::chrono::seconds(1800)){
  canonical::schemaNeed(gate.at("method")=="M3"&&gate.at("M3").at("binding")==context,"own M3 sidecar binding");const auto& paths=gate.at("M3").at("IPC_paths");canonical::schemaNeed(paths.size()==2,"two declared M3 FIFOs");for(const auto& path:paths)canonical::schemaNeed(!std::filesystem::is_symlink(path.get<std::string>())&&std::filesystem::is_fifo(path.get<std::string>()),"declared own producer FIFO");request=::open(paths[0].get<std::string>().c_str(),O_RDWR|O_NONBLOCK);response=::open(paths[1].get<std::string>().c_str(),O_RDWR|O_NONBLOCK);canonical::schemaNeed(request>=0&&response>=0,"M3 FIFO endpoints open");
 }
 ~Bridge(){if(request>=0)::close(request);if(response>=0)::close(response);}
 json install(Swarm& swarm,std::uint64_t frame,const json& payload){
  canonical::schemaNeed(frame==next&&frame<700,"strict single M3 nominal request per frame");json bases=json::array(),formations=json::array();for(const auto& p:swarm.config.at("bases"))bases.push_back({p.at(0).get<double>(),p.at(1).get<double>()});for(const auto& r:swarm.robots){auto f=r->myFormation;formations.push_back({{"id",r->id},{"baseIds",f.at("baseIds")},{"anchorIds",f.at("anchorIds")}});}
  json req={{"schema","shared-M3-snapshot-request-v1"},{"kind","frame"},{"binding",context},{"frame",frame},{"payload",payload},{"payload_sha256",shared_m1::nativeDigest(payload)},{"bases",bases},{"formations",formations},{"protocol_sha256",protocolSHA},{"core_source_sha256",coreSHA},{"producer_source_sha256",producerSHA}};send(req);const auto sidecar=receive();canonical::keys(sidecar,{"schema","binding","frame","request_sha256","payload_sha256","packet_sha256","packet"});canonical::schemaNeed(sidecar.at("schema")=="shared-M3-nominal-sidecar-v1"&&canonical::nativeBitsEqual(sidecar.at("binding"),context),"whole M3 sidecar scope");canonical::nativeInt(sidecar.at("frame"),frame,frame);canonical::schemaNeed(sidecar.at("request_sha256")==shared_m1::nativeDigest(req)&&sidecar.at("payload_sha256")==shared_m1::nativeDigest(payload)&&sidecar.at("packet_sha256")==shared_m1::nativeDigest(sidecar.at("packet")),"whole current frame/request/payload/packet identities");
  json envelope=payload;envelope["cano_pcg"]=sidecar.at("packet");commitNominals(swarm.robots,frame,[&]{return tvt_cano::parse(envelope,frame,swarm.config.at("cano-pcg"));});
  ++next;return {{"event","M3_nominal_installed_pre_target_QP_hold"},{"frame",frame},{"snapshot_request",req},{"frozen_formations",formations},{"request_sha256",shared_m1::nativeDigest(req)},{"packet_sha256",sidecar.at("packet_sha256")},{"packet",sidecar.at("packet")},{"atomic_all14_install",true},{"before_target_QP_hold",true}};
 }
 void finish(){canonical::schemaNeed(next==700,"M3 service finish only after700 nominal responses");send({{"schema","shared-M3-snapshot-request-v1"},{"kind","finish"},{"binding",context},{"frames",700}});const auto ack=receive();canonical::schemaNeed(canonical::nativeBitsEqual(ack,json{{"kind","finish"},{"binding",context},{"frames",700}}),"complete one producer service finish acknowledgement");}
};
}

namespace shared_m3 {
using shared_m1::evidenceNeed;using shared_m1::nativeDigest;using shared_m1::ConsumerEvidenceError;
class M3EventLog {
    int fd=-1;std::size_t spent=0;json context;
public:
    explicit M3EventLog(const std::string& path,bool guardedFIFO){
        evidenceNeed(guardedFIFO&&!std::filesystem::is_symlink(path)&&std::filesystem::is_fifo(path),"reviewed event FIFO required");fd=::open(path.c_str(),O_WRONLY);evidenceNeed(fd>=0,"guarded event FIFO open failed");
    }
    void setContext(const json& value){context=value;}
    ~M3EventLog(){if(fd>=0)::close(fd);}
    void append(const json& value){try{json record=value;record["evidence_context"]=context;record["hash_codec"]="native-binary64-tree-compact-JSON-v1";record["evidence_body_sha256"]=nativeDigest(record);const std::string data=record.dump()+"\n";evidenceNeed(data.size()<=4u*1024u*1024u&&spent+data.size()<=2ull*1024ull*1024ull*1024ull,"registered consumer record/cumulative cap beforewrite");spent+=data.size();std::size_t offset=0;while(offset<data.size()){auto n=::write(fd,data.data()+offset,data.size()-offset);if(n<0&&errno==EINTR)continue;evidenceNeed(n>0,"consumer evidence full write");offset+=static_cast<std::size_t>(n);}evidenceNeed(::fsync(fd)==0||errno==EINVAL,"FIFO full write, parent persists/fsyncs exact stream");}catch(const ConsumerEvidenceError&){throw;}catch(const std::exception& error){throw ConsumerEvidenceError(std::string("evidence serialization/hash/persistence: ")+std::string(error.what()).substr(0,16384));}}
};
}
