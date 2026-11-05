#ifndef SHARED_CANONICAL_CAPTURE_AUDIT_HPP
#define SHARED_CANONICAL_CAPTURE_AUDIT_HPP
#include "utils.h"
#include <chrono>
#include <cstdint>
namespace canonical {
inline bool active=false;
inline std::uint64_t robotConstructorAttempts=0,robotConstructorReturns=0;
inline std::uint64_t environmentFactoryAttempts=0,environmentStartsCompleted=0;
inline std::uint64_t repoRandAttempts=0,repoRandReturns=0;
inline json counters() {return {{"robot_constructor_attempts",robotConstructorAttempts},{"robot_constructor_returns",robotConstructorReturns},
 {"repo_environment_factory_attempts",environmentFactoryAttempts},{"environment_start_completed",environmentStartsCompleted},
 {"repo_rand_attempts",repoRandAttempts},{"repo_rand_returns",repoRandReturns},{"opaque_process_RNG_calls",nullptr},
 {"optimize_API_calls",0},{"trajectory_calls",0},{"target_calls",0},{"hold_calls",0},{"producer_PCG_calls",0},
 {"zero_scope_proof","CANONICAL_CAPTURE_ONLY compile guards reject these entry points before scientific work"}};}
inline void forbid(const char* operation) {
#ifdef CANONICAL_CAPTURE_ONLY
    throw std::runtime_error(std::string("capture-only source forbids ")+operation);
#else
    (void)operation;
#endif
}
inline void save(const std::string& path,const json& value) {std::ofstream f(path);f<<value.dump(2)<<'\n';f.flush();if(!f)throw std::runtime_error("capture receipt persistence failed");}
inline json matrix(const Eigen::MatrixXd& m) {
    if(!m.allFinite())throw std::runtime_error("capture nonfinite matrix");
    json out=json::array();for(int i=0;i<m.rows();++i){json row=json::array();for(int j=0;j<m.cols();++j)row.push_back(m(i,j));out.push_back(row);}return out;
}
inline json vector(const Eigen::VectorXd& v) {
    if(!v.allFinite())throw std::runtime_error("capture nonfinite vector");json a=json::array();for(int i=0;i<v.size();++i)a.push_back(v[i]);return a;
}
inline void gate(const json& g) {
    if(g.at("schema")!="canonical-capture-release-v1" || g.at("scope")!="one-canonical-native-capture"
       || !g.at("allowed").is_boolean() || !g.at("allowed").get<bool>()
       || !g.at("release").is_boolean() || !g.at("release").get<bool>()
       || !g.at("independent_source_pass").is_boolean() || !g.at("independent_source_pass").get<bool>()
       || !g.at("generation_budget_adopted").is_boolean() || !g.at("generation_budget_adopted").get<bool>())
        throw std::runtime_error("capture/generation scope remains closed");
}
}
#endif
