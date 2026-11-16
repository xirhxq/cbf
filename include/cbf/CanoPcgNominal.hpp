#pragma once
#include "utils.h"
#include <stdexcept>
#include <cmath>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
namespace tvt_cano {
constexpr const char* identity="cano-dopt-pcg50-commonfilter-static-candidate-v3";
using Decimal50=boost::multiprecision::cpp_dec_float_50;
inline Decimal50 residualDecimal(const json& v) {
 if(!v.is_string())throw std::runtime_error("PCG50 full precision residual string required");
 const Decimal50 x(v.get<std::string>());
 if(!boost::math::isfinite(x)||x<0||x>Decimal50("1e-14"))throw std::runtime_error("PCG50 internal residual gate rejected");
 return x;
}
struct NodeNominal { int id; Eigen::Vector3d value; json record; };
inline Eigen::Vector3d normalizedNominal(const json& gradient) {
 if(!gradient.is_array()||gradient.size()!=2)throw std::runtime_error("Cano planar gradient missing");
 const double x=gradient[0].get<double>(),y=gradient[1].get<double>();
 if(!std::isfinite(x)||!std::isfinite(y))throw std::runtime_error("Cano gradient nonfinite");
 const double scale=-5.0/std::max(1.0,std::hypot(x,y));return {scale*x,scale*y,0.};
}
inline std::vector<NodeNominal> parse(const json& payload,std::uint64_t frame,const json& policy) {
 const auto& p=payload.at("cano_pcg");
 if(p.at("schema")!="cano-pcg50-nominal-packet-v2"||p.at("method")!=identity||p.at("frame_index").get<std::uint64_t>()!=frame||p.at("status")!="accepted"||p.at("protocol_sha256")!=policy.at("protocol-sha256")||p.at("core_source_sha256")!=policy.at("core-source-sha256")||p.at("central_values_used_online").get<bool>())throw std::runtime_error("Cano packet identity/frame/eligibility rejected");
 if(p.at("arithmetic_decimal_digits")!=50||p.at("transport_codec_version")!=2||p.at("transport_layout")!=2||p.at("internal_true_residual_gate_decimal")!="1e-14"||p.at("bootstrap_policy")!="fresh tree/RHS mapping for each standalone static case")throw std::runtime_error("PCG50 frozen precision/codec/policy rejected");
 residualDecimal(p.at("internal_joint_relative_frobenius_decimal"));
 if(p.at("spd_collective_pass_counts")!=json::array({7,7}))throw std::runtime_error("Cano collective SPD count rejected");
 if(p.at("components").size()!=2||p.at("nodes").size()!=14||p.at("spd_local_certificates").size()!=14)throw std::runtime_error("Cano incomplete collective packet");
 int componentIndex=0;
 for(const auto& c:p.at("components")) {
  if(!c.at("accepted").get<bool>()||c.at("nodes").size()!=7||c.at("rhs_columns").size()!=14||c.at("iterations_per_rhs").size()!=14||c.at("true_residual_norm_per_rhs").size()!=14)throw std::runtime_error("Cano component incomplete");
  if(c.at("true_residual_norm_per_rhs_decimal").size()!=14)throw std::runtime_error("PCG50 incomplete decimal residuals");
  Decimal50 decimalSum=0;
  for(int k=0;k<14;++k){const auto r=residualDecimal(c.at("true_residual_norm_per_rhs_decimal")[k]);decimalSum+=r*r;}
  const auto decimalAggregate=residualDecimal(c.at("relative_frobenius_residual_decimal"));
  if(abs(decimalAggregate-sqrt(decimalSum/14))>Decimal50("1e-45"))throw std::runtime_error("PCG50 decimal aggregation rejected");
  double sum=0.;for(int k=0;k<7;++k)if(c.at("nodes")[k].get<int>()!=1+7*componentIndex+k)throw std::runtime_error("Cano component ID mapping changed");
  for(int k=0;k<14;++k){int steps=c.at("iterations_per_rhs")[k].get<int>();double r=c.at("true_residual_norm_per_rhs")[k].get<double>();if(steps<1||steps>64||!std::isfinite(r)||r<0||r>1e-8||c.at("rhs_columns")[k][0].get<int>()!=1+7*componentIndex+k/2||c.at("rhs_columns")[k][1]!=(k%2?"y":"x"))throw std::runtime_error("Cano RHS budget/residual/map rejected");sum+=r*r;}
  const double aggregate=c.at("relative_frobenius_residual").get<double>();if(!std::isfinite(aggregate)||aggregate<0||aggregate>1e-8||std::abs(aggregate-std::sqrt(sum/14))>1e-14)throw std::runtime_error("Cano residual aggregation rejected");++componentIndex;
 }
 std::vector<NodeNominal> values;
 for(int i=0;i<14;++i){const auto& c=p.at("spd_local_certificates")[i];double sine=c.at("normalized_cross_absolute").get<double>();if(c.at("tag").get<int>()!=i+1||!c.at("lower_ID_dependency_verified").get<bool>()||!c.at("passes_predeclared_margin").get<bool>()||!std::isfinite(sine)||sine<1e-8||sine>1+1e-12)throw std::runtime_error("Cano structural SPD certificate rejected");const auto& n=p.at("nodes")[i];const auto& e=payload.at("robots")[i];if(n.at("id").get<int>()!=i+1||e.at("id").get<int>()!=i+1||n.at("frame_index").get<std::uint64_t>()!=frame||n.at("estimate")!=e.at("estimate"))throw std::runtime_error("Cano nominal is not bound to common estimate");if(n.at("gradient_decimal").size()!=2)throw std::runtime_error("PCG50 final gradient precision evidence missing");for(int a=0;a<2;++a){if(!n.at("gradient_decimal")[a].is_string())throw std::runtime_error("PCG50 gradient decimal string required");const Decimal50 dg(n.at("gradient_decimal")[a].get<std::string>());if(!boost::math::isfinite(dg))throw std::runtime_error("PCG50 gradient decimal nonfinite");const double converted=dg.convert_to<double>(),supplied=n.at("gradient")[a].get<double>();if(!std::isfinite(converted)||!std::isfinite(supplied)||std::abs(converted-supplied)>1e-15*std::max(1.,std::abs(converted)))throw std::runtime_error("PCG50 final float gradient mismatch");}auto v=normalizedNominal(n.at("gradient"));if(n.at("nominal").size()!=3)throw std::runtime_error("Cano nominal shape");for(int k=0;k<3;++k)if(!std::isfinite(n.at("nominal")[k].get<double>())||std::abs(v[k]-n.at("nominal")[k].get<double>())>1e-12)throw std::runtime_error("Cano normalized nominal mismatch");values.push_back({i+1,v,n});}
 return values;
}
}
