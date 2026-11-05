#ifndef CBF_GUROBI_PYTHON_HPP
#define CBF_GUROBI_PYTHON_HPP
#include "OptimiserBase.hpp"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <unistd.h>
// Thin transport of the unchanged convex QP to Linux gurobipy 12.
class GurobiPython:public OptimiserBase {
 int n=0,u=0,aux=0;json problem,answer;
 inline static std::uint64_t serial=0;
 public:
 GurobiPython(json&s):OptimiserBase(s){}
 void clear()override{problem=json::object();answer=json::object();aux=0;}
 void setAuxiliaryVariableCount(int k)override{aux=k;}
 void start(int total,int controls)override{n=total;u=controls;problem={{"n",n},{"u",u},{"auxiliary",aux},{"k_delta",k_delta},{"rows",json::array()}};}
 void setObjective(Eigen::VectorXd&v)override{problem["nominal"]=json::array();for(int j=0;j<u;++j)problem["nominal"].push_back(v[j]);}
 void addLinearConstraint(Eigen::VectorXd a,double b)override{json row=json::array();for(int j=0;j<n;++j)row.push_back(j<a.size()?a[j]:0.);problem["rows"].push_back({{"a",row},{"rhs",b}});}
 Eigen::VectorXd solve()override{
  const char*env=std::getenv("CBF_QP_BRIDGE");if(!env)throw std::runtime_error("CBF_QP_BRIDGE required");const std::string d=env;
  problem["serial"]=++serial;{std::ofstream f(d+"/qp-request.tmp");f<<problem.dump()<<'\n';}std::rename((d+"/qp-request.tmp").c_str(),(d+"/qp-request.json").c_str());
  const auto began=std::chrono::steady_clock::now();
  while(access((d+"/qp-response.json").c_str(),F_OK)!=0){if(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>30)throw std::runtime_error("Gurobi bridge timeout");usleep(1000);}
  {std::ifstream f(d+"/qp-response.json");f>>answer;}std::remove((d+"/qp-response.json").c_str());
  if(answer.at("serial")!=serial||answer.value("status","")!="optimal")throw std::runtime_error("Gurobi QP failed: "+answer.dump());
  Eigen::VectorXd x(n);for(int j=0;j<n;++j)x[j]=answer.at("x").at(j);return x;
 }
 void write(std::string p)override{std::ofstream f(p);f<<problem.dump(2);}
 double getObjectiveValue()const override{return answer.value("objective",0.);}
 json getStatus()const override{return answer;}
};
#endif
