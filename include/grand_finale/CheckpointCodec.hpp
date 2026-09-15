#pragma once

#include "json.hpp"
#include <Eigen/Core>
#include <array>
#include <cstring>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <type_traits>

namespace gf::checkpoint {
using Json=nlohmann::json;
template<class T,class Enable=void> struct Codec;
template<class T>struct Empty {static T make(){return T{};}};
template<class A,class B>struct Empty<std::pair<A,B>>{static std::pair<A,B> make(){return {Empty<A>::make(),Empty<B>::make()};}};
template<class T> Json encode(const T& v){return Codec<T>::encode(v);}
template<class T> void decode(const Json& j,T& v){Codec<T>::decode(j,v);}

// Float bits, including signed zero and diagnostic infinities, survive JSON
// and CBOR. No decimal rounding, null-for-infinity or native object dumps.
template<> struct Codec<double> {
    static Json encode(double v){std::uint64_t bits;std::memcpy(&bits,&v,8);return bits;}
    static void decode(const Json& j,double& v){
        if(!j.is_number_unsigned())throw std::invalid_argument("checkpoint float bits");
        const auto bits=j.get<std::uint64_t>();std::memcpy(&v,&bits,8);
    }
};
template<class T> struct Codec<T,std::enable_if_t<std::is_integral_v<T>>> {
    static Json encode(T v){return v;}
    static void decode(const Json& j,T& v){
        if constexpr(std::is_same_v<T,bool>){if(!j.is_boolean())throw std::invalid_argument("checkpoint bool");}
        else if(!j.is_number_integer())throw std::invalid_argument("checkpoint integer");
        v=j.get<T>();if(Json(v)!=j)throw std::invalid_argument("checkpoint integer overflow");
    }
};
template<class T> struct Codec<T,std::enable_if_t<std::is_enum_v<T>>> {
    static Json encode(T v){return checkpoint::encode(static_cast<std::underlying_type_t<T>>(v));}
    static void decode(const Json& j,T& v){std::underlying_type_t<T> n;checkpoint::decode(j,n);v=static_cast<T>(n);}
};
template<> struct Codec<std::string>{static Json encode(const std::string& v){return v;}static void decode(const Json& j,std::string& v){v=j.get<std::string>();}};
template<> struct Codec<Json>{static Json encode(const Json& v){return v;}static void decode(const Json& j,Json& v){v=j;}};
template<class T> struct Codec<std::optional<T>> {
    static Json encode(const std::optional<T>& v){return v?checkpoint::encode(*v):Json(nullptr);}
    static void decode(const Json& j,std::optional<T>& v){if(j.is_null())v.reset();else{T x=Empty<T>::make();checkpoint::decode(j,x);v=std::move(x);}}
};
template<class T> struct Codec<std::vector<T>> {
    static Json encode(const std::vector<T>& v){Json j=Json::array();for(const auto& x:v)j.push_back(checkpoint::encode<T>(x));return j;}
    static void decode(const Json& j,std::vector<T>& v){if(!j.is_array())throw std::invalid_argument("checkpoint array");std::vector<T> tmp;tmp.reserve(j.size());for(const auto& x:j){T item=Empty<T>::make();checkpoint::decode(x,item);tmp.push_back(std::move(item));}v=std::move(tmp);}
};
template<> struct Codec<std::vector<bool>> {
    static Json encode(const std::vector<bool>& v){std::string s((v.size()+3)/4,'0');constexpr char hex[]="0123456789abcdef";for(std::size_t i=0;i<s.size();++i){unsigned n=0;for(unsigned k=0;k<4&&4*i+k<v.size();++k)n|=unsigned(v[4*i+k])<<k;s[i]=hex[n];}return Json{{"size",v.size()},{"hex",s}};}
    static void decode(const Json& j,std::vector<bool>& v){const auto n=j.at("size").get<std::size_t>();const auto s=j.at("hex").get<std::string>();if(n>100000000||s.size()!=(n+3)/4)throw std::invalid_argument("checkpoint bitset size");std::vector<bool> tmp(n);for(std::size_t i=0;i<s.size();++i){unsigned b=s[i]>='0'&&s[i]<='9'?s[i]-'0':s[i]>='a'&&s[i]<='f'?s[i]-'a'+10:16;if(b>15)throw std::invalid_argument("checkpoint bitset hex");for(unsigned k=0;k<4;++k){if(4*i+k<n)tmp[4*i+k]=(b>>k)&1;else if((b>>k)&1)throw std::invalid_argument("checkpoint padding");}}v=std::move(tmp);}
};
template<class K,class V> struct Codec<std::map<K,V>> {
    static Json encode(const std::map<K,V>& v){Json j=Json::array();for(const auto& [k,x]:v)j.push_back(Json::array({checkpoint::encode(k),checkpoint::encode(x)}));return j;}
    static void decode(const Json& j,std::map<K,V>& v){if(!j.is_array())throw std::invalid_argument("checkpoint map");std::map<K,V> tmp;for(const auto& p:j){if(!p.is_array()||p.size()!=2)throw std::invalid_argument("checkpoint map entry");K k{};V x{};checkpoint::decode(p[0],k);checkpoint::decode(p[1],x);if(!tmp.emplace(std::move(k),std::move(x)).second)throw std::invalid_argument("checkpoint duplicate key");}v=std::move(tmp);}
};
template<class T> struct Codec<std::set<T>> {
    static Json encode(const std::set<T>& v){return checkpoint::encode(std::vector<T>(v.begin(),v.end()));}
    static void decode(const Json& j,std::set<T>& v){std::vector<T> tmp;checkpoint::decode(j,tmp);v={tmp.begin(),tmp.end()};if(v.size()!=tmp.size())throw std::invalid_argument("checkpoint duplicate set item");}
};
template<class A,class B> struct Codec<std::pair<A,B>> {
    static Json encode(const std::pair<A,B>& v){return Json::array({checkpoint::encode(v.first),checkpoint::encode(v.second)});}
    static void decode(const Json& j,std::pair<A,B>& v){if(!j.is_array()||j.size()!=2)throw std::invalid_argument("checkpoint pair");checkpoint::decode(j[0],v.first);checkpoint::decode(j[1],v.second);}
};
template<class... T> struct Codec<std::tuple<T...>> {
    template<std::size_t... I>static Json put(const std::tuple<T...>& v,std::index_sequence<I...>){return Json::array({checkpoint::encode(std::get<I>(v))...});}
    template<std::size_t... I>static void get(const Json& j,std::tuple<T...>& v,std::index_sequence<I...>){(checkpoint::decode(j[I],std::get<I>(v)),...);}
    static Json encode(const std::tuple<T...>& v){return put(v,std::index_sequence_for<T...>{});}
    static void decode(const Json& j,std::tuple<T...>& v){if(!j.is_array()||j.size()!=sizeof...(T))throw std::invalid_argument("checkpoint tuple");get(j,v,std::index_sequence_for<T...>{});}
};
template<class T,std::size_t N> struct Codec<std::array<T,N>> {
    static Json encode(const std::array<T,N>& v){Json j=Json::array();for(const auto& x:v)j.push_back(checkpoint::encode(x));return j;}
    static void decode(const Json& j,std::array<T,N>& v){if(!j.is_array()||j.size()!=N)throw std::invalid_argument("checkpoint fixed array");for(std::size_t i=0;i<N;++i)checkpoint::decode(j[i],v[i]);}
};
template<class S,int R,int C,int O,int MR,int MC> struct Codec<Eigen::Matrix<S,R,C,O,MR,MC>> {
    using T=Eigen::Matrix<S,R,C,O,MR,MC>;
    static Json encode(const T& v){Json d=Json::array();for(int r=0;r<v.rows();++r)for(int c=0;c<v.cols();++c)d.push_back(checkpoint::encode(v(r,c)));return Json{{"rows",v.rows()},{"cols",v.cols()},{"data",d}};}
    static void decode(const Json& j,T& v){const int r=j.at("rows").get<int>(),c=j.at("cols").get<int>();if(r<0||c<0||r>10000||c>10000||(R!=Eigen::Dynamic&&r!=R)||(C!=Eigen::Dynamic&&c!=C)||j.at("data").size()!=std::size_t(r)*c)throw std::invalid_argument("checkpoint matrix shape");v.resize(r,c);for(int i=0;i<r;++i)for(int k=0;k<c;++k)checkpoint::decode(j.at("data")[i*c+k],v(i,k));}
};

struct Fields {
    Json& object;bool loading;std::set<std::string> names;
    template<class T>void operator()(const char* name,T& value){if(!names.insert(name).second)throw std::logic_error("duplicate checkpoint field");if(loading)checkpoint::decode(object.at(name),value);else object[name]=checkpoint::encode(value);}
    void finish(){if(!object.is_object()||object.size()!=names.size())throw std::invalid_argument("checkpoint field set mismatch");}
};
template<class T> struct RecordCodec {
    static Json encode(const T& value){T copy=value;Json j=Json::object();Fields f{j,false,{}};Codec<T>::fields(f,copy);f.finish();return j;}
    static void decode(const Json& j,T& value){Json copy=j;T result=Empty<T>::make();Fields f{copy,true,{}};Codec<T>::fields(f,result);f.finish();value=std::move(result);}
};
} // namespace gf::checkpoint
