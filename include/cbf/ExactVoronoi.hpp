#ifndef CBF_EXACT_VORONOI_HPP
#define CBF_EXACT_VORONOI_HPP
#include "utils.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <cstring>
namespace cbf2026 {
// Exact arithmetic over the binary values actually carried by finite doubles.
// No midpoint rounding and no floating threshold can create an ID tie.
struct Dyadic {
    boost::multiprecision::cpp_int numerator;
    int exponent=0;
    Dyadic()=default;
    Dyadic(boost::multiprecision::cpp_int n,int e):numerator(std::move(n)),exponent(e) {}
    explicit Dyadic(double value) {
        std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));
        const auto fraction=bits&((std::uint64_t(1)<<52)-1);const int biased=(bits>>52)&2047;
        if(biased==2047)throw std::invalid_argument("exact predicate requires finite doubles");
        if(biased==0){numerator=fraction;exponent=-1074;}
        else {numerator=(std::uint64_t(1)<<52)|fraction;exponent=biased-1023-52;}
        if(bits>>63)numerator=-numerator;
        if(numerator==0)exponent=0;
    }
};
inline Dyadic operator+(const Dyadic& a,const Dyadic& b) {
    if(a.numerator==0)return b;if(b.numerator==0)return a;
    const int e=std::min(a.exponent,b.exponent);
    return {(a.numerator<<(a.exponent-e))+(b.numerator<<(b.exponent-e)),e};
}
inline Dyadic operator-(const Dyadic& a,const Dyadic& b) {return a+Dyadic(-b.numerator,b.exponent);}
inline Dyadic operator*(const Dyadic& a,const Dyadic& b) {return {a.numerator*b.numerator,a.exponent+b.exponent};}
struct ExactVoronoiPlane {
    Dyadic ax,ay,c;
    ExactVoronoiPlane(const Point& own,const Point& other) {
        const Dyadic lx(own.x),ly(own.y),jx(other.x),jy(other.y),two(2.);
        ax=two*(jx-lx);ay=two*(jy-ly);c=lx*lx+ly*ly-jx*jx-jy*jy;
    }
    int sign(const Point& point) const {
        const auto value=ax*Dyadic(point.x)+ay*Dyadic(point.y)+c;
        return value.numerator>0?1:value.numerator<0?-1:0;
    }
};
}
#endif
