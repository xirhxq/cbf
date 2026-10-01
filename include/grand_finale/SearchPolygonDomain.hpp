#pragma once
#include "json.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gf {
// Search-domain input only: no anchor, initial-state, mode or control mutation.
// This version deliberately accepts convex polygons on the frozen 10 m lattice.
inline nlohmann::json searchPolygonSettings(const nlohmann::json& settings,
                                           const nlohmann::json& site) {
    if (site.value("schema",std::string())!="gf-search-polygon-v1" ||
        site.value("cell_size_m",0.)!=10. ||
        settings.at("world").at("spacing").get<double>()!=10.)
        throw std::invalid_argument("search polygon schema/cell size mismatch");
    const auto& vertices=site.at("vertices_m");
    if (!vertices.is_array()||vertices.size()<3)
        throw std::invalid_argument("search polygon needs at least three vertices");
    std::vector<std::pair<double,double>> points;
    for (const auto& v:vertices) {
        if (!v.is_array()||v.size()!=2)
            throw std::invalid_argument("search polygon vertex shape");
        const double x=v[0].get<double>(),y=v[1].get<double>();
        if (!std::isfinite(x)||!std::isfinite(y)||std::fmod(x,10.)!=0.||std::fmod(y,10.)!=0.)
            throw std::invalid_argument("search polygon vertices must be finite 10 m multiples");
        points.emplace_back(x,y);
    }
    bool positive=false,negative=false;
    for (std::size_t k=0;k<points.size();++k) {
        const auto a=points[k],b=points[(k+1)%points.size()];
        if (a==b)throw std::invalid_argument("zero search polygon edge");
        for (const auto& p:points) {
            const double first=(b.first-a.first)*(p.second-a.second);
            const double second=(b.second-a.second)*(p.first-a.first);
            const double tolerance=16*std::numeric_limits<double>::epsilon()
                *(std::abs(first)+std::abs(second));
            positive|=first-second>tolerance;
            negative|=first-second<-tolerance;
        }
    }
    if (positive==negative)
        throw std::invalid_argument("search polygon must be nondegenerate and convex in boundary order");
    auto result=settings;
    result["world"]["boundary"]=vertices;
    result["world"]["cell-domain"]="closed-polygon-centers-v1";
    return result;
}
} // namespace gf
