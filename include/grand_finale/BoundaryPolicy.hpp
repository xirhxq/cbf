#pragma once

#include "grand_finale/CanonicalHardRows.hpp"

#include <string>
#include <vector>

namespace gf {

enum class BoundaryPolicy {
    None,
    SoftSearchRetention,
    HardFlightBoundary
};

enum class FlightPolygonSource {
    SearchPolygon,
    ExplicitPolygon
};

struct BoundaryPolicyConfig {
    BoundaryPolicy policy=BoundaryPolicy::None;
    FlightPolygonSource flight_polygon_source=
        FlightPolygonSource::SearchPolygon;
    std::vector<Eigen::Vector2d> explicit_flight_polygon;
    // Development soft-objective weight.  It remains configurable and is not
    // frozen as a scientific parameter by Task 10.11h.
    double soft_slack_weight=1.0;
};

struct BoundarySoftFacet2D {
    std::string id;
    Eigen::Vector2d outward_normal=Eigen::Vector2d::Zero();
    double offset_m=0.0;
};

struct BoundaryBlueprint {
    BoundaryPolicy policy=BoundaryPolicy::None;
    std::vector<WorkspaceFacet2D> hard_facets;
    std::vector<BoundarySoftFacet2D> soft_facets;
};

namespace boundary_policy_detail {

inline std::vector<WorkspaceFacet2D> polygonFacets(
    const std::vector<Eigen::Vector2d>& polygon) {
    if (polygon.size()<3)
        throw std::invalid_argument("boundary polygon requires three vertices");
    Eigen::Vector2d centroid=Eigen::Vector2d::Zero();
    for (const auto& point : polygon) {
        if (!point.allFinite())
            throw std::invalid_argument("boundary polygon must be finite");
        centroid+=point;
    }
    centroid/=static_cast<double>(polygon.size());
    std::vector<WorkspaceFacet2D> facets;
    for (std::size_t index=0;index<polygon.size();++index) {
        const Eigen::Vector2d edge=
            polygon[(index+1)%polygon.size()]-polygon[index];
        if (edge.norm()<=1e-12)
            throw std::invalid_argument("boundary polygon has zero edge");
        Eigen::Vector2d outward(edge.y(),-edge.x());
        outward.normalize();
        double offset=outward.dot(polygon[index]);
        if (outward.dot(centroid)>offset) {
            outward=-outward;
            offset=-offset;
        }
        facets.push_back({"facet:"+std::to_string(index),outward,offset});
    }
    return facets;
}

}  // namespace boundary_policy_detail

inline BoundaryBlueprint buildBoundaryBlueprint(
    const BoundaryPolicyConfig& config,
    const std::vector<Eigen::Vector2d>& search_polygon,
    const std::vector<Eigen::Vector2d>& explicit_polygon) {
    if (!std::isfinite(config.soft_slack_weight) ||
        config.soft_slack_weight<=0.0)
        throw std::invalid_argument("boundary soft slack weight must be positive");
    BoundaryBlueprint result;
    result.policy=config.policy;
    if (config.policy==BoundaryPolicy::None) return result;
    if (config.policy==BoundaryPolicy::SoftSearchRetention) {
        for (const auto& facet :
             boundary_policy_detail::polygonFacets(search_polygon))
            result.soft_facets.push_back(
                {facet.id,facet.outward_normal,facet.offset_m});
        return result;
    }
    const auto& selected=config.flight_polygon_source==
            FlightPolygonSource::SearchPolygon
        ?search_polygon:explicit_polygon;
    result.hard_facets=boundary_policy_detail::polygonFacets(selected);
    return result;
}

inline double distanceOutsidePolygon(
    const Eigen::Vector2d& point,
    const std::vector<Eigen::Vector2d>& polygon) {
    if (!point.allFinite())
        throw std::invalid_argument("excursion point must be finite");
    const auto facets=boundary_policy_detail::polygonFacets(polygon);
    bool inside=true;
    for (const auto& facet : facets)
        inside=inside &&
            facet.outward_normal.dot(point)<=facet.offset_m+1e-12;
    if (inside) return 0.0;
    double distance=std::numeric_limits<double>::infinity();
    for (std::size_t index=0;index<polygon.size();++index) {
        const Eigen::Vector2d first=polygon[index];
        const Eigen::Vector2d edge=
            polygon[(index+1)%polygon.size()]-first;
        const double fraction=std::max(0.0,std::min(1.0,
            (point-first).dot(edge)/edge.squaredNorm()));
        distance=std::min(distance,(point-(first+fraction*edge)).norm());
    }
    return distance;
}

}  // namespace gf
