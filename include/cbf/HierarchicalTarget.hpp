#ifndef CBF_HIERARCHICAL_TARGET_HPP
#define CBF_HIERARCHICAL_TARGET_HPP
#include "world/world"
#include <cstdint>
#include "cbf/ExactVoronoi.hpp"

namespace cbf2026 {
// Execution protocol, not a convergence guarantee. No index clamp or epsilon
// is used to admit a grid point to the closed continuous Voronoi cell.
struct LeaderTargetSelection {
    Point point;
    json record;
};
inline long double leaderPlane(const Point& p, const Point& self, const Point& other) {
    const long double dx=static_cast<long double>(other.x)-self.x;
    const long double dy=static_cast<long double>(other.y)-self.y;
    return (static_cast<long double>(p.x)-(static_cast<long double>(self.x)+other.x)/2)*dx
          +(static_cast<long double>(p.y)-(static_cast<long double>(self.y)+other.y)/2)*dy;
}
inline bool insideConvexBoundary(const Point& point, const Polygon& boundary) {
    long double area=0;
    for(int i=1;i<=boundary.n;++i) {const Point a=boundary.p[i],b=boundary.p[i%boundary.n+1];area+=static_cast<long double>(a.x)*b.y-static_cast<long double>(a.y)*b.x;}
    for(int i=1;i<=boundary.n;++i) {
        const Point a=boundary.p[i],b=boundary.p[i%boundary.n+1];
        const long double cross=(static_cast<long double>(b.x)-a.x)*(static_cast<long double>(point.y)-a.y)-(static_cast<long double>(b.y)-a.y)*(static_cast<long double>(point.x)-a.x);
        if((area>0 && cross<0)||(area<0 && cross>0))return false;
    }
    return boundary.n>=3 && area!=0;
}
inline std::vector<Point> continuousLeaderCell(const Polygon& boundary,int ownId,const std::vector<std::pair<int,Point>>& sites) {
    std::vector<Point> polygon;for(int i=1;i<=boundary.n;++i)polygon.push_back(boundary.p[i]);
    Point own;for(const auto& s:sites)if(s.first==ownId)own=s.second;
    for(const auto& site:sites)if(site.first!=ownId) {
        if(site.second.x==own.x && site.second.y==own.y && site.first<ownId)return {};
        std::vector<Point> clipped;
        for(std::size_t i=0;i<polygon.size();++i) {
            const Point a=polygon[i],b=polygon[(i+1)%polygon.size()];
            const long double fa=leaderPlane(a,own,site.second),fb=leaderPlane(b,own,site.second);
            if(fa<=0)clipped.push_back(a);
            if((fa<0 && fb>0)||(fa>0 && fb<0)) {
                const long double t=fa/(fa-fb);
                clipped.emplace_back(static_cast<double>(a.x+t*(static_cast<long double>(b.x)-a.x)),static_cast<double>(a.y+t*(static_cast<long double>(b.y)-a.y)));
            }
        }
        polygon=std::move(clipped);if(polygon.empty())break;
    }
    return polygon;
}
inline LeaderTargetSelection selectLeaderTarget(GridWorld& grid,const Polygon& boundary,int ownId,const std::vector<std::pair<int,Point>>& sites,double yaw,double frontDistance,bool exactPredicate=false) {
    Point own;bool found=false;
    for(const auto& s:sites)if(s.first==ownId){own=s.second;found=true;}
    if(!found || !std::isfinite(own.x) || !std::isfinite(own.y) || !std::isfinite(yaw))throw std::invalid_argument("invalid leader target snapshot");
    const Point focus=own+frontDistance*Point(std::cos(yaw),std::sin(yaw));
    std::vector<std::pair<int,ExactVoronoiPlane>> exactPlanes;
    if(exactPredicate)for(const auto& site:sites)if(site.first!=ownId)exactPlanes.emplace_back(site.first,ExactVoronoiPlane(own,site.second));
    std::vector<unsigned char> bitmap((grid.xNum*grid.yNum+7)/8,0);
    std::int64_t sx=0,sy=0;int count=0,todo=0,bestX=-1,bestY=-1;double best=std::numeric_limits<double>::infinity();Point result;
    for(int ix=0;ix<grid.xNum;++ix)for(int iy=0;iy<grid.yNum;++iy) {
        if(!grid.valid[grid.getIndex(ix,iy)])continue;
        const Point p(grid.xLim.first+(grid.xLim.second-grid.xLim.first)*ix/grid.xNum,grid.yLim.first+(grid.yLim.second-grid.yLim.first)*iy/grid.yNum);
        if(!insideConvexBoundary(p,boundary))continue;
        bool admitted=true;
        if(exactPredicate) {
            for(const auto& item:exactPlanes) {const int sign=item.second.sign(p);if(sign>0||(sign==0&&item.first<ownId)){admitted=false;break;}}
        } else for(const auto& s:sites)if(s.first!=ownId) {
            const long double plane=leaderPlane(p,own,s.second);
            if(plane>0 || (plane==0 && s.first<ownId)){admitted=false;break;}
        }
        if(!admitted)continue;
        ++count;sx+=ix;sy+=iy;const int bit=grid.getIndex(ix,iy);bitmap[bit/8]|=1u<<(bit%8);
        if(grid.getValue(ix,iy))continue;
        ++todo;const Point d=p-focus;const double distance=d*d;
        if(distance<best){best=distance;bestX=ix;bestY=iy;result=p;}
    }
    std::string rule="nearest-unsearched-grid";
    if(todo==0 && count>0) {
        rule="completed-cell-integer-index-centroid";
        result=Point(grid.xLim.first+(grid.xLim.second-grid.xLim.first)*sx/(static_cast<double>(grid.xNum)*count),grid.yLim.first+(grid.yLim.second-grid.yLim.first)*sy/(static_cast<double>(grid.yNum)*count));
    } else if(count==0) {
        const auto polygon=continuousLeaderCell(boundary,ownId,sites);
        if(polygon.empty()) {rule="empty-continuous-cell-hold-estimated-leader";result=own;}
        else {rule="empty-raster-cell-continuous-vertex-mean";result=Point(0,0);for(const auto& p:polygon)result=result+p;result=result/static_cast<double>(polygon.size());}
    }
    if(!std::isfinite(result.x)||!std::isfinite(result.y))throw std::runtime_error("nonfinite leader target");
    json vertices=json::array();for(const auto& p:continuousLeaderCell(boundary,ownId,sites))vertices.push_back({p.x,p.y});
    json siteJson=json::array();for(const auto& s:sites)siteJson.push_back({{"id",s.first},{"estimate",{s.second.x,s.second.y}}});
    json identity=exactPredicate?"continuous-halfplane-grid-v3-exact-dyadic":"continuous-halfplane-grid-v2";
    std::string bitset;const char* hex="0123456789abcdef";if(exactPredicate)for(const auto byte:bitmap){bitset.push_back(hex[byte>>4]);bitset.push_back(hex[byte&15]);}
    return {result,{{"admitted_grid_bitset_hex",exactPredicate?json(bitset):json(nullptr)},{"predicate",exactPredicate?"exact dyadic sign of squared-distance difference":"legacy floating midpoint sign"},{"identity",identity},{"leader_id",ownId},{"sites",siteJson},{"continuous_cell_vertices",vertices},{"leader_yaw_rad",yaw},{"focus",{focus.x,focus.y}},{"grid_cells",count},{"unsearched_cells",todo},{"sum_x_indices",sx},{"sum_y_indices",sy},{"selected_indices",{bestX,bestY}},{"rule",rule},{"leader_target",{result.x,result.y}},{"grid_tie_rule","x-major/y-minor; equal site distance owned by lowest leader ID"},{"yaw_row_method","central difference .001 m/rad; finite zero target convention"},{"display_polygon_used_for_target",false}}};
}
}
#endif
