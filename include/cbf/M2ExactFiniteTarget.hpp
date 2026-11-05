#ifndef CBF_M2_EXACT_FINITE_TARGET_HPP
#define CBF_M2_EXACT_FINITE_TARGET_HPP
#include "world/world"
#include "cbf/ExactVoronoi.hpp"
#include <cfenv>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>

#ifdef __FAST_MATH__
#error "M2 exact target prototype requires ordinary IEEE binary64 arithmetic"
#endif
namespace cbf2026 {
static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559
              && std::numeric_limits<double>::digits==53,
              "M2 target evidence requires IEEE binary64");
struct M2FiniteTargetSelection {
    Point point;
    json record;
};
inline std::string m2BitmapHex(const std::vector<unsigned char>& bytes) {
    const char* digits="0123456789abcdef";
    std::string out;out.reserve(2*bytes.size());
    for(const auto byte:bytes){out.push_back(digits[byte>>4]);out.push_back(digits[byte&15]);}
    return out;
}
inline std::string m2Binary64BitsHex(double value) {
    std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));
    const char* digits="0123456789abcdef";
    std::string out(16,'0');
    for(int j=15;j>=0;--j){out[j]=digits[bits&15];bits>>=4;}
    return out;
}

// Finite task selector only: no controller, solver, random generator or oracle.
// Empty ownership is a declared finite zero task, not a failed-input substitute.
inline M2FiniteTargetSelection selectM2ExactFiniteTarget(
        GridWorld& grid,int ownId,const std::vector<std::pair<int,Point>>& sites) {
    const auto started=std::chrono::steady_clock::now();
    if(sites.size()!=14 || ownId<1 || ownId>14)
        throw std::invalid_argument("M2 requires ordered14 mobile sites and valid own ID");
    if(grid.xNum!=300 || grid.yNum!=300 || grid.xLim.first!=-1500.
       || grid.xLim.second!=1500. || grid.yLim.first!=-1500.
       || grid.yLim.second!=1500. || grid.valid.size()!=90000 || grid.vis.size()!=90000)
        throw std::invalid_argument("M2 fixed finite grid/schema mismatch");
    if(std::fegetround()!=FE_TONEAREST)
        throw std::invalid_argument("M2 target conversion requires nearest-even rounding");
    for(std::size_t k=0;k<sites.size();++k)
        if(sites[k].first!=static_cast<int>(k)+1 || !std::isfinite(sites[k].second.x)
           || !std::isfinite(sites[k].second.y))
            throw std::invalid_argument("M2 incomplete/nonfinite/duplicate estimated sites");
    const Point own=sites[ownId-1].second;
    std::vector<std::pair<int,ExactVoronoiPlane>> planes;
    for(const auto& site:sites)if(site.first!=ownId)
        planes.emplace_back(site.first,ExactVoronoiPlane(own,site.second));
    std::vector<unsigned char> ownedBits(11250,0),unvisitedBits(11250,0);
    std::int64_t owned=0,unvisited=0,ownedX=0,ownedY=0,unvisitedX=0,unvisitedY=0;
    std::uint64_t predicateCalls=0;
    for(int ix=0;ix<300;++ix)for(int iy=0;iy<300;++iy) {
        const int index=ix*300+iy;
        if(grid.getIndex(ix,iy)!=index || !grid.valid[index])
            throw std::invalid_argument("M2 valid mask must be all90000 canonical points");
        const Point point(-1500+10*ix,-1500+10*iy);
        bool admitted=true;
        for(const auto& plane:planes) {
            ++predicateCalls;const int sign=plane.second.sign(point);
            if(sign>0 || (sign==0 && plane.first<ownId)){admitted=false;break;}
        }
        if(!admitted)continue;
        ++owned;ownedX+=ix;ownedY+=iy;ownedBits[index/8]|=1u<<(index%8);
        if(grid.vis[index])continue;
        ++unvisited;unvisitedX+=ix;unvisitedY+=iy;unvisitedBits[index/8]|=1u<<(index%8);
    }
    Point target=own;std::string branch="empty-owned-cell-estimated-self-zero-task";
    std::int64_t denominator=0,sx=0,sy=0,numeratorX=0,numeratorY=0;
    if(unvisited>0) {
        branch="unvisited-cell-exact-index-centroid";
        denominator=unvisited;sx=unvisitedX;sy=unvisitedY;
    } else if(owned>0) {
        branch="completed-owned-cell-exact-index-centroid";
        denominator=owned;sx=ownedX;sy=ownedY;
    }
    if(denominator>0) {
        // All integer operands are exactly representable: |numerator|<=135000000.
        numeratorX=-1500*denominator+10*sx;numeratorY=-1500*denominator+10*sy;
        target=Point(static_cast<double>(numeratorX)/static_cast<double>(denominator),
                     static_cast<double>(numeratorY)/static_cast<double>(denominator));
    }
    if(!std::isfinite(target.x) || !std::isfinite(target.y))
        throw std::runtime_error("M2 nonfinite target, no substitute allowed");
    json siteRecord=json::array();
    for(const auto& site:sites)siteRecord.push_back({{"id",site.first},{"estimate",{site.second.x,site.second.y}}});
    const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    return {target,{{"identity","M2-exact-finite-unvisited-voronoi-centroid-PROTOTYPE-v1"},
        {"qualification","unreviewed/unexecuted prototype, compile is not qualification"},
        {"own_id",ownId},{"sites",siteRecord},{"grid_shape",{300,300}},
        {"valid_grid_cells",90000},{"valid_mask_rule","all90000 fixed canonical points"},
        {"grid_order","x-major/y-minor; little bit order"},
        {"predicate","reviewed exact IEEE dyadic squared-distance difference"},
        {"tie_rule","lowest mobile node ID; beacons not task owners"},
        {"owned_grid_bitset_hex",m2BitmapHex(ownedBits)},
        {"unvisited_owned_grid_bitset_hex",m2BitmapHex(unvisitedBits)},
        {"owned_count",owned},{"unvisited_count",unvisited},
        {"owned_sum_x_indices",ownedX},{"owned_sum_y_indices",ownedY},
        {"unvisited_sum_x_indices",unvisitedX},{"unvisited_sum_y_indices",unvisitedY},
        {"centroid_mass",denominator},{"centroid_sum_x_indices",sx},{"centroid_sum_y_indices",sy},
        {"rational_target",denominator>0?json{{"x_numerator",numeratorX},{"y_numerator",numeratorY},{"denominator",denominator}}:json(nullptr)},
        {"target",{target.x,target.y}},
        {"target_binary64_bits_hex",{m2Binary64BitsHex(target.x),m2Binary64BitsHex(target.y)}},
        {"rule",branch},{"empty_zero_task",denominator==0},
        {"conversion","exact integer operands, one nearest-even binary64 division per centroid coordinate"},
        {"continuous_cell_vertices",json::array()},{"display_polygon_used_for_target",false},
        {"exact_plane_evaluations",predicateCalls},{"target_construction_wall_s",wall},
        {"cost_scope","complete target validation/13planes/grid assignment/centroid/site evidence; before final bitmap-to-hex/record serialization, nested in parent update"}}};
}
}
#endif
