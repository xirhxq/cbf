#ifndef CBF_SAME_FRAME_LEADER_REUSE_HPP
#define CBF_SAME_FRAME_LEADER_REUSE_HPP
#include "cbf/HierarchicalTarget.hpp"
#include <cstring>
#include <map>
#include <vector>
#include <cstdint>
namespace leader_reuse {
struct Key {
    std::uint64_t frame;
    int leader, nx, ny;
    bool exact;
    std::vector<std::uint64_t> numbers;
    std::vector<bool> valid, visited;
    bool operator==(const Key& b) const {
        return frame==b.frame && leader==b.leader && nx==b.nx && ny==b.ny && exact==b.exact &&
            numbers==b.numbers && valid==b.valid && visited==b.visited;
    }
};
inline std::uint64_t bits(double x) { std::uint64_t u; std::memcpy(&u,&x,sizeof(u)); return u; }
inline Key key(std::uint64_t frame,const GridWorld& g,const Polygon& boundary,int leader,
               const std::vector<std::pair<int,Point>>& sites,double yaw,double focus,bool exact) {
    Key k{frame,leader,g.xNum,g.yNum,exact,{},g.valid,g.vis};
    k.numbers={bits(g.xLim.first),bits(g.xLim.second),bits(g.yLim.first),bits(g.yLim.second),bits(yaw),bits(focus),static_cast<std::uint64_t>(boundary.n)};
    for(int i=1;i<=boundary.n;++i) {k.numbers.push_back(bits(boundary.p[i].x));k.numbers.push_back(bits(boundary.p[i].y));}
    k.numbers.push_back(sites.size());
    for(const auto& s:sites) {k.numbers.push_back(static_cast<std::uint64_t>(s.first));k.numbers.push_back(bits(s.second.x));k.numbers.push_back(bits(s.second.y));}
    return k;
}
struct Entry { Key key; cbf2026::LeaderTargetSelection value; };
inline std::map<int,Entry>& cache(){static thread_local std::map<int,Entry> c;return c;}
inline std::uint64_t& currentFrame(){static thread_local std::uint64_t f=0;return f;}
inline bool& active(){static thread_local bool a=false;return a;}
inline unsigned& hits(){static thread_local unsigned h=0;return h;}
inline unsigned& misses(){static thread_local unsigned m=0;return m;}
inline void beginFrame(std::uint64_t frame){cache().clear();currentFrame()=frame;active()=true;hits()=misses()=0;}
inline cbf2026::LeaderTargetSelection select(std::uint64_t frame,GridWorld& grid,const Polygon& boundary,int leader,
                  const std::vector<std::pair<int,Point>>& sites,double yaw,double focus,bool exact){
    if (!active() || currentFrame()!=frame) throw std::runtime_error("leader reuse frame not initialized");
    Key k=key(frame,grid,boundary,leader,sites,yaw,focus,exact);
    auto it=cache().find(leader);
    if(it!=cache().end() && it->second.key==k) {++hits();return it->second.value;}
    ++misses();auto result=cbf2026::selectLeaderTarget(grid,boundary,leader,sites,yaw,focus,exact);
    cache().insert_or_assign(leader,Entry{std::move(k),result});return result;
}
}
#endif
