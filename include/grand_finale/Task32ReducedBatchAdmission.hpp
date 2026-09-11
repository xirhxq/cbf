#pragma once
#include <cstdint>
namespace gf {
// Launch identity only; never consulted by control, sensing or estimation.
inline bool task32RegisteredReducedBatch(int mechanism,double sigma,
    std::uint64_t gaussian,std::uint64_t link) {
    if(mechanism!=0&&mechanism!=2)return false;
    if(sigma==0.)return gaussian==2027&&link==154001;
    if(sigma!=.5)return false;
    return (gaussian==153011&&link==154011)||
           (gaussian==153029&&link==154029)||
           (gaussian==153047&&link==154047);
}
} // namespace gf
