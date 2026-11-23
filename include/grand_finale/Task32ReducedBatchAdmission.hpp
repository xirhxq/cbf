#pragma once
#include <cstdint>
namespace gf {
// Launch identity only; never consulted by control, sensing or estimation.
inline bool task32RegisteredReducedBatch(int mechanism,double sigma,
    std::uint64_t gaussian,std::uint64_t link) {
    if(mechanism!=0&&mechanism!=2)return false;
    if(sigma==0.)return gaussian==2027&&link==154001;
    if(sigma!=.5)return false;
    return (mechanism==2&&((gaussian==211011&&link==212011)||
           (gaussian==211029&&link==212029)||(gaussian==211047&&link==212047)||(gaussian==217011&&link==218011)||(gaussian==217029&&link==218029)||(gaussian==217047&&link==218047)))||
           (gaussian==183011&&link==184011)||
           (gaussian==183029&&link==184029)||
           (gaussian==183047&&link==184047)||
           (mechanism==2&&((gaussian==193011&&link==194011)||(gaussian==193029&&link==194029)||(gaussian==193047&&link==194047)))||(mechanism==2&&((gaussian==153011&&link==154011)||(gaussian==153029&&link==154029)||(gaussian==153047&&link==154047)));
}
} // namespace gf
