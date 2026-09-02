#pragma once

#include "world/World.hpp"
#include "world/GridWorld.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace gf {

struct Task17GridSnapshot {
    int x_cells=0;
    int y_cells=0;
    int valid_count=0;
    std::string valid_bits_hex;
    std::string certified_bits_hex;
    std::string truth_bits_hex;
    int certified_count=0;
    int truth_count=0;
    std::uint64_t certified_hash=0;
    std::uint64_t truth_hash=0;
};

struct Task17GridDelta {
    std::vector<std::string> certified_new_ids;
    std::vector<std::string> truth_new_ids;
    int certified_count=0;
    int truth_count=0;
};

namespace task17_grid_detail {

inline std::string bitHex(const std::vector<bool>& values) {
    static constexpr char digits[]="0123456789abcdef";
    std::string result((values.size()+3)/4,'0');
    for (std::size_t index=0;index<values.size();++index) {
        if (!values[index]) continue;
        const std::size_t digit=index/4;
        const unsigned shift=static_cast<unsigned>(index%4);
        const char character=result[digit];
        const unsigned value=character>='0'&&character<='9'
            ?static_cast<unsigned>(character-'0'):
             static_cast<unsigned>(character-'a'+10);
        result[digit]=digits[value|(1U<<shift)];
    }
    return result;
}

inline std::vector<bool> bitsFromHex(
    const std::string& value,std::size_t count) {
    if (value.size()!=(count+3)/4)
        throw std::invalid_argument("Task 17 packed mask length mismatch");
    std::vector<bool> result(count,false);
    for (std::size_t digit=0;digit<value.size();++digit) {
        const char character=value[digit];
        const unsigned nibble=character>='0'&&character<='9'
            ?static_cast<unsigned>(character-'0'):
            character>='a'&&character<='f'
                ?static_cast<unsigned>(character-'a'+10):16U;
        if (nibble>15U)
            throw std::invalid_argument("Task 17 packed mask digit invalid");
        for (unsigned shift=0;shift<4;++shift) {
            const std::size_t index=4*digit+shift;
            if (index<count) result[index]=(nibble&(1U<<shift))!=0;
            else if ((nibble&(1U<<shift))!=0)
                throw std::invalid_argument(
                    "Task 17 packed mask padding invalid");
        }
    }
    return result;
}

inline int coveredCount(const GridWorld& grid) {
    int result=0;
    for (std::size_t index=0;index<grid.vis.size();++index)
        result+=grid.valid[index]&&grid.vis[index];
    return result;
}

inline std::uint64_t maskHash(const std::string& valid,
    const std::string& covered,int x_cells,int y_cells) {
    std::string canonical=std::to_string(x_cells)+"x"+
        std::to_string(y_cells)+"|"+valid+"|"+covered;
    std::uint64_t hash=1469598103934665603ULL;
    for (const unsigned char value:canonical) {
        hash^=value;
        hash*=1099511628211ULL;
    }
    return hash==0?1:hash;
}

inline void requireCompatible(const GridWorld& first,
    const GridWorld& second) {
    if (first.xNum!=second.xNum||first.yNum!=second.yNum||
        first.valid!=second.valid||first.vis.size()!=second.vis.size())
        throw std::invalid_argument("Task 17 GridWorld mismatch");
}

inline std::pair<int,int> parseId(const std::string& id) {
    const auto colon=id.find(':');
    if (colon==std::string::npos)
        throw std::invalid_argument("Task 17 cell ID invalid");
    return {std::stoi(id.substr(0,colon)),std::stoi(id.substr(colon+1))};
}

}  // namespace task17_grid_detail

inline Task17GridSnapshot task17GridSnapshot(
    const GridWorld& certified,const GridWorld& truth) {
    task17_grid_detail::requireCompatible(certified,truth);
    Task17GridSnapshot result;
    result.x_cells=certified.xNum;
    result.y_cells=certified.yNum;
    result.valid_count=certified.validCount;
    result.valid_bits_hex=task17_grid_detail::bitHex(certified.valid);
    result.certified_bits_hex=task17_grid_detail::bitHex(certified.vis);
    result.truth_bits_hex=task17_grid_detail::bitHex(truth.vis);
    result.certified_count=task17_grid_detail::coveredCount(certified);
    result.truth_count=task17_grid_detail::coveredCount(truth);
    result.certified_hash=task17_grid_detail::maskHash(
        result.valid_bits_hex,result.certified_bits_hex,
        result.x_cells,result.y_cells);
    result.truth_hash=task17_grid_detail::maskHash(
        result.valid_bits_hex,result.truth_bits_hex,
        result.x_cells,result.y_cells);
    return result;
}

inline Task17GridDelta task17GridDelta(
    const GridWorld& previous_certified,const GridWorld& previous_truth,
    const GridWorld& certified,const GridWorld& truth) {
    task17_grid_detail::requireCompatible(previous_certified,certified);
    task17_grid_detail::requireCompatible(previous_truth,truth);
    task17_grid_detail::requireCompatible(certified,truth);
    Task17GridDelta result;
    for (int x=0;x<certified.xNum;++x) for (int y=0;y<certified.yNum;++y) {
        const int index=certified.getIndex(x,y);
        if (previous_certified.vis[index]&&!certified.vis[index])
            throw std::invalid_argument("Task 17 certified coverage regressed");
        if (previous_truth.vis[index]&&!truth.vis[index])
            throw std::invalid_argument("Task 17 truth coverage regressed");
        const std::string id=std::to_string(x)+":"+std::to_string(y);
        if (certified.valid[index]&&!previous_certified.vis[index]&&
            certified.vis[index]) result.certified_new_ids.push_back(id);
        if (truth.valid[index]&&!previous_truth.vis[index]&&truth.vis[index])
            result.truth_new_ids.push_back(id);
    }
    result.certified_count=task17_grid_detail::coveredCount(certified);
    result.truth_count=task17_grid_detail::coveredCount(truth);
    return result;
}

inline Task17GridSnapshot task17ApplyGridDelta(
    const Task17GridSnapshot& initial,const Task17GridDelta& delta) {
    const std::size_t count=static_cast<std::size_t>(initial.x_cells)*
        static_cast<std::size_t>(initial.y_cells);
    auto certified=task17_grid_detail::bitsFromHex(
        initial.certified_bits_hex,count);
    auto truth=task17_grid_detail::bitsFromHex(initial.truth_bits_hex,count);
    const auto apply=[&](std::vector<bool>& bits,
        const std::vector<std::string>& ids) {
        for (const auto& id:ids) {
            const auto [x,y]=task17_grid_detail::parseId(id);
            if (x<0||x>=initial.x_cells||y<0||y>=initial.y_cells)
                throw std::invalid_argument("Task 17 cell ID out of range");
            bits[static_cast<std::size_t>(x*initial.y_cells+y)]=true;
        }
    };
    apply(certified,delta.certified_new_ids);
    apply(truth,delta.truth_new_ids);
    Task17GridSnapshot result=initial;
    result.certified_bits_hex=task17_grid_detail::bitHex(certified);
    result.truth_bits_hex=task17_grid_detail::bitHex(truth);
    result.certified_count=delta.certified_count;
    result.truth_count=delta.truth_count;
    result.certified_hash=task17_grid_detail::maskHash(
        result.valid_bits_hex,result.certified_bits_hex,
        result.x_cells,result.y_cells);
    result.truth_hash=task17_grid_detail::maskHash(
        result.valid_bits_hex,result.truth_bits_hex,
        result.x_cells,result.y_cells);
    return result;
}

}  // namespace gf
