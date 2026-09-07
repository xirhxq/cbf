#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "grand_finale/Task31TriangularLattice.hpp"
#include "grand_finale/Task25P0MultiDag.hpp"

TEST_CASE("Task31 generator derives depth and triangular slots from labelled contracts") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{1800,-50}},{101,{2250,-50}},{102,{2700,-50}}};
    for(int code:{0,2,12,13}) {
        const auto original=gf::task25DagContractFromCode(code);
        const auto g=gf::task31TriangularLattice(original,fixed,Eigen::Vector2d(0,1));
        REQUIRE(g.valid);
        CHECK(g.contract.reference_edges==original.reference_edges);
        CHECK(g.contract.coverage_units.size()==original.coverage_units.size());
        for(const auto& unit:original.coverage_units) {
            std::map<int,std::vector<double>> slots;
            for(auto id:unit.members) {
                REQUIRE(g.cells.count(id)==1);
                const auto cell=g.cells.at(id);
                CHECK(cell.row>=1);
                CHECK(std::abs(2*cell.slot-std::round(2*cell.slot))<1e-9);
                slots[cell.row].push_back(cell.slot);
            }
            for(auto [row,values]:slots) {
                std::sort(values.begin(),values.end());
                for(size_t k=1;k<values.size();++k) CHECK(values[k]-values[k-1]==doctest::Approx(1));
                if(slots.count(row+1))CHECK(std::abs(std::remainder(2*(values[0]-slots.at(row+1)[0]),2))==doctest::Approx(1));
            }
        }
        const auto again=gf::task31TriangularLattice(original,fixed,Eigen::Vector2d(0,1));
        for(const auto& [id,cell]:g.cells) CHECK(again.cells.at(id).slot==cell.slot);
        auto relabel=original;std::map<gf::NodeId,gf::NodeId> ids;
        for(const auto& [id,role]:original.member_roles)ids[id]=300-id*3;
        for(const auto& [id,p]:fixed)ids[id]=900+id;
        for(auto& edge:relabel.reference_edges){edge.reference=ids.at(edge.reference);edge.owner=ids.at(edge.owner);}
        relabel.member_roles.clear();
        for(const auto& [id,r]:original.member_roles){auto role=r;role.member=ids.at(id);relabel.member_roles[role.member]=role;}
        for(auto& unit:relabel.coverage_units){for(auto& id:unit.members)id=ids.at(id);for(auto& id:unit.base_anchors)id=ids.at(id);for(auto& id:unit.front_members)id=ids.at(id);unit.leader=ids.at(unit.leader);}
        std::map<gf::NodeId,Eigen::Vector2d> moved;
        const Eigen::Rotation2Dd rotation(.73);const Eigen::Vector2d shift(93,-117);
        for(const auto& [id,p]:fixed)moved[ids.at(id)]=rotation*p+shift;
        const auto transformed=gf::task31TriangularLattice(relabel,moved,rotation*Eigen::Vector2d(0,1));
        REQUIRE(transformed.valid);
        for(const auto& [id,cell]:g.cells){CHECK(transformed.cells.at(ids.at(id)).row==cell.row);CHECK(transformed.cells.at(ids.at(id)).slot==cell.slot);}
        nlohmann::json record={{"code",code},{"cells",nlohmann::json::object()},{"roles",nlohmann::json::object()}};
        for(const auto& [id,cell]:g.cells){record["cells"][std::to_string(id)]={cell.row,cell.slot};const auto r=g.contract.member_roles.at(id);record["roles"][std::to_string(id)]={r.axial_fraction,r.triangular_fraction};}
        std::cout<<"TASK31_LATTICE "<<record.dump()<<'\n';
    }
}

TEST_CASE("Task31 invalid graph rejects rather than silently generating a pattern") {
    const std::map<gf::NodeId,Eigen::Vector2d> fixed{{100,{-450,0}},{101,{0,0}},{102,{450,0}}};
    auto c=gf::task25DagContractFromCode(12);c.reference_edges.push_back({13,1});
    CHECK_FALSE(gf::task31TriangularLattice(c,fixed,{0,1}).valid);
    c=gf::task25DagContractFromCode(12);c.coverage_units.front().members.pop_back();
    CHECK_FALSE(gf::task31TriangularLattice(c,fixed,{0,1}).valid);
    CHECK_FALSE(gf::task31TriangularLattice(gf::task25DagContractFromCode(12),fixed,{0,0}).valid);
}
