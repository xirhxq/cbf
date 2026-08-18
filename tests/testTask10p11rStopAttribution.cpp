#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11rStopAttribution.hpp"

namespace {

gf::GrandFinaleRuntimeSnapshot twoReferenceSnapshot() {
    gf::GrandFinaleRuntimeSnapshot runtime;
    runtime.runtime_s=5.9;
    runtime.estimator_token=7;
    runtime.estimate.mobile_ids={8};
    runtime.estimate.fixed_positions={{100,{0.0,0.0}},{101,{0.0,100.0}}};
    runtime.estimate.mean=Eigen::Vector4d(100.0,50.0,0.0,0.0);
    runtime.estimate.covariance=Eigen::Matrix4d::Identity()*0.04;
    runtime.topology={{100,8},{101,8}};
    runtime.range_links[gf::UndirectedEdge::canonical(8,100).id()]={0.1,1.0,1.0};
    runtime.range_links[gf::UndirectedEdge::canonical(8,101).id()]={0.2,1.0,1.5};
    return runtime;
}

}

TEST_CASE("Reference attribution keeps nominal proxy and robust cone FIM distinct") {
    const auto audit=gf::attributeReferenceOwner(
        twoReferenceSnapshot(),8,3.0);
    REQUIRE(audit.valid);
    CHECK(audit.owner==8);
    CHECK(audit.effective_edges==std::vector<gf::DirectedEdge>{{100,8},{101,8}});
    CHECK(audit.reference_angle_rad>0.0);
    CHECK(audit.reference_angle_rad<M_PI);
    CHECK(std::isfinite(audit.nominal_fim_eigenvalue));
    CHECK(std::isfinite(audit.posterior_fim_proxy_eigenvalue));
    CHECK(std::isfinite(audit.robust_cone_fim_lower_bound));
    CHECK(audit.nominal_fim_eigenvalue>=
          audit.posterior_fim_proxy_eigenvalue);
    CHECK(audit.posterior_fim_proxy_eigenvalue>
          audit.robust_cone_fim_lower_bound);
    CHECK(audit.edges.size()==2);
    CHECK(audit.edges.front().position_support_m>0.0);
    CHECK(audit.edges.front().range_variance_m2>0.0);
}

TEST_CASE("Reference attribution fails closed when frozen range history is missing") {
    auto runtime=twoReferenceSnapshot();
    runtime.range_links.erase(gf::UndirectedEdge::canonical(8,101).id());
    const auto audit=gf::attributeReferenceOwner(runtime,8,3.0);
    CHECK_FALSE(audit.valid);
    CHECK(audit.reason=="range_missing:8--101");
}

TEST_CASE("Candidate attribution preserves ordered rejection evidence") {
    gf::ReplacementCandidateAttribution first;
    first.removal={100,8};
    first.addition={7,8};
    first.dag_valid=true;
    first.keep_distance_valid=true;
    first.add_distance_valid=true;
    first.aoi_valid=true;
    first.posterior_valid=true;
    first.fim_valid=true;
    first.reference_lens_valid=true;
    first.target_cone_planning_score=-0.2;
    first.exact_rejection_reason="target_cone_planning_score";
    auto second=first;
    second.addition={6,8};
    second.target_cone_planning_score=-0.1;
    const auto ledger=gf::freezeCandidateAttributionLedger({first,second});
    REQUIRE(ledger.valid);
    REQUIRE(ledger.candidates.size()==2);
    CHECK(ledger.candidates[0].addition.id()=="7->8");
    CHECK(ledger.candidates[1].addition.id()=="6->8");
    CHECK(ledger.digest!=0);
}

TEST_CASE("Candidate attribution rejects duplicate or incomplete candidate identity") {
    gf::ReplacementCandidateAttribution value;
    value.removal={100,8};
    value.addition={7,8};
    value.exact_rejection_reason="target_cone_planning_score";
    const auto duplicate=gf::freezeCandidateAttributionLedger({value,value});
    CHECK_FALSE(duplicate.valid);
    CHECK(duplicate.reason=="duplicate_candidate_attribution");
    value.addition={0,1};
    const auto invalid=gf::freezeCandidateAttributionLedger({value});
    CHECK_FALSE(invalid.valid);
    CHECK(invalid.reason=="invalid_candidate_attribution");
}
