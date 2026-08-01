#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "cbf/AllocatedPairwiseCBF.hpp"
#include "cbf/BarrierEdgeRegistry.hpp"

#include <array>
#include <set>
#include <vector>

TEST_CASE("fixed localization registry exposes both UAV endpoints") {
    const cbf2026::BarrierEdgeRegistry registry(
        4,
        {
            {2, 1, false},
            {3, 1, false},
            {3, 2, false},
            {1, 0, true}
        }
    );

    REQUIRE(registry.fixedLocalizationEdges().size() == 4);
    CHECK(registry.incidentEdges(1).size() == 6);
    CHECK(registry.incidentEdges(2).size() == 5);
}

TEST_CASE("registry emits every unordered collision pair exactly once") {
    const cbf2026::BarrierEdgeRegistry registry(4, {});
    const auto& edges = registry.collisionEdges();
    REQUIRE(edges.size() == 6);
    CHECK(edges[0] == cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Collision, 1, 2
    ));
    CHECK(edges[5] == cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Collision, 3, 4
    ));
}

TEST_CASE("four-UAV registry creates complete rows only for fixed hard edges") {
    const cbf2026::BarrierEdgeRegistry registry(
        4,
        {
            {1, 0, true},
            {1, 1, true},
            {2, 0, true},
            {2, 1, false},
            {3, 1, false},
            {3, 2, false},
            {4, 2, false},
            {4, 3, false}
        }
    );
    REQUIRE(registry.fixedLocalizationEdges().size() == 8);
    REQUIRE(registry.collisionEdges().size() == 6);

    std::size_t localizationRows = 0;
    for (const auto& edge : registry.fixedLocalizationEdges()) {
        const Eigen::Vector2d positionI(edge.low, 1.0);
        const Eigen::Vector2d positionJ = edge.baseId >= 0
            ? Eigen::Vector2d(0.0, -edge.baseId - 1.0)
            : Eigen::Vector2d(edge.high, -1.0);
        const auto snapshot = cbf2026::makeEdgeSnapshot(
            edge,
            positionI,
            positionJ,
            0.1,
            edge.baseId >= 0 ? 0.0 : 0.2,
            1.0,
            7,
            11
        );
        const auto rows = cbf2026::allocatedRows(
            snapshot,
            edge.baseId >= 0 ? 1.0 : 0.5,
            edge.baseId >= 0 ? 0.0 : 0.5
        );
        CHECK(rows.size() == (edge.baseId >= 0 ? 1 : 2));
        localizationRows += rows.size();
    }

    std::size_t collisionRows = 0;
    for (const auto& edge : registry.collisionEdges()) {
        const auto snapshot = cbf2026::makeEdgeSnapshot(
            edge,
            Eigen::Vector2d(edge.low, 1.0),
            Eigen::Vector2d(edge.high, -1.0),
            0.1,
            0.2,
            1.0,
            7,
            11
        );
        collisionRows += cbf2026::allocatedRows(
            snapshot, 0.5, 0.5
        ).size();
    }
    CHECK(localizationRows == 13);
    CHECK(collisionRows == 12);

    const auto fimOnly = cbf2026::canonicalUavEdge(
        cbf2026::EdgeKind::Localization, 1, 4
    );
    CHECK(std::find(
        registry.fixedLocalizationEdges().begin(),
        registry.fixedLocalizationEdges().end(),
        fimOnly
    ) == registry.fixedLocalizationEdges().end());

    std::set<cbf2026::EdgeId> uniqueEdges;
    uniqueEdges.insert(
        registry.fixedLocalizationEdges().begin(),
        registry.fixedLocalizationEdges().end()
    );
    uniqueEdges.insert(
        registry.collisionEdges().begin(),
        registry.collisionEdges().end()
    );
    CHECK(uniqueEdges.size() == 14);
}

TEST_CASE("paper topology freezes 28 and 91 barriers with 232 local rows") {
    std::vector<cbf2026::FixedLocalizationReference> references;
    for (int squad = 0; squad < 2; ++squad) {
        const int firstUav = 1 + 7 * squad;
        const int firstBase = 3 * squad;
        references.push_back({firstUav, firstBase, true});
        references.push_back({firstUav, firstBase + 1, true});
        references.push_back({firstUav + 1, firstBase, true});
        for (int localIndex = 2; localIndex <= 7; ++localIndex) {
            const int owner = firstUav + localIndex - 1;
            const int minimumReference = std::max(1, localIndex - 2);
            for (int referenceIndex = minimumReference;
                 referenceIndex < localIndex;
                 ++referenceIndex) {
                references.push_back({
                    owner,
                    firstUav + referenceIndex - 1,
                    false
                });
            }
        }
    }

    const cbf2026::BarrierEdgeRegistry registry(14, references);
    REQUIRE(registry.fixedLocalizationEdges().size() == 28);
    REQUIRE(registry.collisionEdges().size() == 91);

    std::array<int, 14> localizationOwnerRows{};
    int localizationRows = 0;
    for (const auto& edge : registry.fixedLocalizationEdges()) {
        ++localizationOwnerRows[edge.low - 1];
        ++localizationRows;
        if (edge.baseId < 0) {
            ++localizationOwnerRows[edge.high - 1];
            ++localizationRows;
        }
    }
    CHECK(localizationRows == 50);
    const std::array<int, 14> expectedLocalization = {
        4, 4, 4, 4, 4, 3, 2,
        4, 4, 4, 4, 4, 3, 2
    };
    CHECK(localizationOwnerRows == expectedLocalization);

    std::array<int, 14> totalOwnerRows = localizationOwnerRows;
    int collisionRows = 0;
    for (const auto& edge : registry.collisionEdges()) {
        ++totalOwnerRows[edge.low - 1];
        ++totalOwnerRows[edge.high - 1];
        collisionRows += 2;
    }
    const std::array<int, 14> expectedTotal = {
        17, 17, 17, 17, 17, 16, 15,
        17, 17, 17, 17, 17, 16, 15
    };
    CHECK(collisionRows == 182);
    CHECK(localizationRows + collisionRows == 232);
    CHECK(totalOwnerRows == expectedTotal);
    CHECK(registry.fixedLocalizationEdges().size()
          + registry.collisionEdges().size() == 119);
}

TEST_CASE("canonical edge hashing binds kind and every endpoint field") {
    const cbf2026::EdgeId edge = {
        cbf2026::EdgeKind::Localization, 1, 2, -1
    };
    const cbf2026::EdgeIdHash hash;

    auto changed = edge;
    changed.kind = cbf2026::EdgeKind::Collision;
    CHECK(hash(changed) != hash(edge));
    changed = edge;
    changed.low = 3;
    CHECK(hash(changed) != hash(edge));
    changed = edge;
    changed.high = 4;
    CHECK(hash(changed) != hash(edge));
    changed = edge;
    changed.baseId = 0;
    CHECK(hash(changed) != hash(edge));
}

TEST_CASE("registry rejects invalid and duplicated mission edges") {
    CHECK_THROWS_AS(
        cbf2026::BarrierEdgeRegistry(0, {}),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        cbf2026::BarrierEdgeRegistry(
            2,
            {{2, 1, false}, {2, 1, false}}
        ),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        cbf2026::BarrierEdgeRegistry(2, {{1, 2, false}}),
        std::invalid_argument
    );

    const cbf2026::BarrierEdgeRegistry registry(2, {});
    CHECK_THROWS_AS(registry.incidentEdges(0), std::invalid_argument);
    CHECK_THROWS_AS(registry.incidentEdges(3), std::invalid_argument);
}

TEST_CASE("canonical edge APIs reject unsupported edge kinds") {
    const auto unsupported = static_cast<cbf2026::EdgeKind>(2);

    CHECK_THROWS_AS(
        cbf2026::canonicalUavEdge(unsupported, 1, 2),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        cbf2026::validateCanonicalEdge({unsupported, 1, 2, -1}),
        std::invalid_argument
    );
}

TEST_CASE("fixed-reference and endpoint permutations preserve canonical rows") {
    const std::vector<cbf2026::FixedLocalizationReference> references = {
        {1, 0, true},
        {1, 1, true},
        {2, 0, true},
        {2, 1, false},
        {3, 1, false},
        {3, 2, false},
        {4, 2, false},
        {4, 3, false}
    };
    std::vector<cbf2026::FixedLocalizationReference> permuted = references;
    std::reverse(permuted.begin(), permuted.end());

    const cbf2026::BarrierEdgeRegistry canonicalRegistry(4, references);
    const cbf2026::BarrierEdgeRegistry permutedRegistry(4, permuted);
    REQUIRE(canonicalRegistry.fixedLocalizationEdges()
            == permutedRegistry.fixedLocalizationEdges());
    REQUIRE(canonicalRegistry.collisionEdges()
            == permutedRegistry.collisionEdges());

    for (std::size_t index = 0;
         index < canonicalRegistry.fixedLocalizationEdges().size();
         ++index) {
        const auto& canonicalEdge =
            canonicalRegistry.fixedLocalizationEdges()[index];
        const auto& permutedEdge =
            permutedRegistry.fixedLocalizationEdges()[index];
        const Eigen::Vector2d first(canonicalEdge.low, 1.0);
        const Eigen::Vector2d second = canonicalEdge.baseId >= 0
            ? Eigen::Vector2d(0.0, -canonicalEdge.baseId - 1.0)
            : Eigen::Vector2d(canonicalEdge.high, -1.0);
        const auto canonicalSnapshot = cbf2026::makeEdgeSnapshot(
            canonicalEdge, first, second, 0.1,
            canonicalEdge.baseId >= 0 ? 0.0 : 0.2,
            0.7, 7, 11
        );
        const auto permutedSnapshot = cbf2026::makeEdgeSnapshot(
            permutedEdge, first, second, 0.1,
            permutedEdge.baseId >= 0 ? 0.0 : 0.2,
            0.7, 7, 11
        );
        const double allocationI = canonicalEdge.baseId >= 0 ? 1.0 : 0.5;
        const double allocationJ = canonicalEdge.baseId >= 0 ? 0.0 : 0.5;
        const auto canonicalRows = cbf2026::allocatedRows(
            canonicalSnapshot, allocationI, allocationJ
        );
        auto permutedRows = cbf2026::allocatedRows(
            permutedSnapshot, allocationI, allocationJ
        );
        REQUIRE(canonicalRows.size() == permutedRows.size());
        for (std::size_t rowIndex = 0;
             rowIndex < canonicalRows.size();
             ++rowIndex) {
            CHECK(canonicalRows[rowIndex].edge
                  == permutedRows[rowIndex].edge);
            CHECK(canonicalRows[rowIndex].owner
                  == permutedRows[rowIndex].owner);
            CHECK(canonicalRows[rowIndex].coefficient
                  == permutedRows[rowIndex].coefficient);
            CHECK(canonicalRows[rowIndex].constant
                  == permutedRows[rowIndex].constant);
            CHECK(canonicalRows[rowIndex].allocation
                  == permutedRows[rowIndex].allocation);
            CHECK(canonicalRows[rowIndex].snapshotVersion
                  == permutedRows[rowIndex].snapshotVersion);
            CHECK(canonicalRows[rowIndex].allocationVersion
                  == permutedRows[rowIndex].allocationVersion);
        }
        std::reverse(permutedRows.begin(), permutedRows.end());
        const auto canonicalFull =
            cbf2026::reconstructFullRow(canonicalRows);
        const auto permutedFull =
            cbf2026::reconstructFullRow(permutedRows);
        CHECK(canonicalFull.edge == permutedFull.edge);
        CHECK(canonicalFull.coefficientI == permutedFull.coefficientI);
        CHECK(canonicalFull.coefficientJ == permutedFull.coefficientJ);
        CHECK(canonicalFull.constant == permutedFull.constant);
        CHECK(canonicalFull.snapshotVersion
              == permutedFull.snapshotVersion);
        CHECK(canonicalFull.allocationVersion
              == permutedFull.allocationVersion);
    }
}
