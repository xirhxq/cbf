#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "cbf/FimRateCertificate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace {

struct RelativeKinematics {
    double distance;
    Eigen::Vector2d direction;
    Eigen::Vector2d dotDirection;
};

RelativeKinematics relativeKinematics(
    const Eigen::Vector2d& receiverPosition,
    const Eigen::Vector2d& referencePosition,
    const Eigen::Vector2d& receiverVelocity,
    const Eigen::Vector2d& referenceVelocity
) {
    const Eigen::Vector2d displacement =
        receiverPosition - referencePosition;
    const double distance = displacement.norm();
    const Eigen::Vector2d direction = displacement / distance;
    const Eigen::Vector2d relativeVelocity =
        receiverVelocity - referenceVelocity;
    const Eigen::Vector2d dotDirection = (
        Eigen::Matrix2d::Identity()
        - direction * direction.transpose()
    ) * relativeVelocity / distance;
    return {distance, direction, dotDirection};
}

Eigen::Matrix2d analyticDotInformation(
    const cbf2026::NodeRateCertificate& certificate,
    const std::vector<cbf2026::ReferenceRealizedDerivative>& derivatives
) {
    Eigen::Matrix2d result = Eigen::Matrix2d::Zero();
    for (const auto& term : certificate.frozenReferences) {
        const auto derivative = std::find_if(
            derivatives.begin(),
            derivatives.end(),
            [&term](const auto& candidate) {
                return candidate.referenceId == term.referenceId;
            }
        );
        REQUIRE(derivative != derivatives.end());
        result +=
            -derivative->dotEffectiveVariance
             / std::pow(term.effectiveVariance, 2)
             * (term.direction * term.direction.transpose())
            + (
                derivative->dotDirection * term.direction.transpose()
                + term.direction * derivative->dotDirection.transpose()
            ) / term.effectiveVariance;
    }
    return result;
}

}  // namespace

TEST_CASE("hovering base has zero covariance and epsilon rate bounds") {
    const auto base = cbf2026::baseRateCertificate(0, 7);
    CHECK(base.covarianceRateBound == doctest::Approx(0.0));
    CHECK(base.epsilonRateBound == doctest::Approx(0.0));
    CHECK(base.snapshotVersion == 7);
    CHECK(cbf2026::realizedEpsilonRate(base, {})
          == doctest::Approx(0.0));
    CHECK_THROWS_AS(
        cbf2026::baseRateCertificate(-1, 7),
        std::invalid_argument
    );
}

TEST_CASE("two orthogonal hovering references match the hand-computed recursion") {
    constexpr std::uint64_t version = 9;
    const auto firstBase = cbf2026::baseRateCertificate(100, version);
    const auto secondBase = cbf2026::baseRateCertificate(101, version);
    const cbf2026::NodeRateInput input{
        4,
        3,
        version,
        25.0,
        {
            {
                {cbf2026::canonicalBaseReferenceId(100), 0, version, true,
                 firstBase.covariance,
                 firstBase.covarianceRateBound, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {cbf2026::canonicalBaseReferenceId(101), 0, version, true,
                 secondBase.covariance,
                 secondBase.covarianceRateBound, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            }
        }
    };

    const double expectedW = 1.0;
    const double expectedBeta = std::sqrt(2.0) * 25.0 / 10.0;
    const double expectedDotW = 0.0;
    const double expectedLPhi = 2.0 * (
        expectedDotW / std::pow(expectedW, 2)
        + 2.0 * expectedBeta / expectedW
    );
    const double expectedLP = expectedLPhi;
    const double expectedBarNu = 3.0 * expectedLP / 2.0;

    const auto certificate = cbf2026::computeNodeRateCertificate(input);

    CHECK(certificate.robotId == 4);
    CHECK(certificate.snapshotVersion == version);
    CHECK(certificate.information.isApprox(Eigen::Matrix2d::Identity(), 1e-12));
    CHECK(certificate.covariance.isApprox(Eigen::Matrix2d::Identity(), 1e-12));
    CHECK(certificate.epsilon == doctest::Approx(3.0).epsilon(1e-12));
    CHECK(certificate.informationRateBound
          == doctest::Approx(expectedLPhi).epsilon(1e-12));
    CHECK(certificate.covarianceRateBound
          == doctest::Approx(expectedLP).epsilon(1e-12));
    CHECK(certificate.epsilonRateBound
          == doctest::Approx(expectedBarNu).epsilon(1e-12));
    REQUIRE(certificate.frozenReferences.size() == 2);
    CHECK(certificate.frozenReferences[0].effectiveVariance
          == doctest::Approx(expectedW).epsilon(1e-12));
    CHECK(certificate.frozenReferences[1].effectiveVariance
          == doctest::Approx(expectedW).epsilon(1e-12));
}

TEST_CASE("anisotropic predecessor and nonunit range sigma use directional variance") {
    constexpr std::uint64_t version = 10;
    const Eigen::Matrix2d predecessorCovariance = (
        Eigen::Matrix2d() << 4.0, 0.0, 0.0, 9.0
    ).finished();
    const cbf2026::NodeRateInput input{
        4,
        2,
        version,
        25.0,
        {
            {
                {cbf2026::canonicalBaseReferenceId(0), 0, version, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                2.0
            },
            {
                {cbf2026::canonicalUavReferenceId(1), 1, version, false,
                 predecessorCovariance, 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                4.0
            }
        }
    };
    const double beta = std::sqrt(2.0) * 25.0 / 10.0;
    const double expectedLPhi =
        (18.0 * beta) / 64.0 + (2.0 * beta) / 8.0 + beta;
    const double expectedLP = 64.0 * expectedLPhi;
    const double expectedBarNu =
        3.0 * expectedLP / (2.0 * std::sqrt(8.0));

    const auto certificate = cbf2026::computeNodeRateCertificate(input);

    REQUIRE(certificate.frozenReferences.size() == 2);
    CHECK(certificate.frozenReferences[0].effectiveVariance
          == doctest::Approx(2.0).epsilon(1e-12));
    CHECK(certificate.frozenReferences[1].effectiveVariance
          == doctest::Approx(8.0).epsilon(1e-12));
    CHECK(certificate.information.isApprox(
        (Eigen::Matrix2d() << 0.125, 0.0, 0.0, 0.5).finished(),
        1e-12
    ));
    CHECK(certificate.covariance.isApprox(
        (Eigen::Matrix2d() << 8.0, 0.0, 0.0, 2.0).finished(),
        1e-12
    ));
    CHECK(certificate.epsilon
          == doctest::Approx(3.0 * std::sqrt(8.0)).epsilon(1e-12));
    CHECK(certificate.informationRateBound
          == doctest::Approx(expectedLPhi).epsilon(1e-12));
    CHECK(certificate.covarianceRateBound
          == doctest::Approx(expectedLP).epsilon(1e-12));
    CHECK(certificate.epsilonRateBound
          == doctest::Approx(expectedBarNu).epsilon(1e-12));
}

TEST_CASE("node certificate rejects non-topological predecessor snapshots") {
    constexpr std::uint64_t version = 12;
    cbf2026::NodeRateInput input{
        4,
        3,
        version,
        25.0,
        {
            {
                {cbf2026::canonicalBaseReferenceId(100), 0, version, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            },
            {
                {1, 1, version, false, Eigen::Matrix2d::Zero(), 0.0, 1.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            }
        }
    };

    SUBCASE("snapshot version differs") {
        input.references[1].predecessor.snapshotVersion = version - 1;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("non-base local index is not lower") {
        input.references[1].predecessor.localIndex = input.localIndex;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("non-base predecessor local index is not positive") {
        input.references[1].predecessor.localIndex = 0;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }
}

TEST_CASE("node certificate rejects noncanonical reference ordering") {
    constexpr std::uint64_t version = 14;
    cbf2026::NodeRateInput input{
        4,
        3,
        version,
        25.0,
        {
            {
                {cbf2026::canonicalUavReferenceId(1), 1, version, false,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {cbf2026::canonicalBaseReferenceId(0), 0, version, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            }
        }
    };

    SUBCASE("UAV precedes base") {
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("UAV local indices decrease") {
        input.references = {
            {
                {cbf2026::canonicalBaseReferenceId(0), 0, version, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {cbf2026::canonicalUavReferenceId(2), 2, version, false,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            },
            {
                {cbf2026::canonicalUavReferenceId(1), 1, version, false,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d(1.0, 1.0).normalized(),
                1.0
            }
        };
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }
}

TEST_CASE("node certificate rejects invalid numeric reference data") {
    constexpr std::uint64_t version = 15;
    cbf2026::NodeRateInput input{
        4,
        3,
        version,
        25.0,
        {
            {
                {cbf2026::canonicalBaseReferenceId(100), 0, version, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {cbf2026::canonicalBaseReferenceId(101), 0, version, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            }
        }
    };

    SUBCASE("component bound is nonfinite") {
        input.planarComponentMax = std::numeric_limits<double>::infinity();
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("component bound is zero") {
        input.planarComponentMax = 0.0;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("node local index is not positive") {
        input.localIndex = 0;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("base local index is not zero") {
        input.references[0].predecessor.localIndex = 1;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("distance is at the singular threshold") {
        input.references[0].distance = 1e-8;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("ranging variance is nonpositive") {
        input.references[0].rangingVariance = 0.0;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("direction is not a finite unit vector") {
        input.references[0].direction.x() =
            std::numeric_limits<double>::quiet_NaN();
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("predecessor covariance rate bound is negative") {
        input.references[0].predecessor.covarianceRateBound = -1.0;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("predecessor speed bound is negative") {
        input.references[0].predecessor.speedBound = -1.0;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("reference identities are duplicated") {
        input.references[1].predecessor.referenceId =
            input.references[0].predecessor.referenceId;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("hovering base uses a UAV identity") {
        input.references[0].predecessor.referenceId =
            cbf2026::canonicalUavReferenceId(100);
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("UAV uses a base identity") {
        input.references[0].predecessor.hoveringBase = false;
        input.references[0].predecessor.localIndex = 1;
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }
}

TEST_CASE("node certificate rejects singular and ill-conditioned information") {
    constexpr std::uint64_t version = 16;
    cbf2026::NodeRateInput input{
        4,
        3,
        version,
        25.0,
        {
            {
                {cbf2026::canonicalBaseReferenceId(100), 0, version, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {cbf2026::canonicalBaseReferenceId(101), 0, version, true,
                 Eigen::Matrix2d::Zero(), 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            }
        }
    };

    SUBCASE("information is singular") {
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }

    SUBCASE("information exceeds the frozen condition limit") {
        input.references[1].direction = Eigen::Vector2d(1.0, 1e-7).normalized();
        CHECK_THROWS_AS(
            cbf2026::computeNodeRateCertificate(input),
            std::invalid_argument
        );
    }
}

TEST_CASE("base and UAV reference identities use disjoint canonical encodings") {
    const int baseIdentity = cbf2026::canonicalBaseReferenceId(1);
    const int uavIdentity = cbf2026::canonicalUavReferenceId(1);

    CHECK(baseIdentity < 0);
    CHECK(uavIdentity >= 0);
    CHECK(baseIdentity != uavIdentity);
    CHECK_THROWS_AS(
        cbf2026::canonicalBaseReferenceId(-1),
        std::invalid_argument
    );
    CHECK_THROWS_AS(
        cbf2026::canonicalUavReferenceId(-1),
        std::invalid_argument
    );
}

TEST_CASE("realized epsilon rate reconstructs dot Phi by canonical identity") {
    constexpr std::uint64_t version = 18;
    const int baseIdentity = cbf2026::canonicalBaseReferenceId(1);
    const int uavIdentity = cbf2026::canonicalUavReferenceId(1);
    const cbf2026::NodeRateInput input{
        4,
        3,
        version,
        25.0,
        {
            {
                {baseIdentity, 0, version, true, Eigen::Matrix2d::Zero(),
                 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {uavIdentity, 2, version, false, Eigen::Matrix2d::Zero(),
                 0.0, 1.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            }
        }
    };
    const auto certificate = cbf2026::computeNodeRateCertificate(input);
    const std::vector<cbf2026::ReferenceRealizedDerivative> derivatives{
        {uavIdentity, -0.1, Eigen::Vector2d(-0.4, 0.0)},
        {baseIdentity, 0.2, Eigen::Vector2d(0.0, 0.3)}
    };
    const double expectedDotPhiNorm = (1.0 + std::sqrt(13.0)) / 20.0;
    const double expectedRealizedRate = 1.5 * expectedDotPhiNorm;

    CHECK(cbf2026::realizedEpsilonRate(certificate, derivatives)
          == doctest::Approx(expectedRealizedRate).epsilon(1e-12));
}

TEST_CASE("realized epsilon rate requires exactly one derivative per frozen term") {
    constexpr std::uint64_t version = 19;
    const int firstIdentity = cbf2026::canonicalBaseReferenceId(0);
    const int secondIdentity = cbf2026::canonicalBaseReferenceId(1);
    const cbf2026::NodeRateInput input{
        4,
        3,
        version,
        25.0,
        {
            {
                {firstIdentity, 0, version, true, Eigen::Matrix2d::Zero(),
                 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitX(),
                1.0
            },
            {
                {secondIdentity, 0, version, true, Eigen::Matrix2d::Zero(),
                 0.0, 0.0},
                10.0,
                Eigen::Vector2d::UnitY(),
                1.0
            }
        }
    };
    const auto certificate = cbf2026::computeNodeRateCertificate(input);
    std::vector<cbf2026::ReferenceRealizedDerivative> derivatives{
        {firstIdentity, 0.0, Eigen::Vector2d::Zero()},
        {secondIdentity, 0.0, Eigen::Vector2d::Zero()}
    };

    SUBCASE("missing") {
        derivatives.pop_back();
        CHECK_THROWS_AS(
            cbf2026::realizedEpsilonRate(certificate, derivatives),
            std::invalid_argument
        );
    }

    SUBCASE("duplicate") {
        derivatives.push_back(derivatives.front());
        CHECK_THROWS_AS(
            cbf2026::realizedEpsilonRate(certificate, derivatives),
            std::invalid_argument
        );
    }

    SUBCASE("extra") {
        derivatives.push_back({999, 0.0, Eigen::Vector2d::Zero()});
        CHECK_THROWS_AS(
            cbf2026::realizedEpsilonRate(certificate, derivatives),
            std::invalid_argument
        );
    }
}

TEST_CASE("topological realized rates match finite perturbations and stay bounded") {
    constexpr double componentMax = 25.0;
    constexpr double step = 1e-7;
    constexpr double finiteDifferenceTolerance = 1e-7;
    constexpr double realizedBoundTolerance = 1e-9;
    constexpr std::uint64_t version = 23;
    const double uavSpeedBound = std::sqrt(2.0) * componentMax;

    for (unsigned int seed = 11; seed <= 30; ++seed) {
        CAPTURE(seed);
        std::mt19937 generator(seed);
        std::uniform_real_distribution<double> coordinate(-20.0, 20.0);
        std::uniform_real_distribution<double> velocity(-componentMax, componentMax);
        std::uniform_real_distribution<double> angle(-3.0, 3.0);
        std::uniform_real_distribution<double> distance(8.0, 24.0);
        std::uniform_real_distribution<double> variance(0.5, 2.0);

        const Eigen::Vector2d pA(coordinate(generator), coordinate(generator));
        Eigen::Vector2d vA(velocity(generator), velocity(generator));
        Eigen::Vector2d vB(velocity(generator), velocity(generator));
        if (seed == 11) {
            vA = Eigen::Vector2d(componentMax, -componentMax);
            vB = Eigen::Vector2d(-componentMax, componentMax);
        } else if (seed == 12) {
            vA = Eigen::Vector2d(-componentMax, componentMax);
            vB = Eigen::Vector2d(componentMax, -componentMax);
        }
        const double thetaA = angle(generator);
        const Eigen::Vector2d nA0(std::cos(thetaA), std::sin(thetaA));
        const Eigen::Vector2d nA1(
            std::cos(thetaA + 1.2),
            std::sin(thetaA + 1.2)
        );
        const Eigen::Vector2d base0 = pA - distance(generator) * nA0;
        const Eigen::Vector2d base1 = pA - distance(generator) * nA1;

        const double thetaB = angle(generator);
        const Eigen::Vector2d nBA(std::cos(thetaB), std::sin(thetaB));
        const Eigen::Vector2d pB = pA + distance(generator) * nBA;
        const Eigen::Vector2d nBBase(
            std::cos(thetaB + 1.1),
            std::sin(thetaB + 1.1)
        );
        const Eigen::Vector2d base2 = pB - distance(generator) * nBBase;
        const double rangingVarianceA0 = variance(generator);
        const double rangingVarianceA1 = variance(generator);
        const double rangingVarianceBA = variance(generator);
        const double rangingVarianceBBase = variance(generator);

        auto certificateAAt = [&](double time) {
            const Eigen::Vector2d currentA = pA + time * vA;
            const auto first = relativeKinematics(
                currentA, base0, vA, Eigen::Vector2d::Zero()
            );
            const auto second = relativeKinematics(
                currentA, base1, vA, Eigen::Vector2d::Zero()
            );
            return cbf2026::computeNodeRateCertificate({
                1,
                1,
                version,
                componentMax,
                {
                    {
                        {cbf2026::canonicalBaseReferenceId(0), 0, version,
                         true, Eigen::Matrix2d::Zero(), 0.0, 0.0},
                        first.distance,
                        first.direction,
                        rangingVarianceA0
                    },
                    {
                        {cbf2026::canonicalBaseReferenceId(1), 0, version,
                         true, Eigen::Matrix2d::Zero(), 0.0, 0.0},
                        second.distance,
                        second.direction,
                        rangingVarianceA1
                    }
                }
            });
        };

        auto certificateBAt = [&] (
            double time,
            const cbf2026::NodeRateCertificate& certificateA
        ) {
            const Eigen::Vector2d currentA = pA + time * vA;
            const Eigen::Vector2d currentB = pB + time * vB;
            const auto first = relativeKinematics(
                currentB, currentA, vB, vA
            );
            const auto second = relativeKinematics(
                currentB, base2, vB, Eigen::Vector2d::Zero()
            );
            return cbf2026::computeNodeRateCertificate({
                2,
                2,
                version,
                componentMax,
                {
                    {
                        {cbf2026::canonicalBaseReferenceId(2), 0, version,
                         true, Eigen::Matrix2d::Zero(), 0.0, 0.0},
                        second.distance,
                        second.direction,
                        rangingVarianceBBase
                    },
                    {
                        {cbf2026::canonicalUavReferenceId(1), 1, version,
                         false, certificateA.covariance,
                         certificateA.covarianceRateBound, uavSpeedBound},
                        first.distance,
                        first.direction,
                        rangingVarianceBA
                    }
                }
            });
        };

        const auto certificateA = certificateAAt(0.0);
        const auto plusA = certificateAAt(step);
        const auto minusA = certificateAAt(-step);
        const auto relativeA0 = relativeKinematics(
            pA, base0, vA, Eigen::Vector2d::Zero()
        );
        const auto relativeA1 = relativeKinematics(
            pA, base1, vA, Eigen::Vector2d::Zero()
        );
        const std::vector<cbf2026::ReferenceRealizedDerivative> derivativesA{
            {
                cbf2026::canonicalBaseReferenceId(0),
                0.0,
                relativeA0.dotDirection
            },
            {
                cbf2026::canonicalBaseReferenceId(1),
                0.0,
                relativeA1.dotDirection
            }
        };
        const Eigen::Matrix2d analyticDotPhiA =
            analyticDotInformation(certificateA, derivativesA);
        const Eigen::Matrix2d finiteDotPhiA =
            (plusA.information - minusA.information) / (2.0 * step);
        CHECK((finiteDotPhiA - analyticDotPhiA).cwiseAbs().maxCoeff()
              <= finiteDifferenceTolerance);
        CHECK(cbf2026::realizedEpsilonRate(certificateA, derivativesA)
              <= certificateA.epsilonRateBound + realizedBoundTolerance);

        const Eigen::Matrix2d dotCovarianceA =
            -certificateA.covariance
             * analyticDotPhiA
             * certificateA.covariance;
        const auto certificateB = certificateBAt(0.0, certificateA);
        const auto plusB = certificateBAt(step, plusA);
        const auto minusB = certificateBAt(-step, minusA);
        const auto relativeBA = relativeKinematics(pB, pA, vB, vA);
        const auto relativeBBase = relativeKinematics(
            pB, base2, vB, Eigen::Vector2d::Zero()
        );
        const double dotEffectiveVarianceBA =
            2.0 * relativeBA.dotDirection.dot(
                certificateA.covariance * relativeBA.direction
            )
            + relativeBA.direction.dot(
                dotCovarianceA * relativeBA.direction
            );
        const std::vector<cbf2026::ReferenceRealizedDerivative> derivativesB{
            {
                cbf2026::canonicalUavReferenceId(1),
                dotEffectiveVarianceBA,
                relativeBA.dotDirection
            },
            {
                cbf2026::canonicalBaseReferenceId(2),
                0.0,
                relativeBBase.dotDirection
            }
        };
        const Eigen::Matrix2d analyticDotPhiB =
            analyticDotInformation(certificateB, derivativesB);
        const Eigen::Matrix2d finiteDotPhiB =
            (plusB.information - minusB.information) / (2.0 * step);
        CHECK((finiteDotPhiB - analyticDotPhiB).cwiseAbs().maxCoeff()
              <= finiteDifferenceTolerance);
        CHECK(cbf2026::realizedEpsilonRate(certificateB, derivativesB)
              <= certificateB.epsilonRateBound + realizedBoundTolerance);
    }
}
