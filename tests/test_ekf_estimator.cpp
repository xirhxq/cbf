// Standalone comparison test for EkfEstimator (C++) vs the Python reference
// (scripts/diagnostics/ekf_estimator_service.py). Feed both the same bases,
// deployment, references, and commanded velocities with a fixed RNG seed, then
// compare the resulting estimates and epsilons.
//
// Build: see scripts/compare_ekf.sh (compiles this against cbf headers).
// Run:   ./test_ekf_estimator
//
// It prints, for frames 0..N-1, each robot's estimate (x,y) and epsilon so the
// driver can diff against the Python output captured with the same inputs.

#include <cstdio>
#include <vector>
#include "estimator/EkfEstimator.hpp"
#include "ComputingGeometry/Point.hpp"

int main() {
    std::vector<Point> bases = {{-1550, -300}, {-1550, 0}, {-1550, 300}};
    // 3 robots at distinct deployment points.
    std::vector<Point> deployment = {{-1490, -120}, {-1470, 120}, {-1450, 0}};

    EkfEstimator::Params p;
    p.sigma0 = 0.5;
    p.range0 = 850.0;
    p.process_noise_mps = 10.0;
    p.p0_std = 1.0;
    p.dt = 0.5;
    p.innovation_gate = 3.0;
    p.anchor_covariance_scale = 3.0;
    p.seed = 2026081301u;

    EkfEstimator ekf(bases, deployment, p);

    // Simulated truth trajectory: move each robot +x over frames.
    int nFrames = 5;
    int nRobots = (int)deployment.size();
    double vx = 5.0, vy = 2.0;

    // truth positions advance each frame
    std::vector<Point> truth(nRobots);
    for (int r = 0; r < nRobots; ++r) truth[r] = deployment[r];

    for (int f = 0; f < nFrames; ++f) {
        // advance truth by the commanded velocity (used to generate ranges)
        for (int r = 0; r < nRobots; ++r) {
            truth[r].x += vx * p.dt;
            truth[r].y += vy * p.dt;
        }
        // references: robot 0 -> bases {0,1}; robot 1 -> base {1} + uav {1};
        // robot 2 -> uav {1, 2}.
        std::vector<std::vector<EkfEstimator::Reference>> refs(nRobots);
        refs[0].push_back({true, 0, bases[0]});
        refs[0].push_back({true, 1, bases[1]});
        refs[1].push_back({true, 1, bases[1]});
        refs[1].push_back({false, 1, truth[0]});
        refs[2].push_back({false, 1, truth[0]});
        refs[2].push_back({false, 2, truth[1]});

        std::vector<std::pair<double, double>> vel(nRobots, {vx, vy});
        std::vector<Point> truthVec(truth.begin(), truth.end());
        ekf.step(refs, vel, truthVec);

        for (int r = 0; r < nRobots; ++r) {
            Point est = ekf.estimate(r + 1);
            double eps = ekf.epsilon(r + 1);
            printf("frame %d robot %d  est=(%.6f, %.6f) eps=%.6f truth=(%.2f,%.2f) fresh=%d\n",
                   f, r + 1, est.x, est.y, eps, truth[r].x, truth[r].y,
                   ekf.fresh(r + 1) ? 1 : 0);
        }
    }
    return 0;
}
