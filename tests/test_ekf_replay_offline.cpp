// Offline C++ EKF replay on a recorded data.json trajectory.
//
// Reuses the same EkfEstimator class that runs in-process inside Swarm, so it
// validates the C++ EKF directly (not the Python reference). Reads truth + formation
// from data.json, builds references each frame, runs the EKF, and reports
// containment / error / epsilon / NEES statistics — the same metrics as
// scripts/replay_estimator_offline.py.
//
// Build: g++ -std=c++17 -O2 -I<cbf>/include -I<cbf>/external tests/test_ekf_replay_offline.cpp -o /tmp/ekf_replay
// Run:   /tmp/ekf_replay <data.json> [process_noise_mps] [> stats.txt]

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "ComputingGeometry/Point.hpp"
#include "estimator/EkfEstimator.hpp"

// Minimal nlohmann::json forward — we only need to parse the data.json we
// produced, so reuse the bundled json header.
#include "nlohmann/json.hpp"

using json = nlohmann::json;

static double percentile(std::vector<double> v, double pct) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t idx = (size_t)std::round((pct / 100.0) * (v.size() - 1));
    return v[idx];
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <data.json> [process_noise_mps=10] [anchor_kappa=3]\n";
        return 1;
    }
    double q = (argc > 2) ? std::stod(argv[2]) : 10.0;
    double kappa = (argc > 3) ? std::stod(argv[3]) : 3.0;
    std::cerr << "starting ekf replay: " << argv[1] << " q=" << q << " kappa=" << kappa << "\n";

    json d = json::parse(std::ifstream(argv[1]));
    auto& states = d["state"];
    int nFrames = (int)states.size();
    int nRobots = (int)states[0]["robots"].size();

    // bases
    std::vector<Point> bases;
    for (auto& b : d["config"]["bases"]) bases.emplace_back(b[0].get<double>(), b[1].get<double>());

    // deployment = frame-0 truth
    std::vector<Point> deployment;
    for (auto& r : states[0]["robots"]) deployment.emplace_back(
        r["state"]["x"].get<double>(), r["state"]["y"].get<double>());

    EkfEstimator::Params p;
    p.process_noise_mps = q;
    p.anchor_covariance_scale = kappa;
    p.dt = d["config"]["execute"]["time-step"].get<double>();
    EkfEstimator ekf(bases, deployment, p);

    // stats accumulators
    std::vector<std::vector<double>> errByRobot(nRobots), epsByRobot(nRobots), neesByRobot(nRobots);
    std::vector<long> freshCount(nRobots, 0), coastCount(nRobots, 0);
    long refCountTotal = 0;

    for (int f = 0; f < nFrames; ++f) {
        auto& fr = states[f];
        auto& formation = fr["formation"];   // frame-level list, one entry per robot
        auto& covFormation = fr["covariance_formation"];
        auto& robots = fr["robots"];

        std::vector<std::vector<EkfEstimator::Reference>> refsByRobot(nRobots);
        std::vector<std::pair<double, double>> vel(nRobots, {0.0, 0.0});

        // held command = previous frame opt.result
        for (int i = 0; i < nRobots; ++i) {
            if (f > 0) {
                auto& prev = states[f - 1]["robots"][i]["opt"]["result"];
                vel[i] = {prev.value("vx", 0.0), prev.value("vy", 0.0)};
            }
            // references from formation (frame-level, has baseIds/anchorIds).
            // NOTE: covariance_formation in data.json is often {} (not populated
            // at logOnce time), but formation has the actual topology.
            auto& cf = formation[i];
            if (cf.contains("baseIds")) {
                for (auto& bi : cf["baseIds"]) {
                    int b = bi.get<int>();
                    if (b >= 0 && b < (int)bases.size())
                        refsByRobot[i].push_back({true, b, bases[b]});
                }
            }
            if (cf.contains("anchorIds")) {
                for (auto& ai : cf["anchorIds"]) {
                    int a = ai.get<int>();
                    if (a >= 1 && a <= nRobots) {
                        auto& anchor = robots[a - 1]["state"];
                        refsByRobot[i].push_back({false, a, Point(anchor["x"].get<double>(), anchor["y"].get<double>())});
                    }
                }
            }
            refCountTotal += (long)refsByRobot[i].size();
        }

        // truth positions for this frame (for measurement generation)
        std::vector<Point> truthPos(nRobots);
        for (int i = 0; i < nRobots; ++i) {
            truthPos[i] = Point(robots[i]["state"]["x"].get<double>(),
                                robots[i]["state"]["y"].get<double>());
        }

        ekf.step(refsByRobot, vel, truthPos);
        if (f == 0) {
            std::cerr << "frame 0: refs/robot =";
            for (int i = 0; i < nRobots; ++i) std::cerr << " " << refsByRobot[i].size();
            std::cerr << "\n";
        }

        for (int i = 0; i < nRobots; ++i) {
            int rid = robots[i]["id"].get<int>();
            double tx = robots[i]["state"]["x"].get<double>();
            double ty = robots[i]["state"]["y"].get<double>();
            Point est = ekf.estimate(rid);
            double eps = ekf.epsilon(rid);
            double err = std::hypot(tx - est.x, ty - est.y);
            double lam = (eps / 3.0) * (eps / 3.0);
            double nees = (lam > 1e-12)
                ? ((Eigen::Vector2d(tx - est.x, ty - est.y)).squaredNorm() / lam) / 2.0
                : 0.0;  // 2-DoF normalized
            errByRobot[i].push_back(err);
            epsByRobot[i].push_back(eps);
            neesByRobot[i].push_back(nees);
            if (ekf.fresh(rid)) ++freshCount[i]; else ++coastCount[i];
        }
    }

    // pooled
    std::vector<double> allErr, allEps, allNees;
    for (int i = 0; i < nRobots; ++i) {
        allErr.insert(allErr.end(), errByRobot[i].begin(), errByRobot[i].end());
        allEps.insert(allEps.end(), epsByRobot[i].begin(), epsByRobot[i].end());
        allNees.insert(allNees.end(), neesByRobot[i].begin(), neesByRobot[i].end());
    }
    long total = (long)allErr.size();
    long within1e = 0, within3e = 0;
    for (size_t k = 0; k < allErr.size() && k < allEps.size(); ++k) {
        if (allErr[k] <= allEps[k]) ++within1e;
        if (allErr[k] <= 3.0 * allEps[k]) ++within3e;
    }

    std::cout << "=== C++ EKF offline replay ===\n";
    std::cout << "frames=" << nFrames << " robots=" << nRobots << " q=" << q << " kappa=" << kappa << "\n";
    std::cout << "avg refs/robot/frame=" << (double)refCountTotal / (nFrames * nRobots) << "\n";
    std::cout << "containment: |err|<=eps=" << (double)within1e/total
              << "  |err|<=3eps=" << (double)within3e/total << "\n";
    std::cout << "error(m): p50=" << percentile(allErr, 50)
              << " p95=" << percentile(allErr, 95)
              << " max=" << percentile(allErr, 100) << "\n";
    std::cout << "epsilon(m): mean=" << (std::accumulate(allEps.begin(), allEps.end(), 0.0)/allEps.size())
              << " p95=" << percentile(allEps, 95)
              << " max=" << percentile(allEps, 100) << "\n";
    std::cout << "NEES: mean=" << (std::accumulate(allNees.begin(), allNees.end(), 0.0)/allNees.size())
              << " p95=" << percentile(allNees, 95) << "\n\n";

    std::cout << "=== per-robot ===\n";
    for (int i = 0; i < nRobots; ++i) {
        int rid = i + 1;
        long we = 0;
        for (size_t k = 0; k < errByRobot[i].size() && k < epsByRobot[i].size(); ++k) {
            if (errByRobot[i][k] <= epsByRobot[i][k]) ++we;
        }
        std::cout << "#" << rid
                  << " err p50/p95/max=" << percentile(errByRobot[i], 50) << "/"
                  << percentile(errByRobot[i], 95) << "/" << percentile(errByRobot[i], 100)
                  << " eps_mean=" << (std::accumulate(epsByRobot[i].begin(), epsByRobot[i].end(), 0.0)/epsByRobot[i].size())
                  << " cont1e=" << (double)we/(long)errByRobot[i].size()
                  << " fresh=" << freshCount[i] << " coast=" << coastCount[i] << "\n";
    }
    return 0;
}
