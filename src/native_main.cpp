#include "Swarm.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

namespace {

json softRows(Robot& r) {
    json rows = json::array();
    auto f = r.model->f();
    auto g = r.model->g();
    auto x = r.taskEvaluationState();
    for (auto& item : r.cbfSlack) {
        const auto& name = item.first;
        auto& cbf = item.second;
        auto a = cbf.constraintUCoe(f, g, x, r.runtime);
        const double c = cbf.constraintConstWithoutTime(f, g, x, r.runtime);
        rows.push_back({{"name", name}, {"coe", {a[0], a[1], a[2]}}, {"const", c}});
    }
    return rows;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            throw std::runtime_error("usage: cbf2026_native <config.json>");
        }
        std::ifstream input(argv[1]);
        if (!input) {
            throw std::runtime_error(std::string("failed to open config: ") + argv[1]);
        }
        json cfg;
        input >> cfg;

        if (!cfg.contains("work2") ||
            cfg.at("tvt-experiment").at("row-mode") != "symmetric-l1" ||
            cfg.at("cbfs").at("route1").at("joint-resolve").get<bool>()) {
            throw std::runtime_error("unexpected sampled-hold symmetric-l1 protocol config");
        }

        Swarm swarm(cfg);

        if (cfg.contains("comparator")) {
            const auto method = cfg.at("comparator").at("method").get<std::string>();
            const auto directory = cfg.at("comparator").at("ipc_directory").get<std::string>();
            if (method != "native" && method != "cvt-target" && method != "gdms") {
                swarm.comparatorControl = [&, method, directory](std::uint64_t frame) {
                    const auto start = std::chrono::steady_clock::now();
                    json request = {{"frame", frame}, {"time_s", swarm.robots.front()->runtime}, {"nodes", json::array()}};
                    for (auto& rp : swarm.robots) {
                        auto& r = *rp;
                        r.comparatorTarget = json::object();
                        r.setupFormation();
                        r.setCVTCBF(r.settings.at("cbfs").at("with-slack"));
                        const auto p = r.taskPosition();
                        const auto q = r.cvt.ct[r.id];
                        auto held = r.model->getControlInput();
                        request["nodes"].push_back({
                            {"id", r.id},
                            {"position", {p.x, p.y}},
                            {"epsilon", r.estimatorEpsilon},
                            {"yaw", r.model->getStateVariable("yawRad")},
                            {"held_command", {held[0], held[1], held[2]}},
                            {"formation", r.myFormation},
                            {"goal", {q.x, q.y}},
                            {"leader", r.id == r.endInMyPart},
                            {"soft", softRows(r)}
                        });
                    }
                    {
                        std::ofstream out(directory + "/control-state.tmp");
                        out << request.dump() << '\n';
                    }
                    std::rename((directory + "/control-state.tmp").c_str(),
                                (directory + "/control-state.json").c_str());
                    {
                        std::ofstream ready(directory + "/control-state.ready");
                        ready << frame;
                    }
                    const auto responsePath = directory + "/control-response.json";
                    while (!swarm.estimatorFileExists(responsePath)) {
                        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > 180.) {
                            throw std::runtime_error("external comparator response timeout");
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                    json response;
                    {
                        std::ifstream stream(responsePath);
                        stream >> response;
                    }
                    std::remove(responsePath.c_str());
                    if (response.at("frame") != frame) {
                        throw std::runtime_error("stale comparator response");
                    }
                    if (response.at("status") != "success") {
                        for (auto& r : swarm.robots) {
                            r->opt = {{"status", "comparator-failure"}, {"comparator", response}};
                        }
                        throw std::runtime_error("external comparator rejected; see controller log");
                    }
                    if (response.contains("targets")) {
                        for (size_t i = 0; i < swarm.robots.size(); ++i) {
                            swarm.robots[i]->comparatorTarget = {
                                {"frame", frame},
                                {"position", response.at("targets").at(i)},
                                {"clip", method == "cano"}
                            };
                        }
                        return false;
                    }
                    if (!response.contains("commands")) {
                        return false;
                    }
                    auto diag = response.value("diagnostics", json::object());
                    diag.erase("raw_solver_candidate");
                    diag.erase("hard_residuals");
                    diag.erase("slacks");
                    const double U = cfg.at("cbfs").at("input-limits").at("planar-component-max");
                    const double W = cfg.at("cbfs").at("input-limits").at("yaw-rate-max");
                    for (size_t i = 0; i < swarm.robots.size(); ++i) {
                        auto& r = *swarm.robots[i];
                        Eigen::VectorXd u = Eigen::VectorXd::Zero(3);
                        for (int a = 0; a < 3; ++a) {
                            u[a] = response.at("commands").at(i).at(a).get<double>();
                        }
                        if (!u.allFinite() || std::abs(u[0]) > U + 1e-7 ||
                            std::abs(u[1]) > U + 1e-7 || std::abs(u[2]) > W + 1e-7) {
                            throw std::runtime_error("external command outside input limits");
                        }
                        r.model->setControlInput(u);
                        r.opt = {
                            {"status", "success"},
                            {"result", r.model->control2Json(u)},
                            {"solver_info", {{"status", "external-accepted"}}},
                            {"cbfNoSlack", json::array()},
                            {"cbfSlack", softRows(r)},
                            {"comparator", diag}
                        };
                    }
                    for (auto& r : swarm.robots) {
                        for (auto& other : swarm.robots) {
                            other->comm->receiveVelocity2D(r->id, r->model->getVelocity());
                            other->sameFrameCommandVersions[r->id] = frame;
                        }
                    }
                    swarm.route1ChainLatencySeconds =
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                    return true;
                };
            }
        }

        swarm.run();
        swarm.updateGridWorld();

        json term = {
            {"time_s", swarm.robots.front()->runtime},
            {"true_coverage", swarm.gridWorldGroundTruth.getPercentage()},
            {"estimated_coverage_last_measured", swarm.robots.front()->gridWorld.getPercentage()},
            {"true_valid_count", swarm.gridWorldGroundTruth.validCount},
            {"true_visited_indices", json::array()},
            {"robots", json::array()}
        };
        for (size_t j = 0; j < swarm.gridWorldGroundTruth.vis.size(); ++j) {
            if (swarm.gridWorldGroundTruth.valid[j] && swarm.gridWorldGroundTruth.vis[j]) {
                term["true_visited_indices"].push_back(j);
            }
        }
        for (const auto& r : swarm.robots) {
            const auto p = r->model->xy();
            const auto v = r->model->getWork2PhysicalVelocity();
            term["robots"].push_back({
                {"id", r->id},
                {"truth", {p.x, p.y}},
                {"physical_velocity", {v[0], v[1]}},
                {"yaw", r->model->getStateVariable("yawRad")}
            });
        }
        std::ofstream output(cfg.at("work2").at("terminal_path").get<std::string>());
        output << term.dump() << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[CBF2026_NATIVE_ERROR] " << e.what() << '\n';
        return 1;
    }
}
