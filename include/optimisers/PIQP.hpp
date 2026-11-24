#ifndef CBF_OPTIMISER_PIQP_HPP
#define CBF_OPTIMISER_PIQP_HPP

#include "ConstraintRowScaling.hpp"
#include "OptimiserBase.hpp"

#include <piqp/piqp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

class PIQP : public OptimiserBase {
    int n_ = 0;
    int controls_ = 0;
    bool quadratic_slack_ = false;
    double eps_abs_ = 1.0e-9;
    double eps_rel_ = 1.0e-10;
    double eps_duality_gap_abs_ = 1.0e-11;
    double eps_duality_gap_rel_ = 2.0e-13;
    int max_iter_ = 250;
    Eigen::VectorXd nominal_;
    std::vector<Eigen::VectorXd> rows_;
    std::vector<double> rhs_;
    json status_ = {{"status", "not_initialized"}, {"solver", "PIQP"}};
    double objective_ = 0.0;

    struct AttemptResult {
        piqp::Status status = piqp::Status::PIQP_UNSOLVED;
        Eigen::VectorXd x;
        int iter = -1;
        double primal_res = std::numeric_limits<double>::infinity();
        double dual_res = std::numeric_limits<double>::infinity();
        double setup_ms = 0.0;
        double solve_ms = 0.0;
        double objective = 0.0;
    };

    AttemptResult runSolveAttempt(
        const Eigen::MatrixXd& P,
        const Eigen::VectorXd& c,
        const Eigen::MatrixXd& eq,
        const Eigen::VectorXd& b,
        const Eigen::VectorXd& xl,
        const Eigen::VectorXd& xu,
        bool use_row_scaling,
        bool enable_refinement,
        bool scale_cost,
        int iter_limit) const {
        const auto started = std::chrono::steady_clock::now();
        Eigen::MatrixXd G(rows_.size(), n_);
        Eigen::VectorXd lower(rows_.size());
        for (std::size_t i = 0; i < rows_.size(); ++i) {
            const double scale = use_row_scaling
                ? exactEquivalentConstraintRowScale(rows_[i], rhs_[i])
                : 1.0;
            G.row(static_cast<Eigen::Index>(i)) = scale * rows_[i].transpose();
            lower[static_cast<Eigen::Index>(i)] = scale * rhs_[i];
        }

        piqp::DenseSolver<double> solver;
        solver.settings().verbose = false;
        solver.settings().compute_timings = true;
        solver.settings().eps_abs = eps_abs_;
        solver.settings().eps_rel = eps_rel_;
        solver.settings().eps_duality_gap_abs = eps_duality_gap_abs_;
        solver.settings().eps_duality_gap_rel = eps_duality_gap_rel_;
        solver.settings().max_iter = iter_limit;
        solver.settings().iterative_refinement_always_enabled = enable_refinement;
        solver.settings().preconditioner_scale_cost = scale_cost;

        solver.setup(P, c, eq, b, G, lower, piqp::nullopt, xl, xu);
        const auto setup_done = std::chrono::steady_clock::now();
        const piqp::Status st = solver.solve();
        const auto finished = std::chrono::steady_clock::now();

        AttemptResult out;
        out.status = st;
        out.x = solver.result().x;
        out.iter = static_cast<int>(solver.result().info.iter);
        out.primal_res = solver.result().info.primal_res;
        out.dual_res = solver.result().info.dual_res;
        out.setup_ms = std::chrono::duration<double, std::milli>(setup_done - started).count();
        out.solve_ms = std::chrono::duration<double, std::milli>(finished - setup_done).count();
        if (st == piqp::Status::PIQP_SOLVED && out.x.size() == n_ && out.x.allFinite()) {
            out.objective = 0.5 * out.x.dot(P * out.x) + c.dot(out.x) + nominal_.squaredNorm();
        }
        return out;
    }

public:
    explicit PIQP(json& settings) : OptimiserBase(settings) {
        const std::string penalty = settings.value("slack-penalty", std::string("linear"));
        if (penalty != "linear" && penalty != "quadratic") {
            throw std::invalid_argument("unknown slack penalty: " + penalty);
        }
        quadratic_slack_ = (penalty == "quadratic");
        if (settings.contains("absolute-tolerance") &&
            settings["absolute-tolerance"].is_number()) {
            const double requested = settings["absolute-tolerance"].get<double>();
            if (std::isfinite(requested) && requested > 0.0) {
                eps_abs_ = std::min(requested, 1.0e-9);
            }
        }
        if (settings.contains("relative-tolerance") &&
            settings["relative-tolerance"].is_number()) {
            const double requested = settings["relative-tolerance"].get<double>();
            if (std::isfinite(requested) && requested >= 0.0) {
                eps_rel_ = std::min(requested, 1.0e-10);
            }
        }
        if (settings.contains("maximum-iterations") &&
            settings["maximum-iterations"].is_number_integer()) {
            const int requested = settings["maximum-iterations"].get<int>();
            if (requested > 0) {
                max_iter_ = std::min(requested, 500);
            }
        }
    }

    void clear() override {
        n_ = 0;
        controls_ = 0;
        rows_.clear();
        rhs_.clear();
        nominal_.resize(0);
        status_ = {{"status", "not_initialized"}, {"solver", "PIQP"}};
        objective_ = 0.0;
    }

    void start(int total, int controls) override {
        if (controls < 0 || total < controls) {
            throw std::invalid_argument("invalid PIQP variable layout");
        }
        n_ = total;
        controls_ = controls;
        rows_.clear();
        rhs_.clear();
        nominal_.resize(0);
        status_ = {{"status", "not_initialized"}, {"solver", "PIQP"}};
        objective_ = 0.0;
    }

    void setObjective(const Eigen::VectorXd& nominal) override {
        if (nominal.size() != controls_) {
            throw std::invalid_argument("invalid nominal size");
        }
        nominal_ = nominal;
    }

    void addLinearConstraint(const Eigen::VectorXd& coe, double rhs) override {
        if (coe.size() > n_) {
            throw std::invalid_argument("PIQP row too large");
        }
        Eigen::VectorXd row = Eigen::VectorXd::Zero(n_);
        row.head(coe.size()) = coe;
        rows_.push_back(std::move(row));
        rhs_.push_back(rhs);
    }

    Eigen::VectorXd solve() override {
        Eigen::VectorXd failure = Eigen::VectorXd::Zero(n_);
        objective_ = 0.0;
        try {
            if (n_ <= 0 || nominal_.size() != controls_) {
                throw std::runtime_error("PIQP objective not initialized");
            }

            const double inf = std::numeric_limits<double>::infinity();
            Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_, n_);
            Eigen::MatrixXd eq(0, n_);
            Eigen::VectorXd c = Eigen::VectorXd::Zero(n_);
            Eigen::VectorXd b(0);
            Eigen::VectorXd xl(n_);
            Eigen::VectorXd xu = Eigen::VectorXd::Constant(n_, inf);

            for (int i = 0; i < n_; ++i) {
                xl[i] = i < controls_ ? -inf : 0.0;
                if (i < controls_) {
                    P(i, i) = 2.0;
                    c[i] = -2.0 * nominal_[i];
                } else {
                    if (quadratic_slack_) {
                        P(i, i) = 2.0 * k_delta;
                    } else {
                        c[i] = k_delta;
                    }
                }
            }

            // Primary solve passes raw rows directly to PIQP's built-in Ruiz equilibration.
            AttemptResult attempt = runSolveAttempt(
                P, c, eq, b, xl, xu, false, false, false, std::min(max_iter_, 250));
            bool used_retry = false;
            if (attempt.status != piqp::Status::PIQP_SOLVED) {
                used_retry = true;
                attempt = runSolveAttempt(
                    P, c, eq, b, xl, xu, true, true, true, 500);
            }

            const bool solved = attempt.status == piqp::Status::PIQP_SOLVED &&
                                attempt.x.size() == n_ && attempt.x.allFinite();
            status_ = {
                {"status", solved ? "optimal" : "failed"},
                {"solver", "PIQP"},
                {"piqp_status", static_cast<int>(attempt.status)},
                {"piqp_status_text", piqp::status_to_string(attempt.status)},
                {"iteration_count", attempt.iter},
                {"primal_residual", attempt.primal_res},
                {"dual_residual", attempt.dual_res},
                {"vars_count", n_},
                {"constraints_count", static_cast<int>(rows_.size())},
                {"setup_time_ms", attempt.setup_ms},
                {"solve_time_ms", attempt.solve_ms},
                {"refined_retry", used_retry},
            };

            if (!solved) {
                if (attempt.status == piqp::Status::PIQP_SOLVED) {
                    status_["error"] = "nonfinite solution";
                }
                return failure;
            }

            objective_ = attempt.objective;
            status_["objective_value"] = objective_;

            if (!rows_.empty()) {
                double min_residual = std::numeric_limits<double>::infinity();
                for (std::size_t i = 0; i < rows_.size(); ++i) {
                    min_residual = std::min(min_residual, rows_[i].dot(attempt.x) - rhs_[i]);
                }
                status_["min_linear_residual"] = min_residual;
            }
            return attempt.x;
        } catch (const std::exception& e) {
            status_ = {{"status", "failed"}, {"solver", "PIQP"}, {"error", e.what()}};
            return failure;
        }
    }

    double getObjectiveValue() const override { return objective_; }

    json getStatus() const override { return status_; }

    void write(std::string filename) override {
        json data = {
            {"n", n_},
            {"controls", controls_},
            {"rows", json::array()},
            {"rhs", rhs_},
            {"status", status_},
        };
        for (const auto& row : rows_) {
            data["rows"].push_back(std::vector<double>(row.data(), row.data() + row.size()));
        }
        std::ofstream out(filename);
        out << data.dump(2) << '\n';
    }
};

#endif  // CBF_OPTIMISER_PIQP_HPP
