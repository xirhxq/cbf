#ifndef CBF_OPTIMISER_PIQP_HPP
#define CBF_OPTIMISER_PIQP_HPP

#include "OptimiserBase.hpp"
#include <piqp/piqp.hpp>
#include <chrono>
#include <limits>
#include <vector>

// Adapter for the linear-constraint QPs used by the sampled-hold main method.
class PIQP : public OptimiserBase {
    int n_ = 0, controls_ = 0, auxiliary_ = 0;
    bool quadraticSlack_ = false;
    Eigen::VectorXd nominal_;
    std::vector<Eigen::VectorXd> rows_;
    std::vector<double> rhs_;
    json status_ = {{"status", "not_initialized"}};
    double objective_ = 0.;

public:
    explicit PIQP(json& settings) : OptimiserBase(settings) {
        const auto penalty = settings.value("slack-penalty", "linear");
        if (penalty != "linear" && penalty != "quadratic")
            throw std::invalid_argument("unknown slack penalty");
        quadraticSlack_ = penalty == "quadratic";
    }
    void clear() override {
        n_ = controls_ = auxiliary_ = 0;
        rows_.clear(); rhs_.clear(); nominal_.resize(0);
        status_ = {{"status", "not_initialized"}}; objective_ = 0.;
    }
    void setAuxiliaryVariableCount(int count) override {
        if (count < 0) throw std::invalid_argument("negative auxiliary count");
        auxiliary_ = count;
    }
    void start(int total, int controls) override {
        if (controls < 0 || total < controls || auxiliary_ > total-controls)
            throw std::invalid_argument("invalid PIQP variable layout");
        n_ = total; controls_ = controls;
        rows_.clear(); rhs_.clear(); nominal_.resize(0);
        status_ = {{"status", "not_initialized"}}; objective_ = 0.;
    }
    void setObjective(Eigen::VectorXd& nominal) override {
        if (nominal.size() != controls_) throw std::invalid_argument("invalid nominal size");
        nominal_ = nominal;
    }
    void addLinearConstraint(Eigen::VectorXd coe, double rhs) override {
        if (coe.size() > n_) throw std::invalid_argument("PIQP row too large");
        Eigen::VectorXd row = Eigen::VectorXd::Zero(n_);
        row.head(coe.size()) = coe;
        rows_.push_back(std::move(row)); rhs_.push_back(rhs);
    }
    Eigen::VectorXd solve() override {
        Eigen::VectorXd failure = Eigen::VectorXd::Zero(n_);
        try {
            if (nominal_.size() != controls_) throw std::runtime_error("PIQP objective not initialized");
            const auto started = std::chrono::steady_clock::now();
            Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_, n_), eq(0,n_), G(rows_.size(),n_);
            Eigen::VectorXd c = Eigen::VectorXd::Zero(n_), b(0), lower(rows_.size()), xl(n_), xu(n_);
            const double inf = std::numeric_limits<double>::infinity();
            for (int i=0; i<n_; ++i) {
                xl[i] = i<controls_ ? -inf : 0.; xu[i] = inf;
                if (i<controls_) { P(i,i)=2.; c[i]=-2.*nominal_[i]; }
                else if (i<n_-auxiliary_) {
                    if (quadraticSlack_) P(i,i)=2.*k_delta; else c[i]=k_delta;
                }
            }
            for (size_t i=0; i<rows_.size(); ++i) { G.row(i)=rows_[i].transpose(); lower[i]=rhs_[i]; }
            piqp::DenseSolver<double> solver;
            solver.settings().compute_timings = true;
            solver.setup(P,c,eq,b,G,lower,piqp::nullopt,xl,xu);
            const auto setupDone = std::chrono::steady_clock::now();
            const auto result = solver.solve();
            const auto finished = std::chrono::steady_clock::now();
            status_ = {{"status", result==piqp::Status::PIQP_SOLVED ? "optimal" : "failed"},
                       {"piqp_status", int(result)}, {"piqp_status_text", piqp::status_to_string(result)},
                       {"vars_count",n_},{"constraints_count",rows_.size()},
                       {"solve_time_ms",std::chrono::duration<double,std::milli>(finished-setupDone).count()},
                       {"jetson_setup_wall_s", std::chrono::duration<double>(setupDone-started).count()},
                       {"jetson_solve_wall_s", std::chrono::duration<double>(finished-setupDone).count()}};
            if (result != piqp::Status::PIQP_SOLVED) return failure;
            const Eigen::VectorXd x=solver.result().x;
            if (!x.allFinite()) { status_["status"]="failed"; status_["error"]="nonfinite solution"; return failure; }
            objective_=0.5*x.dot(P*x)+c.dot(x)+nominal_.squaredNorm();
            status_["objective_value"]=objective_;
            if (!rows_.empty()) status_["min_linear_residual"]=(G*x-lower).minCoeff();
            status_["primal_residual"]=solver.result().info.primal_res;
            status_["dual_residual"]=solver.result().info.dual_res;
            return x;
        } catch (const std::exception& e) {
            status_={{"status","failed"},{"error",e.what()}};
            return failure;
        }
    }
    double getObjectiveValue() const override { return objective_; }
    json getStatus() const override { return status_; }
    json sampledHoldPrimalDiagnostics(const Eigen::VectorXd& x) const {
        if(x.size()!=n_||!x.allFinite())throw std::invalid_argument("invalid diagnostic PIQP primal");
        piqp::Settings<double> defaults;
        json d={{"linear_rows",json::array()},{"lower_bound_residuals",json::array()},
                {"linear_residual_definition","nonnegative is feasible; original solver row order"},
                {"piqp_eps_abs",defaults.eps_abs},{"piqp_eps_rel",defaults.eps_rel},
                {"piqp_eps_duality_gap_abs",defaults.eps_duality_gap_abs},
                {"piqp_eps_duality_gap_rel",defaults.eps_duality_gap_rel},{"piqp_max_iter",defaults.max_iter}};
        for(size_t i=0;i<rows_.size();++i)d["linear_rows"].push_back(rows_[i].dot(x)-rhs_[i]);
        for(int i=0;i<n_;++i)d["lower_bound_residuals"].push_back(i<controls_?json(nullptr):json(x[i]));
        return d;
    }
    void write(std::string filename) override {
        json data={{"n",n_},{"controls",controls_},{"auxiliary",auxiliary_},
                   {"rows",json::array()},{"rhs",rhs_},{"status",status_}};
        for (const auto& row: rows_) data["rows"].push_back(std::vector<double>(row.data(),row.data()+row.size()));
        std::ofstream out(filename); out << data.dump(2) << '\n';
    }
};
#endif
