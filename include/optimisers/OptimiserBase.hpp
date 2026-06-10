#ifndef CBF_OPTIMISER_BASE_HPP
#define CBF_OPTIMISER_BASE_HPP

#include "utils.h"
#include <memory>
#include <stdexcept>
#include <vector>
#include <string>

class OptimiserBase {
protected:
    double k_delta = 1.0;
public:
    OptimiserBase(json &settings) {
        if (settings.contains("k_delta")) {
            k_delta = settings["k_delta"];
        }
    }

    virtual void clear() = 0;
    virtual void start(int total_size, int u_size) = 0;

    virtual void setObjective(const Eigen::VectorXd &uNominal) = 0;
    virtual void addLinearConstraint(const Eigen::VectorXd &coe, double rhs) = 0;

    virtual Eigen::VectorXd solve() = 0;

    virtual void write(std::string filename) = 0;

    virtual double getObjectiveValue() const = 0;

    virtual json getStatus() const = 0;

    virtual ~OptimiserBase() = default;
};

inline Eigen::VectorXd makeSlackConstraintCoefficients(const Eigen::VectorXd &uCoe,
                                                       int slackSize,
                                                       int slackIndex) {
    if (slackSize < 0) {
        throw std::invalid_argument("slackSize must be non-negative");
    }
    if (slackIndex < 0 || slackIndex >= slackSize) {
        throw std::out_of_range("slackIndex outside slack coefficient block");
    }

    Eigen::VectorXd coe = Eigen::VectorXd::Zero(uCoe.size() + slackSize);
    coe.head(uCoe.size()) = uCoe;
    coe(uCoe.size() + slackIndex) = 1.0;
    return coe;
}

inline std::vector<std::string> getAvailableOptimisers() {
    std::vector<std::string> available;
#ifdef ENABLE_GUROBI
    available.push_back("Gurobi");
#endif
#ifdef ENABLE_HIGHS
    available.push_back("HiGHS");
#endif
#ifdef ENABLE_OSQP
    available.push_back("OSQP");
#endif
#ifdef ENABLE_PROXQP
    available.push_back("ProxQP");
#endif
    return available;
}

inline std::string joinAvailableOptimisers() {
    auto available = getAvailableOptimisers();
    std::string result;
    for (size_t i = 0; i < available.size(); i++) {
        if (i > 0) result += ", ";
        result += available[i];
    }
    return result;
}

std::unique_ptr<OptimiserBase> createOptimiser(const std::string& name, json& settings);

#endif // CBF_OPTIMISER_BASE_HPP
