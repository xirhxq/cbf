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

    virtual void setObjective(Eigen::VectorXd &uNominal) = 0;
    virtual void addLinearConstraint(Eigen::VectorXd coe, double rhs) = 0;

    virtual Eigen::VectorXd solve() = 0;

    virtual void write(std::string filename) = 0;

    virtual double getObjectiveValue() const = 0;

    virtual json getStatus() const = 0;

    virtual ~OptimiserBase() = default;
};

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