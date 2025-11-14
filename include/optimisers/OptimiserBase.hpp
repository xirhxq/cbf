#ifndef CBF_OPTIMISER_BASE_HPP
#define CBF_OPTIMISER_BASE_HPP

#include "utils.h"

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

#endif // CBF_OPTIMISER_BASE_HPP