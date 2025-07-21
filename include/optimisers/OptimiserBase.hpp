#ifndef CBF_OPTIMISER_BASE_HPP
#define CBF_OPTIMISER_BASE_HPP

#include "utils.h"

class OptimiserBase {
public:
    virtual void clear() = 0;
    virtual void start(int size) = 0;

    virtual void setObjective(Eigen::VectorXd &uNominal) = 0;
    virtual void addLinearConstraint(Eigen::VectorXd coe, double rhs) = 0;

    virtual Eigen::VectorXd solve() = 0;

    virtual void write(std::string filename) = 0;

    virtual double getObjectiveValue() const = 0;

    virtual ~OptimiserBase() = default;
};

#endif // CBF_OPTIMISER_BASE_HPP