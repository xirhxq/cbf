#ifndef CENTRALIZEDMODEL_HPP
#define CENTRALIZEDMODEL_HPP

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "BaseModel.hpp"
#include "../utils.h"

class CentralizedModel : public BaseModel {
public:
    std::vector<BaseModel*> individualModels;
    std::vector<int> stateSizes;
    std::vector<int> stateOffsets;
    std::vector<int> controlSizes;
    std::vector<int> controlOffsets;
    int totalStateSize;
    int totalControlSize;

    CentralizedModel() : totalStateSize(0), totalControlSize(0) {}

    void addRobot(BaseModel* model) {
        individualModels.push_back(model);
        stateSizes.push_back(model->getStateSize());
        controlSizes.push_back(model->uSize());

        stateOffsets.push_back(totalStateSize);
        controlOffsets.push_back(totalControlSize);

        totalStateSize += model->getStateSize();
        totalControlSize += model->uSize();

        updateConcatenatedStates();
        updateConcatenatedControls();
    }

    void updateConcatenatedStates() {
        X.resize(totalStateSize);
        for (size_t i = 0; i < individualModels.size(); ++i) {
            X.block(stateOffsets[i], 0, stateSizes[i], 1) = individualModels[i]->getX();
        }
    }

    void updateConcatenatedControls() {
        u.resize(totalControlSize);
        for (size_t i = 0; i < individualModels.size(); ++i) {
            u.block(controlOffsets[i], 0, controlSizes[i], 1) = individualModels[i]->getControlInput();
        }
    }

    Eigen::MatrixXd f() const {
        Eigen::VectorXd concatenatedF = Eigen::VectorXd::Zero(totalStateSize);
        for (size_t i = 0; i < individualModels.size(); ++i) {
            Eigen::VectorXd localF = individualModels[i]->f();
            concatenatedF.block(stateOffsets[i], 0, stateSizes[i], 1) = localF;
        }
        return concatenatedF;
    }

    Eigen::MatrixXd g() const {
        Eigen::MatrixXd concatenatedG = Eigen::MatrixXd::Zero(totalStateSize, totalControlSize);
        for (size_t i = 0; i < individualModels.size(); ++i) {
            Eigen::MatrixXd localG = individualModels[i]->g();
            concatenatedG.block(stateOffsets[i], controlOffsets[i], stateSizes[i], controlSizes[i]) = localG;
        }
        return concatenatedG;
    }

    void setRobotControl(int robotIndex, const Eigen::VectorXd& globalSolution) {
        if (robotIndex >= 0 && robotIndex < static_cast<int>(individualModels.size())) {
            Eigen::VectorXd robotControl = globalSolution.block(
                controlOffsets[robotIndex], 0, controlSizes[robotIndex], 1);
            individualModels[robotIndex]->setControlInput(robotControl);
            updateConcatenatedControls();
        }
    }

    void setRobotStates(int robotIndex, const Eigen::VectorXd& robotState) {
        if (robotIndex >= 0 && robotIndex < static_cast<int>(individualModels.size())) {
            individualModels[robotIndex]->setStateVector(robotState);
            updateConcatenatedStates();
        }
    }

    Eigen::VectorXd getRobotState(int robotIndex) const {
        if (robotIndex >= 0 && robotIndex < static_cast<int>(individualModels.size())) {
            return individualModels[robotIndex]->getX();
        }
        return Eigen::VectorXd();
    }

    Eigen::VectorXd getRobotControl(int robotIndex) const {
        if (robotIndex >= 0 && robotIndex < static_cast<int>(individualModels.size())) {
            return individualModels[robotIndex]->getControlInput();
        }
        return Eigen::VectorXd();
    }

    int getNumRobots() const {
        return static_cast<int>(individualModels.size());
    }

    int getTotalControlSize() const {
        return totalControlSize;
    }

    int getTotalStateSize() const {
        return totalStateSize;
    }

    int getControlOffset(int robotIndex) const {
        if (robotIndex >= 0 && robotIndex < static_cast<int>(individualModels.size())) {
            return controlOffsets[robotIndex];
        }
        return -1;
    }

    int getStateOffset(int robotIndex) const {
        if (robotIndex >= 0 && robotIndex < static_cast<int>(individualModels.size())) {
            return stateOffsets[robotIndex];
        }
        return -1;
    }

    void output() const override {
        printf("CentralizedModel with %d robots:\n", (int)individualModels.size());
        for (size_t i = 0; i < individualModels.size(); ++i) {
            printf("Robot %d: state_offset=%d, control_offset=%d\n",
                   (int)i, stateOffsets[i], controlOffsets[i]);
        }
    }

    void clear() {
        individualModels.clear();
        stateSizes.clear();
        stateOffsets.clear();
        controlSizes.clear();
        controlOffsets.clear();
        totalStateSize = 0;
        totalControlSize = 0;
        X.resize(0);
        u.resize(0);
    }
};

#endif // CENTRALIZEDMODEL_HPP