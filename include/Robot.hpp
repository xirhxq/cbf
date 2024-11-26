#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

#include "utils.h"
#include "CBF.hpp"
#include "MultiCBF.hpp"
#include "World.hpp"
#include "models/models"
#include "optimisers/optimisers"

class Robot {
public:
    int id = 0;
    MultiCBF cbfNoSlack;
    std::unordered_map<std::string, CBF> cbfSlack;
    std::unique_ptr<BaseModel> model;
    json opt;
    std::unique_ptr<OptimiserBase> optimiser;
public:

    Robot() = default;

    Robot(int id, const json &settings) : id(id) {
        if (settings["model"] == "SingleIntegrate2D") {
            model = std::make_unique<SingleIntegrate2D>();
        } else if (settings["model"] == "DoubleIntegrate2D") {
            model = std::make_unique<DoubleIntegrate2D>();
        } else {
            throw std::invalid_argument("Invalid model type");
        }
        if (settings["optimiser"] == "Gurobi") {
            optimiser = std::make_unique<Gurobi>();
        }
        else if (settings["optimiser"] == "HiGHS") {
            optimiser = std::make_unique<HiGHS>();
        } else {
            throw std::invalid_argument("Invalid optimiser type");
        }
        model->setStateVariable("battery", 20.0 * (rand() % 100) / 100 + 10);
        model->setYawDeg(180);
        cbfSlack.clear();
        cbfNoSlack.cbfs.clear();
    }

    void optimise(VectorXd &uNominal, double runtime, double dt, World world) {
        opt = {
                {"nominal",    model->control2Json(uNominal)},
                {"result",     model->control2Json(uNominal)},
                {"cbfNoSlack", json::array()},
                {"cbfSlack",   json::array()}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();
        if (world.isCharging(model->xy()) && model->getStateVariable("battery") < 100.0) {
            model->startCharge();
            uNominal.setZero();
            model->setControlInput(uNominal);
        } else {
            optimiser->clear();

            auto f = model->f();
            auto g = model->g();
            auto x = model->getX();

            int uSize = model->uSize();
            int slackSize = cbfSlack.size();
            int totalSize = uSize + slackSize;

            optimiser->start(totalSize);

            optimiser->setObjective(uNominal);

            if (!cbfNoSlack.cbfs.empty()) {
                VectorXd uCoe = cbfNoSlack.constraintUCoe(f, g, x, runtime);
                double constraintConstWithTime = cbfNoSlack.constraintConstWithTime(f, g, x, runtime);

                optimiser->addLinearConstraint(uCoe, -constraintConstWithTime);

                jsonCBFNoSlack.emplace_back(json{
                                                 {"name",  cbfNoSlack.getName()},
                                                 {"coe",   model->control2Json(uCoe)},
                                                 {"const", constraintConstWithTime}
                                         });
            }
            opt["cbfNoSlack"] = jsonCBFNoSlack;

            int cnt = 0;
            for (auto &[name, cbf]: cbfSlack) {
                VectorXd uCoe = cbf.constraintUCoe(f, g, x, runtime);
                Eigen::VectorXd sCoe = Eigen::VectorXd::Zero(slackSize);
                sCoe(cnt) = 1.0;
                Eigen::VectorXd coe(totalSize);
                coe << uCoe, sCoe;
                double constraintConst = cbf.constraintConstWithoutTime(f, g, x, runtime);

                optimiser->addLinearConstraint(coe, -constraintConst);

                jsonCBFSlack.emplace_back(json{
                                               {"name",  cbf.name},
                                               {"coe",   model->control2Json(uCoe)},
                                               {"const", constraintConst}
                                       });
                ++cnt;
            }
            opt["cbfSlack"] = jsonCBFSlack;

            auto result = optimiser->solve();

            auto u = result.head(uSize);
            model->setControlInput(u);
            opt["result"] = model->control2Json(u);
            opt["slacks"] = result.tail(slackSize);
        }
    }

    void stepTimeForward(double dt) const {
        model->stepTimeForward(dt);
    }

    void output() const {
        std::cout << "Robot " << id << ": ";
        model->output();
    }

};


#endif //CBF_ROBOT_HPP
