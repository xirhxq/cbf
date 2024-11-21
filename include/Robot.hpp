#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

#include "utils.h"
#include "CBF.hpp"
#include "MultiCBF.hpp"
#include "World.hpp"
#include "models/models"

class Robot {
public:
    int id = 0;
    MultiCBF cbfNoSlack;
    std::unordered_map<std::string, CBF> cbfSlack;
    std::unique_ptr<BaseModel> model;
    json opt;
public:

    Robot() = default;

    Robot(int id, const std::string &modelType): id(id) {
        if (modelType == "SingleIntegrate2D") {
            model = std::make_unique<SingleIntegrate2D>();
        } else if (modelType == "DoubleIntegrate2D") {
            model = std::make_unique<DoubleIntegrate2D>();
        } else {
            throw std::invalid_argument("Invalid model type");
        }
    }

    void optimise(VectorXd &uNominal, double runtime, double dt, World world) {
        opt = {
                {"nominal", model->toJson(uNominal)},
                {"result", model->toJson(uNominal)},
                {"cbfNoSlack", json::array()},
                {"cbfSlack", json::array()}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();
        if (world.isCharging(model->xy()) && model->getStateVariable("battery") < 100.0) {
            model->startCharge();
            uNominal.setZero();
            model->setControlInput(uNominal);
        } else {
            try {
                GRBEnv env = GRBEnv(true);
                env.set("OutputFlag", "0");
                env.start();

                GRBModel grbModel = GRBModel(env);

                std::vector<GRBVar> vars;
                char s[10];
                for (int i = 0; i < model->uSize(); i++) {
                    snprintf(s, 10, "var-%d", i);
                    vars.push_back(grbModel.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, s));
                }

                std::vector<GRBVar> slackVars;
                for (int i = 0; i < cbfSlack.size(); i++) {
                    snprintf(s, 10, "slack-%d", i);
                    slackVars.push_back(grbModel.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, s));
                }

                GRBQuadExpr obj = 0.0;
                for (int i = 0; i < model->uSize(); i++) {
                    obj += (vars[i] - uNominal[i]) * (vars[i] - uNominal[i]);
                }
                for (int i = 0; i < cbfSlack.size(); i++) {
                    obj += 0.1 * slackVars[i] * slackVars[i];
                }
                grbModel.setObjective(obj, GRB_MINIMIZE);

                if (!cbfNoSlack.cbfs.empty()) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = cbfNoSlack.constraintUCoe(model->f(), model->g(), model->getX(), runtime);
                    for (int j = 0; j < model->uSize(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    double constraintConstWithTime = cbfNoSlack.constraintConstWithTime(model->f(), model->g(), model->getX(), runtime);
                    jsonCBFNoSlack.push_back({
                                                     {"name",  cbfNoSlack.getName()},
                                                     {"coe",   model->toJson(uCoe)},
                                                     {"const", constraintConstWithTime}
                                             });
                    grbModel.addConstr(ln, '>', -constraintConstWithTime);
                }
                opt["cbfNoSlack"] = jsonCBFNoSlack;

                int cnt = 0;
                for (auto &[name, cbf]: cbfSlack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = cbf.constraintUCoe(model->f(), model->g(), model->getX(), runtime);
                    for (int j = 0; j < model->uSize(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    ln += slackVars[cnt];
                    double constraintConst = cbf.constraintConstWithoutTime(
                            model->f(), model->g(), model->getX(), runtime);

                    jsonCBFSlack.push_back({
                                                   {"name",  cbf.name},
                                                   {"coe",   model->toJson(uCoe)},
                                                   {"const", constraintConst}
                                           });
                    grbModel.addConstr(ln, '>', -constraintConst);
                    ++cnt;
                }
                opt["cbfSlack"] = jsonCBFSlack;

//                model.set(GRB_IntParam_OutputFlag, 0);
                grbModel.optimize();

                Eigen::VectorXd  u(model->uSize());
                for (int i = 0; i < model->uSize(); i++) u(i) = vars[i].get(GRB_DoubleAttr_X);
                model->setControlInput(u);
                opt["result"] = model->toJson(u);
                VectorXd slacks(slackVars.size());
                for (int i = 0; i < slackVars.size(); i++) slacks(i) = slackVars[i].get(GRB_DoubleAttr_X);
            } catch (GRBException e) {
                std::cout << "Error code = " << e.getErrorCode() << std::endl;
                printf(".......\n");
                std::cout << e.getMessage() << std::endl;
                std::cout << "id" << id << std::endl;
                assert(0);
            } catch (...) {
                std::cout << "Exception during optimization" << std::endl;
                printf("...................\n");
                assert(0);
            }
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
