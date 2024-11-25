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
    GRBEnv env;
    std::vector<GRBVar> vars;
    std::vector<GRBVar> slackVars;
    GRBQuadExpr obj = 0.0;
public:

    Robot() = default;

    Robot(int id, const std::string &modelType) : id(id), env(true) {
        if (modelType == "SingleIntegrate2D") {
            model = std::make_unique<SingleIntegrate2D>();
        } else if (modelType == "DoubleIntegrate2D") {
            model = std::make_unique<DoubleIntegrate2D>();
        } else {
            throw std::invalid_argument("Invalid model type");
        }
        env.set("OutputFlag", "0");
        env.start();
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
            try {
                GRBModel *grbModel = new GRBModel(env);
                vars.clear();
                slackVars.clear();

                auto f = model->f();
                auto g = model->g();
                auto x = model->getX();

                int uSize = model->uSize();
                int slackSize = cbfSlack.size();
                int totalSize = uSize + slackSize;

                std::vector<double> lowerBounds(totalSize, -GRB_INFINITY);
                std::vector<double> upperBounds(totalSize, GRB_INFINITY);
                std::vector<double> objCoeffs(totalSize, 0.0);
                std::vector<char> varTypes(totalSize, GRB_CONTINUOUS);

                GRBVar *allVars = grbModel->addVars(
                        lowerBounds.data(),
                        upperBounds.data(),
                        objCoeffs.data(),
                        varTypes.data(),
                        nullptr,
                        totalSize
                );

                vars.assign(allVars, allVars + uSize);
                slackVars.assign(allVars + uSize, allVars + totalSize);

                obj = 0.0;
                for (int i = 0; i < model->uSize(); i++) {
                    obj += (vars[i] - uNominal[i]) * (vars[i] - uNominal[i]);
                }
                for (int i = 0; i < cbfSlack.size(); i++) {
                    obj += 0.1 * slackVars[i] * slackVars[i];
                }
                grbModel->setObjective(obj, GRB_MINIMIZE);

                if (!cbfNoSlack.cbfs.empty()) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = cbfNoSlack.constraintUCoe(f, g, x, runtime);
                    for (int j = 0; j < model->uSize(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    double constraintConstWithTime = cbfNoSlack.constraintConstWithTime(f, g, x, runtime);
                    jsonCBFNoSlack.emplace_back(json{
                                                     {"name",  cbfNoSlack.getName()},
                                                     {"coe",   model->control2Json(uCoe)},
                                                     {"const", constraintConstWithTime}
                                             });
                    grbModel->addConstr(ln, '>', -constraintConstWithTime);
                }
                opt["cbfNoSlack"] = jsonCBFNoSlack;

                int cnt = 0;
                for (auto &[name, cbf]: cbfSlack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = cbf.constraintUCoe(f, g, x, runtime);
                    for (int j = 0; j < model->uSize(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    ln += slackVars[cnt];
                    double constraintConst = cbf.constraintConstWithoutTime(f, g, x, runtime);

                    jsonCBFSlack.emplace_back(json{
                                                   {"name",  cbf.name},
                                                   {"coe",   model->control2Json(uCoe)},
                                                   {"const", constraintConst}
                                           });
                    grbModel->addConstr(ln, '>', -constraintConst);
                    ++cnt;
                }
                opt["cbfSlack"] = jsonCBFSlack;

                grbModel->optimize();

                Eigen::VectorXd u = Eigen::VectorXd::NullaryExpr(model->uSize(), [&](int i) {
                    return vars[i].get(GRB_DoubleAttr_X);
                });
                model->setControlInput(u);
                opt["result"] = model->control2Json(u);
                Eigen::VectorXd  slacks = Eigen::VectorXd::NullaryExpr(slackVars.size(), [&](int i) {
                    return slackVars[i].get(GRB_DoubleAttr_X);
                });
            } catch (GRBException& e) {
                std::cerr << "Gurobi Exception: Code = " << e.getErrorCode()
                          << ", Message = " << e.getMessage() << std::endl;
                throw;
            } catch (std::exception& e) {
                std::cerr << "Standard Exception: " << e.what() << std::endl;
                throw;
            } catch (...) {
                std::cerr << "Unknown Exception occurred during optimization." << std::endl;
                throw;
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
