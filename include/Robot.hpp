#ifndef CBF_MAIN_ROBOT_HPP
#define CBF_MAIN_ROBOT_HPP

//#define OPT_DEBUG

#include "utils.h"
#include "CBF.hpp"
#include "World.hpp"

class Robot {
public:
    int id = 0;
    int xIndex = 0, yIndex = 1, batteryIndex = 2, cameraIndex = 3;
    std::map<std::string, CBF> cbfNoSlack, cbfSlack;
    MatrixXd G;
    VectorXd F, X;

public:

    Robot() = default;

    Robot(int dimension) {
        G = MatrixXd::Identity(dimension, dimension);
        X.resize(dimension);
        F.resize(dimension);
    }

    void setPosition(Point position) {
        X(xIndex) = position.x;
        X(yIndex) = position.y;
    }

    void setBattery(double battery) {
        if (X.size() > 2) {
            X(batteryIndex) = battery;
        }
    }

    void setXYIndex(int xIndex, int yIndex) {
        xIndex = xIndex;
        yIndex = yIndex;
    }

    double x() {
        return X(xIndex);
    }

    double y() {
        return X(yIndex);
    }

    double batt() {
        return X(batteryIndex);
    }

    double camera() {
        return X(cameraIndex);
    }


    Point xy() {
        return Point(X(xIndex), X(yIndex));
    }

    json stepTimeForward(VectorXd &nominalControlInput, double runtime, double dt, World world) {
        json jsonRobot = {
                {"nominal", {
                        {"x", nominalControlInput(xIndex)},
                        {"y", nominalControlInput(yIndex)}
                }}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();
        if (world.isCharging(X) && X(batteryIndex) <= 100.0) {
            X(batteryIndex) += dt * (10);
            jsonRobot["result"] = {{"x", 0.0},
                                   {"y", 0.0}};
            jsonRobot["cbf_no_slack"] = jsonCBFNoSlack;
            jsonRobot["cbf_slack"] = jsonCBFSlack;
            return jsonRobot;
        } else {
            VectorXd optimalControlInput = nominalControlInput;
            try {
                // Create an environment
                GRBEnv env = GRBEnv(true);
//            env.set("LogFile", "energy_cbf.log");
                env.set("OutputFlag", "0");
                env.start();

                // Create an empty model
                GRBModel model = GRBModel(env);

                // Create variables
                std::vector<GRBVar> vars;
                char s[10];
                for (int i = 0; i < X.size(); i++) {
                    snprintf(s, 10, "var_%d", i);
                    vars.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, s));
                }

                std::vector<GRBVar> slackVars;
                for (int i = 0; i < cbfSlack.size(); i++) {
                    snprintf(s, 10, "slack_%d", i);
                    slackVars.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, s));
                }

                // Set objective:
                GRBQuadExpr obj = 0.0;
                for (int i = 0; i < X.size(); i++) {
                    obj += (vars[i] - nominalControlInput[i]) * (vars[i] - nominalControlInput[i]);
                }
                for (int i = 0; i < cbfSlack.size(); i++) {
                    obj += 0.1 * slackVars[i] * slackVars[i];
                }
                model.setObjective(obj, GRB_MINIMIZE);

                // Add constraint:
                for (auto &i: cbfNoSlack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = i.second.constraintUCoe(F, G, X, runtime);
                    for (int j = 0; j < X.size(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    double constraintConstWithTime = i.second.constraintConstWithTime(F, G, X, runtime);
                    jsonCBFNoSlack.push_back({
                                                     {"name",  i.first},
                                                     {"x",     uCoe(xIndex)},
                                                     {"y",     uCoe(yIndex)},
                                                     {"const", constraintConstWithTime}
                                             });
                    model.addConstr(ln, '>', -constraintConstWithTime);
                }
                jsonRobot["cbf_no_slack"] = jsonCBFNoSlack;

                int cnt = 0;
                for (auto &i: cbfSlack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = i.second.constraintUCoe(F, G, X, runtime);
                    for (int j = 0; j < X.size(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
//                ln += u_coe(3) * var_v[3];
                    ln += slackVars[cnt];
                    double constraintConst = i.second.constraintConstWithoutTime(
                            F, G, X, runtime);

                    jsonCBFSlack.push_back({
                                                   {"name",  i.first},
                                                   {"x",     uCoe(xIndex)},
                                                   {"y",     uCoe(yIndex)},
                                                   {"const", constraintConst}
                                           });
                    model.addConstr(ln, '>', -constraintConst);
                    ++cnt;
                }
                jsonRobot["cbf_slack"] = jsonCBFSlack;

                // Optimize model
//                model.set(GRB_IntParam_OutputFlag, 0);
                model.optimize();

                for (int i = 0; i < X.size(); i++) optimalControlInput(i) = vars[i].get(GRB_DoubleAttr_X);
                jsonRobot["result"] = {
                        {"x", optimalControlInput(xIndex)},
                        {"y", optimalControlInput(yIndex)}
                };
                VectorXd slacks(slackVars.size());
                for (int i = 0; i < slackVars.size(); i++) slacks(i) = slackVars[i].get(GRB_DoubleAttr_X);

            } catch (GRBException e) {
                std::cout << "Error code = " << e.getErrorCode() << std::endl;
                printf(".......\n");
                std::cout << e.getMessage() << std::endl;
                assert(0);
            } catch (...) {
                std::cout << "Exception during optimization" << std::endl;
                printf("...................\n");
                assert(0);
            }

            X += (F + G * optimalControlInput) * dt;
            return jsonRobot;
        }
    }

    void output() {
        std::cout << "A UGV @ (" << x() << ", " << y() << ")";
        if (X.size() > 2) {
            std::cout << " with Battery " << X(batteryIndex);
        }
        std::cout << std::endl;
    }

};


#endif //CBF_MAIN_ROBOT_HPP
