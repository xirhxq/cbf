#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

//#define OPT_DEBUG

#include "utils.h"
#include "CBF.hpp"
#include "World.hpp"
#include "State.hpp"

class Robot {
public:
    int id = 0;
    std::map<std::string, CBF> cbfNoSlack, cbfSlack;
    MatrixXd G;
    VectorXd F;
    State state;

public:

    Robot() = default;

    Robot(int dimension) : state(dimension) {
        G = MatrixXd::Identity(dimension, dimension);
        F.resize(dimension);
    }

    json stepTimeForward(VectorXd &nominalControlInput, double runtime, double dt, World world) {
        State inputState(nominalControlInput);
        json jsonRobot = {
                {"nominal",    inputState.toJson()},
                {"result",     State(nominalControlInput.size()).toJson()},
                {"cbfNoSlack", json::array()},
                {"cbfSlack",   json::array()}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();
        if (world.isCharging(state.xy()) && state.battery() <= 100.0) {
            state.setBattery(state.battery() + dt * (10));
            return jsonRobot;
        } else {
            VectorXd optimalControlInput = nominalControlInput;
            try {
                GRBEnv env = GRBEnv(true);
                env.set("OutputFlag", "0");
                env.start();

                GRBModel model = GRBModel(env);

                std::vector<GRBVar> vars;
                char s[10];
                for (int i = 0; i < state.X.size(); i++) {
                    snprintf(s, 10, "var-%d", i);
                    vars.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, s));
                }

                std::vector<GRBVar> slackVars;
                for (int i = 0; i < cbfSlack.size(); i++) {
                    snprintf(s, 10, "slack-%d", i);
                    slackVars.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, s));
                }

                GRBQuadExpr obj = 0.0;
                for (int i = 0; i < state.X.size(); i++) {
                    obj += (vars[i] - nominalControlInput[i]) * (vars[i] - nominalControlInput[i]);
                }
                for (int i = 0; i < cbfSlack.size(); i++) {
                    obj += 0.1 * slackVars[i] * slackVars[i];
                }
                model.setObjective(obj, GRB_MINIMIZE);

                for (auto &i: cbfNoSlack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = i.second.constraintUCoe(F, G, state.X, runtime);
                    for (int j = 0; j < state.X.size(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    double constraintConstWithTime = i.second.constraintConstWithTime(F, G, state.X, runtime);
                    jsonCBFNoSlack.push_back({
                                                     {"name",  i.first},
                                                     {"coe",   State(uCoe).toJson()},
                                                     {"const", constraintConstWithTime}
                                             });
                    model.addConstr(ln, '>', -constraintConstWithTime);
                }
                jsonRobot["cbfNoSlack"] = jsonCBFNoSlack;

                int cnt = 0;
                for (auto &i: cbfSlack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = i.second.constraintUCoe(F, G, state.X, runtime);
                    for (int j = 0; j < state.X.size(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    ln += slackVars[cnt];
                    double constraintConst = i.second.constraintConstWithoutTime(
                            F, G, state.X, runtime);

                    jsonCBFSlack.push_back({
                                                   {"name",  i.first},
                                                   {"coe",   State(uCoe).toJson()},
                                                   {"const", constraintConst}
                                           });
                    model.addConstr(ln, '>', -constraintConst);
                    ++cnt;
                }
                jsonRobot["cbfSlack"] = jsonCBFSlack;

//                model.set(GRB_IntParam_OutputFlag, 0);
                model.optimize();

                for (int i = 0; i < state.X.size(); i++) optimalControlInput(i) = vars[i].get(GRB_DoubleAttr_X);
                jsonRobot["result"] = State(optimalControlInput).toJson();
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

            state.X += (F + G * optimalControlInput) * dt;
            return jsonRobot;
        }
    }

    void output() {
        std::cout << "A UGV @ (" << state.x() << ", " << state.y() << ")";
        if (state.X.size() > 2) {
            std::cout << " with Battery " << state.battery();
        }
        std::cout << std::endl;
    }

};


#endif //CBF_ROBOT_HPP
