#ifndef CBF_ROBOT_HPP
#define CBF_ROBOT_HPP

#include "utils.h"
#include "CBF.hpp"
#include "MultiCBF.hpp"
#include "World.hpp"
#include "State.hpp"

class Robot {
public:
    int id = 0;
    MultiCBF cbfNoSlack;
    std::unordered_map<std::string, CBF> cbfSlack;
    MatrixXd G;
    VectorXd F;
    State state;
    State u;
    json opt;
public:

    Robot() = default;

    Robot(int dimension) : state(dimension), u(dimension) {
        G = MatrixXd::Identity(dimension, dimension);
        F.resize(dimension);
        cbfNoSlack.controlVariable = VectorXd::Ones(dimension);
        cbfNoSlack.controlVariable(0) = 1;
        cbfNoSlack.controlVariable(1) = 1;
        cbfNoSlack.controlVariable(2) = 0;
        cbfNoSlack.controlVariable(3) = 0;
    }

    void optimise(VectorXd &nominalControlInput, double runtime, double dt, World world) {
        State inputState(nominalControlInput);
        opt = {
                {"nominal", inputState.toJson()},
                {"result", State(nominalControlInput.size()).toJson()},
                {"cbfNoSlack", json::array()},
                {"cbfSlack", json::array()}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();
        if (world.isCharging(state.xy()) && state.battery() <= 100.0) {
            state.setBattery(state.battery() + dt * (10));
            u.X.setZero();
        } else {
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

                if (!cbfNoSlack.cbfs.empty()) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = cbfNoSlack.constraintUCoe(F, G, state.X, runtime);
                    for (int j = 0; j < state.X.size(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    double constraintConstWithTime = cbfNoSlack.constraintConstWithTime(F, G, state.X, runtime);
                    jsonCBFNoSlack.push_back({
                                                     {"name",  cbfNoSlack.getName()},
                                                     {"coe",   State(uCoe).toJson()},
                                                     {"const", constraintConstWithTime}
                                             });
                    model.addConstr(ln, '>', -constraintConstWithTime);
                }
                opt["cbfNoSlack"] = jsonCBFNoSlack;

                int cnt = 0;
                for (auto &[name, cbf]: cbfSlack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd uCoe = cbf.constraintUCoe(F, G, state.X, runtime);
                    for (int j = 0; j < state.X.size(); j++) {
                        ln += uCoe(j) * vars[j];
                    }
                    ln += slackVars[cnt];
                    double constraintConst = cbf.constraintConstWithoutTime(
                            F, G, state.X, runtime);

                    jsonCBFSlack.push_back({
                                                   {"name",  cbf.name},
                                                   {"coe",   State(uCoe).toJson()},
                                                   {"const", constraintConst}
                                           });
                    model.addConstr(ln, '>', -constraintConst);
                    ++cnt;
                }
                opt["cbfSlack"] = jsonCBFSlack;

//                model.set(GRB_IntParam_OutputFlag, 0);
                model.optimize();

                for (int i = 0; i < state.X.size(); i++) u.X(i) = vars[i].get(GRB_DoubleAttr_X);
                opt["result"] = State(u).toJson();
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
        }
    }

    void stepTimeForward(double dt) {
        state.X += (F + G * u.X) * dt;
    }

    void output() {
        std::cout << "UAV #" << id << " @ (" << state.x() << ", " << state.y() << ")";
        if (state.X.size() > 2) {
            std::cout << " with Battery " << state.battery();
        }
        std::cout << std::endl;
    }

};


#endif //CBF_ROBOT_HPP
