#ifndef CBF_MAIN_ROBOT_HPP
#define CBF_MAIN_ROBOT_HPP

//#define OPT_DEBUG

#include "utils.h"
#include "computing_geometry/Point.hpp"
#include "CBF.hpp"
#include "World.hpp"

class Robot{
public:
    int id = 0;
    int x_ord = 0, y_ord = 1, batt_ord = 2, camera_ord = 3;
    std::map<std::string, CBF> cbf_no_slack, cbf_slack;
    MatrixXd G;
    VectorXd F, X;

public:

    Robot() = default;

    Robot(int _d) {
        G = MatrixXd::Identity(_d, _d);
        X.resize(_d);
        F.resize(_d);
    }

    void set_position(Point _p) {
        X(x_ord) = _p.x;
        X(y_ord) = _p.y;
    }

    void set_battery(double _b) {
        if (X.size() > 2) {
            X(batt_ord) = _b;
        }
    }

    void set_xy_order(int _x_ord, int _y_ord) {
        x_ord = _x_ord;
        y_ord = _y_ord;
    }

    double x() {
        return X(x_ord);
    }

    double y() {
        return X(y_ord);
    }

    double batt() {
        return X(batt_ord);
    }

    double camera() {
        return X(camera_ord);
    }


    Point xy() {
        return Point(X(x_ord), X(y_ord));
    }

    json time_forward(VectorXd &_v, double runtime, double _dt, World _w) {
        json robot_j = {
                {"nominal", {
                        {"x", _v(x_ord)},
                        {"y", _v(y_ord)}
                }}
        };
        json cbf_no_slack_json = json::array(), cbf_slack_json = json::array();
        if (_w.is_charging(X) && X(batt_ord) <= 100.0) {
            X(batt_ord) += _dt * (10);
            robot_j["result"] = {{"x", 0.0},
                                 {"y", 0.0}};
            robot_j["cbf_no_slack"] = cbf_no_slack_json;
            robot_j["cbf_slack"] = cbf_slack_json;
            return robot_j;
        } else {
            VectorXd opt_res = _v;
            try {
                // Create an environment
                GRBEnv env = GRBEnv(true);
//            env.set("LogFile", "energy_cbf.log");
                env.set("OutputFlag", "0");
                env.start();

                // Create an empty model
                GRBModel model = GRBModel(env);

                // Create variables
                std::vector<GRBVar> var_v;
                char s[10];
                for (int i = 0; i < X.size(); i++) {
                    sprintf(s, "var_%d", i);
                    var_v.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, s));
                }

                std::vector<GRBVar> var_slack;
                for (int i = 0; i < cbf_slack.size(); i++) {
                    sprintf(s, "slack_%d", i);
                    var_slack.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, s));
                }

                // Set objective:
                GRBQuadExpr obj = 0.0;
                for (int i = 0; i < X.size(); i++) {
                    obj += (var_v[i] - _v[i]) * (var_v[i] - _v[i]);
                }
                for (int i = 0; i < cbf_slack.size(); i++) {
                    obj += 0.1 * var_slack[i] * var_slack[i];
                }
                model.setObjective(obj, GRB_MINIMIZE);

                // Add constraint:
                for (auto &i: cbf_no_slack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd u_coe = i.second.constraint_u_coe(F, G, X, runtime);
#ifdef OPT_DEBUG
                    std::cout << "cbf_no_slack_u_coe: " << std::endl << u_coe << std::endl;
#endif
                    for (int j = 0; j < X.size(); j++) {
                        ln += u_coe(j) * var_v[j];
                    }
                    double constraint_const = i.second.constraint_const_with_time(F, G, X, runtime);
                    cbf_no_slack_json.push_back({
                                                        {"name", i.first},
                                                        {"x",     u_coe(x_ord)},
                                                        {"y",     u_coe(y_ord)},
                                                        {"const", constraint_const}
                                                });
#ifdef OPT_DEBUG
                    std::cout << "cbf_no_slack_const: " << -constraint_const << std::endl;
#endif
                    model.addConstr(ln, '>', -constraint_const);
                }
                robot_j["cbf_no_slack"] = cbf_no_slack_json;

                int cnt = 0;
                for (auto &i: cbf_slack) {
                    GRBLinExpr ln = 0.0;
                    VectorXd u_coe = i.second.constraint_u_coe(F, G, X, runtime);
#ifdef OPT_DEBUG
                    std::cout << "cbf_slack_u_coe: " << std::endl << u_coe << std::endl;
#endif
                    for (int j = 0; j < X.size(); j++) {
                        ln += u_coe(j) * var_v[j];
                    }
//                ln += u_coe(3) * var_v[3];
                    ln += var_slack[cnt];
                    double constraint_const = i.second.constraint_const_without_time(
                            F, G, X, runtime);

                    cbf_slack_json.push_back({
                                                     {"name", i.first},
                                                     {"x",     u_coe(x_ord)},
                                                     {"y",     u_coe(y_ord)},
                                                     {"const", constraint_const}
                                             });
#ifdef OPT_DEBUG
                    std::cout << "cbf_slack_const: " << -constraint_const << std::endl;
#endif
                    model.addConstr(ln, '>', -constraint_const);
                    ++cnt;
                }
                robot_j["cbf_slack"] = cbf_slack_json;

#ifdef OPT_DEBUG
                std::cout << "Now State" << std::endl << X << std::endl;

            std::cout << "Model contains: " << std::endl;
            std::cout << "_v" << std::endl <<  _v << std::endl;
#endif

                // Optimize model
//            model.set(GRB_IntParam_OutputFlag, 0);
                model.optimize();

                for (int i = 0; i < X.size(); i++) opt_res(i) = var_v[i].get(GRB_DoubleAttr_X);
                robot_j["result"] = {{"x", opt_res(x_ord)},
                                     {"y", opt_res(y_ord)}};
                VectorXd sl(var_slack.size());
                for (int i = 0; i < var_slack.size(); i++) sl(i) = var_slack[i].get(GRB_DoubleAttr_X);
#ifdef OPT_DEBUG
                std::cout << "opt_res " << std::endl << opt_res << std::endl;
            std::cout << "slack_res" << std::endl << sl << std::endl;
#endif

//            for (int i = 0; i < X.size(); i++){
//                std::cout << var_v[i].get(GRB_StringAttr_VarName) << " "
//                    << var_v[i].get(GRB_DoubleAttr_X) << std::endl;
//            }
//
//            std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

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

            X += (F + G * opt_res) * _dt;
            return robot_j;
        }
    }

    void output() {
        std::cout << "A UGV @ (" << x() << ", " << y() << ")";
        if (X.size() > 2) {
            std::cout << " with Battery " << X(batt_ord);
        }
        std::cout << std::endl;
    }

};


#endif //CBF_MAIN_ROBOT_HPP
