// Created by XIRHXQ on 12/30/21.

#include "Swarm.h"

#include <fstream>
#include <iomanip>

Swarm::Swarm(int _n, Point *_p, World &_wd) {
    n = _n;
    for (int i = 1; i <= _n; i++) {
        r[i] = Robot(3);
        r[i].id = i;
        r[i].X << _p[i - 1].x, _p[i - 1].y, 1.0 * (rand() % 40) / 40 + 10;
        r[i].F << 0, 0, -1;
    }
    wd = _wd;
    auto wd_x_lim_pd = wd.w.get_x_limit(1.0), wd_y_lim_pd = wd.w.get_y_limit(1.0);
    int x_num = (wd_x_lim_pd.second - wd_x_lim_pd.first) / spacing;
    int y_num = (wd_y_lim_pd.second - wd_y_lim_pd.first) / spacing;
    gw = GridWorld(wd_x_lim_pd, x_num, wd_y_lim_pd, y_num);
    runtime = 0.0;
}

Swarm::Swarm(int _n, World &_wd) {
    n = _n;
    wd = _wd;
    auto wd_x_lim_pd = wd.w.get_x_limit(1.0), wd_y_lim_pd = wd.w.get_y_limit(1.0);
    int x_num = (wd_x_lim_pd.second - wd_x_lim_pd.first) / spacing;
    int y_num = (wd_y_lim_pd.second - wd_y_lim_pd.first) / spacing;
    gw = GridWorld(wd_x_lim_pd, x_num, wd_y_lim_pd, y_num);
    runtime = 0.0;
    for (int i = 1; i <= _n; i++) {
        r[i] = Robot(4);
        r[i].id = i;
        r[i].G(2, 2) = 0;
        r[i].G(3, 3) = 1;
        r[i].set_battery(20.0 * (rand() % 100) / 100 + 10);
        r[i].X(3) = pi;
//        r[i].set_battery(i * 10 + 10);
//        r[i].F << 0, 0, -1;
        r[i].F << 0, 0, -1, 0;
        r[i].cbf_slack.clear();
        r[i].cbf_no_slack.clear();
    }
    random_initial_position();
}

void Swarm::output() {
    printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, runtime);
    for (int i = 1; i <= n; i++) {
        printf("Robot #%d: ", i);
        r[i].output();
    }
    printf("--------------\n");
}

void Swarm::random_initial_position() {
    for (int i = 1; i <= n; i++) {
        r[i].set_position(wd.get_random_point());
    }
}

void Swarm::random_initial_position(Polygon _p) {
    for (int i = 1; i <= n; i++) {
        r[i].set_position(_p.get_random_point());
    }
}

void Swarm::set_initial_position(std::vector<Point> _v) {
    for (int i = 1; i <= n; i++) {
        r[i].set_position(_v[i - 1]);
    }
}

void Swarm::set_energy_cbf() {
    std::function<double(Point, World)> nearest_dis = [=](Point _p, World _w) {
        return _w.dist_to_charge_place(_p) / _w.charge_place[_w.nearest_charge_place(_p)].second;
    };
    auto dis_to_base = [=](int x) {
        return r[x].xy().len();
    };
    auto part_id = [=](int x) { return (x - 1) % (n / 2) + 1; };
    auto part = [=](int x) { return x > (n / 2); };
    for (int i = 1; i <= n; i++) {
//        printf("set energy cbf of #%d (%d, %d)\n", i, part(i), part_id(i));
        auto swarm_comm_h = [=](VectorXd _x, double _t) {
            std::vector<int> ind_v;
            for (int j = 1; j <= n; j++) {
                if (i == j) continue;
                if (r[i].xy().distance_to(r[j].xy()) > 8) continue;
                ind_v.push_back(j);
            }
            std::sort(ind_v.begin(), ind_v.end(), [=](int a, int b) {
                return dis_to_base(a) < dis_to_base(b);
            });
//            printf("Distance sort:");
//            for (auto &a: ind_v){
//                printf("%d ", a);
//            }
//            printf("\n");
            double res = inf;
            int loc = std::distance(ind_v.begin(), std::find(ind_v.begin(), ind_v.end(), i));
            if (loc < 2) {
//            if (part_id(i) <= 2) {
//                printf("%d -> base\n", i);
                res = std::min(res,
                               0.5 * (8 - Point(_x(r[i].x_ord),
                                                _x(r[i].y_ord)).distance_to(Point(0, 0))));
            }
            for (int t = std::max(0, loc - 2); t < loc; t++) {
                int j = ind_v[t];
//            for (int j = 1; j <= n; j++) {
//                if (i == j) continue;
//                if (i > j - 1 || i < j - 2) continue;
//                if (part(i) != part(j)) continue;
//                if (part_id(i) > part_id(j) - 1) continue;
//                if (part_id(i) < part_id(j) - 2) continue;
//                printf("%d -> %d\n", i, j);
                res = std::min(res,
                               0.5 * (8 - Point(_x(r[i].x_ord),
                                                _x(r[i].y_ord)).distance_to(r[j].xy())));
            }
            return res;
        };
        auto fix_comm_h = [=](VectorXd _x, double _t) {
            double res = inf;
            double max_comm_dis = 8.5;
            int pid = part_id(i);
            if (pid == 1) {
//                printf("%d to Base@(%d, 0)\n", i, -3 + 6 * part(i));
                res = std::min(res,
                               0.5 * (max_comm_dis - Point(_x(r[i].x_ord),
                                                           _x(r[i].y_ord))
                                                     .distance_to(Point(-3 + 6 * part(i), 0))));
            }
            if (pid <= 2) {
//                printf("%d to Base@(0, 0)\n", i);
                res = std::min(res,
                               0.5 * (max_comm_dis - Point(_x(r[i].x_ord),
                                                           _x(r[i].y_ord))
                                                     .distance_to(Point(0, 0))));
            }
            for (int t = std::max(1, pid - 2); t <= pid + 2 && t <= n / 2; t++) {
                if (t <= pid) continue;
//                printf("%d to %d\n", i, t + part(i) * n / 2);
                res = std::min(res, 0.5 * (max_comm_dis - Point(_x(r[i].x_ord),
                                                                _x(r[i].y_ord))
                                                          .distance_to(r[t + part(i) * n / 2].xy())));
            }
            return res;
        };
        auto fix_comm_h_realexp = [=](VectorXd _x, double _t) {
            double res = inf;
            double max_comm_dis = 10;
            if (i == 1) {
                res = std::min(
                        res,
                        0.5 * (
                                max_comm_dis -
                                Point(_x(r[i].x_ord), _x(r[i].y_ord))
                                .distance_to(Point(5, 0))
                        )
                );
            }
            if (i <= 2) {
                res = std::min(
                        res,
                        0.5 * (
                                max_comm_dis -
                                Point(_x(r[i].x_ord), _x(r[i].y_ord))
                                .distance_to(Point(-5, 0))
                        )
                );
            }
            for (int t = std::max(1, i - 2); t < i; t++) {
                res = std::min(
                        res,
                        0.5 * (
                                max_comm_dis -
                                Point(_x(r[i].x_ord),_x(r[i].y_ord))
                                .distance_to(r[t].xy())
                        )
                );
            }
            return res;
        };
        auto battery_h = [=](VectorXd _x, double _t) {
            double res = inf;
            res = std::min(res, _x(r[i].batt_ord) -
                                log(nearest_dis(Point(_x(r[i].x_ord), _x(r[i].y_ord)), wd)));
            return res;
        };
        auto safety_h = [=](VectorXd _x, double _t) {
            double res = inf;
            for (int j = 1; j <= n; j++) {
                if (i == j) continue;
                res = std::min(res,
                               0.5 * (Point(_x(r[i].x_ord),
                                            _x(r[i].y_ord)).distance_to(r[j].xy()) - 3));
            }
            return res;
        };
        auto energy_h = [=](VectorXd _x, double _t) {
            double res = inf;
            res = std::min(res, battery_h(_x, _t));
//            res = std::min(res, fix_comm_h(_x, _t));
//            res = std::min(res, fix_comm_h_realexp(_x, _t));
            return res;
        };
        CBF energy_cbf;
        energy_cbf.name = "energy_cbf";
        energy_cbf.h = energy_h;
        energy_cbf.alpha = [](double _h) { return _h; };
        energy_cbf.ctrl_var.resize(r[i].X.size());
        energy_cbf.ctrl_var << 1, 1, 1, 1;
        r[i].cbf_no_slack["energy_cbf"] = energy_cbf;
    }
}

void Swarm::set_camera_cbf() {
    std::function<double(Point, double)> angle_to_center = [=](Point _p, double _t) {
        double res = -1, ang = 0.0;
        for (auto i: wd.target) {
            if (i.visible(_t)) {
                if (res == -1 || _p.distance_to(i.pos(_t)) < res) {
                    res = _p.distance_to(i.pos(_t));
                    ang = _p.angle_to(i.pos(_t));
                }
            }
        }
//        std::cout << "ang = " << 180 / pi * ang << std::endl;
        return ang;
    };
    for (int i = 1; i <= n; i++) {
        std::function<double(VectorXd, double)> camera_angle = [=](VectorXd _x, double _t) {
            Point pos{_x(r[i].x_ord), _x(r[i].y_ord)};
            double res = -1, delta_angle = 0.0;
            for (auto tar: wd.target) {
                if (!tar.visible(_t)) continue;
                double dis = pos.distance_to(tar.pos(_t));
//                if (dis < 0.5) continue;
                if (res == -1 || dis < res) {
                    res = dis;
                    delta_angle = _x(r[i].camera_ord) - pos.angle_to(tar.pos(_t));
                }
            }
//            double delta_angle = _x(r[i].camera_ord)
//                                 - angle_to_center(Point(_x(r[i].x_ord),
//                                                     _x(r[i].y_ord)),
//                                                   _t);
            if (delta_angle <= -pi) delta_angle += 2 * pi;
            else if (delta_angle >= pi) delta_angle -= 2 * pi;
//            std::cout << "delta_angle = " << delta_angle << std::endl;
            return -5 * abs(delta_angle);
        };
        CBF camera_cbf;
        camera_cbf.name = "camera_cbf";
        camera_cbf.h = camera_angle;
        camera_cbf.ctrl_var.resize(r[i].X.size());
        camera_cbf.ctrl_var << 0, 0, 0, 1;
//        camera_cbf.ctrl_var << 1, 1, 1, 1;
//        camera_cbf.alpha = [](double _h) {return _h;};
        r[i].cbf_slack["camera_cbf"] = camera_cbf;

    }
}

void Swarm::set_comm_cbf() {
    CBF comm_cbf;
    comm_cbf.ctrl_var.resize(r[1].X.size());
    comm_cbf.ctrl_var << 1, 1, 1, 1;

    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++) {
            if (i == j) continue;
            if ((i > 3) != (j > 3)) continue;
            comm_cbf.name = "comm_to_" + std::to_string(j);
            comm_cbf.h = [=](VectorXd _x, double _t) {
                return 2 - Point(_x(r[i].x_ord), _x(r[i].y_ord)).distance_to(r[j].xy());
            };
            r[i].cbf_slack[comm_cbf.name] = comm_cbf;
        }
    }
}

void Swarm::set_safety_cbf() {
    CBF safety_cbf;
    safety_cbf.ctrl_var.resize(r[1].X.size());
    safety_cbf.ctrl_var << 1, 1, 1, 1;

    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++) {
            if (i == j) continue;
            safety_cbf.name = "safe_to_" + std::to_string(j);
            safety_cbf.h = [=](VectorXd _x, double _t) {
                return 5.0 * (Point(_x(r[i].x_ord), _x(r[i].y_ord)).distance_to(r[j].xy()) - 3);
            };
            r[i].cbf_slack[safety_cbf.name] = safety_cbf;
        }
    }
}

void Swarm::init_log_path(char *_p) {
    data_log.open(_p, std::ios::app);
}

void Swarm::end_log() {
    data_log << std::fixed << std::setprecision(6) << data_j;
    data_log.close();
}

void Swarm::para_log_once() {
    data_j["para"]["number"] = n;
    double x_lim[2], y_lim[2];
    get_x_limit(x_lim), get_y_limit(y_lim);
    data_j["para"]["lim"]["x"] = {x_lim[0], x_lim[1]};
    data_j["para"]["lim"]["y"] = {y_lim[0], y_lim[1]};
    get_x_limit(x_lim, 1.0), get_y_limit(y_lim, 1.0);
    for (int i = 1; i <= wd.w.n; i++) {
        data_j["para"]["world"].push_back({
                                                  {"x", wd.w.p[i].x},
                                                  {"y", wd.w.p[i].y}
                                          });
    }
    data_j["para"]["world"].push_back({
                                              {"x", wd.w.p[1].x},
                                              {"y", wd.w.p[1].y}
                                      });
    data_j["para"]["charge"]["num"] = wd.charge_place.size();
    for (auto &i: wd.charge_place) {
        data_j["para"]["charge"]["pos"].push_back({
                                                          {"x", i.first.x},
                                                          {"y", i.first.y}
                                                  });
        data_j["para"]["charge"]["dist"].push_back(i.second);
    }
    for (auto &i: wd.target) {
        data_j["para"]["target"].push_back({
                                                   {"k", i.den_para["k"]},
                                                   {"r", i.den_para["r"]}
                                           });
    }
    data_j["para"]["grid_world"] = {
            {"x_num", gw.x_num},
            {"y_num", gw.y_num},
            {"x_lim", {gw.x_lim.first, gw.x_lim.second}},
            {"y_lim", {gw.y_lim.first, gw.y_lim.second}}
    };
}

void Swarm::log_once() {
    json tmp_j;
    tmp_j["runtime"] = runtime;
    for (int i = 1; i <= n; i++) {
        tmp_j["robot"][i - 1] = {
                {"x",      r[i].x()},
                {"y",      r[i].y()},
                {"batt",   r[i].batt()},
                {"camera", r[i].camera()}
        };
        for (auto cbf: r[i].cbf_no_slack) {
            auto h = cbf.second;
            tmp_j["robot"][i - 1][cbf.first] = h.h(r[i].X, runtime);
        }
        for (auto cbf: r[i].cbf_slack) {
            auto h = cbf.second;
            tmp_j["robot"][i - 1][cbf.first] = h.h(r[i].X, runtime);
        }
        tmp_j["cvt"][i - 1]["num"] = c.pl[i].n + 1;
        for (int j = 1; j <= c.pl[i].n; j++) {
            tmp_j["cvt"][i - 1]["pos"].push_back({{"x", c.pl[i].p[j].x},
                                                  {"y", c.pl[i].p[j].y}});
        }
        tmp_j["cvt"][i - 1]["pos"].push_back({{"x", c.pl[i].p[1].x},
                                              {"y", c.pl[i].p[1].y}});
        tmp_j["cvt"][i - 1]["center"] = {{"x", c.ct[i].x},
                                         {"y", c.ct[i].y}};
    }
    for (auto i: wd.target) {
        if (i.visible(runtime)) {
            tmp_j["target"].push_back({
                                              {"x", i.pos(runtime).x},
                                              {"y", i.pos(runtime).y},
                                              {"k", i.den_para["k"]},
                                              {"r", i.den_para["r"]}
                                      });
        }
    }
//    json data_grid_world;
//    for (int i = 0; i < gw.x_num; i++) {
//        json tmp_j = json::array();
//        for (int j = 0; j < gw.y_num; j++) {
//            tmp_j.push_back(gw.get_result(i, j) == true ? gw.true_weight : gw.false_weight);
//        }
//        data_grid_world.push_back(tmp_j);
//    }
//    tmp_j["grid_world"] = data_grid_world;
    tmp_j["update"] = update_j;
    data_j["state"].push_back(tmp_j);
}

void Swarm::time_forward(double _t) {
    for (int i = 1; i <= n; i++) {
        VectorXd u{3};
        u << 0, 0, 0;
        r[i].time_forward(u, runtime, _t, wd);
    }
    runtime += _t;
}

void Swarm::cal_cvt() {
    c = CVT(n, wd.w);
    for (int i = 1; i <= n; i++) {
        c.pt[i] = r[i].xy();
    }
    c.cal_poly();
//    c.cal_centroid([=](const Point &_p) {
//        return wd.get_dens(runtime)(_p);
//    }, spacing);
    for (int i = 1; i <= c.n; i++) {
        c.ct[i] = gw.get_centroid_in_polygon(c.pl[i]);
    }
}

void Swarm::set_cvt_cbf() {
    for (int i = 1; i <= n; i++) {
        CBF cvt_cbf;
        cvt_cbf.ctrl_var.resize(r[i].X.size());
        cvt_cbf.ctrl_var << 1, 1, 1, 1;
        cvt_cbf.name = "cvt_cbf";
        cvt_cbf.h = [=](VectorXd _x, double _t) {
            return -5.0 * c.ct[i].distance_to(Point(_x(r[i].x_ord), _x(r[i].y_ord)));
        };
        cvt_cbf.alpha = [](double _h) { return _h; };
        r[i].cbf_slack[cvt_cbf.name] = cvt_cbf;
    }
}

void Swarm::cvt_forward(double _t) {

    json opt_j;

    for (int i = 1; i <= n; i++) {
#ifdef OPT_DEBUG
        std::cout << "Robot #" << i << "(" << r[i].id << ")@ runtime: " << runtime << std::endl;
#endif
        Point up = ((c.ct[i] - r[i].xy()) * 5).saturation(1.0);
        VectorXd u;
        u.resize(r[i].X.size());
        u.setZero();
//        u(r[i].x_ord) = up.x;
//        u(r[i].y_ord) = up.y;
        json robot_j = r[i].time_forward(u, runtime, _t, wd);
        opt_j.push_back(robot_j);
    }
    int sz = data_j["state"].size();
    data_j["state"][sz - 1]["opt"] = opt_j;
    runtime += _t;
}

void Swarm::get_pos(Point _p[]) {
    for (int i = 1; i <= n; i++) {
        _p[i - 1] = r[i].xy();
    }
}

void Swarm::get_x_limit(double _x[], double inflation) {
    wd.w.get_x_limit(_x, inflation);
}

void Swarm::get_y_limit(double *_y, double inflation) {
    wd.w.get_y_limit(_y, inflation);
}

void Swarm::update_vis() {
    update_j = json::array();
    for (int i = 1; i <= n; i++) {
        double tol = 2;
        Point deny_p[4] = {{r[i].x() - tol, r[i].y() - tol},
                           {r[i].x() + tol, r[i].y() - tol},
                           {r[i].x() + tol, r[i].y() + tol},
                           {r[i].x() - tol, r[i].y() + tol}};
//        update_j.push_back(gw.set_res_in_polygon(Polygon(4, deny_p), true, true));
        update_j.push_back(gw.set_res_in_radii(r[i].xy(), tol, true, true));
    }
}

void Swarm::grid_world_output() {
    system("clear");
//    std::cout << "\033[2J\033[H";
    std::vector<std::vector<char> > v;
    int y_num = 60, x_num = 150;
    v.resize(y_num);
    for (auto &a: v) {
        a.resize(x_num);
    }
    for (int j = y_num - 1; j >= 0; j--) {
        for (int i = 0; i < x_num; i++) {
            v[j][i] = (gw.get_result(i * gw.x_num / x_num,
                                     j * gw.y_num / y_num) == true) ?
                      ' ' : '.';
        }
    }
    for (int i = 1; i <= n; i++) {
        v[gw.get_num_in_ylim(r[i].y()) * y_num / gw.y_num]
        [gw.get_num_in_xlim(r[i].x()) * x_num / gw.x_num] = 'a' + i - 1;
    }
    for (int j = y_num - 1; j >= 0; j--) {
        for (int i = 0; i < x_num; i++) {
            if (isalpha(v[j][i])) {
                std::cout << RED;
                printf("%c", v[j][i]);
                std::cout << GREEN;
            } else {
                printf("%c", v[j][i]);
            }
        }
        printf("\n");
    }
}