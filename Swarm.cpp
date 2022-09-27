// Created by XIRHXQ on 12/30/21.

#include "Swarm.h"
#include "computing_geometry/CVT.h"

#include <fstream>
#include <iomanip>

Swarm::Swarm(int _n, Point *_p, World& _wd){
    n = _n;
    for (int i = 1; i <= _n; i++){
        r[i] = Robot(3);
        r[i].id = i;
        r[i].X << _p[i - 1].x, _p[i - 1].y,  1.0 * (rand() % 40) / 40 + 10;
        r[i].F << 0, 0, -1;
    }
    wd = _wd;
    runtime = 0.0;
}

Swarm::Swarm(int _n, World& _wd) {
    n = _n;
    wd = _wd;
    runtime = 0.0;
    for (int i = 1; i <= _n; i++){
        r[i] = Robot(4);
        r[i].id = i;
        r[i].G(2, 2) = 0;
        r[i].G(3, 3) = 1;
        r[i].set_battery(20.0 * (rand() % 100) / 100 + 10);
        r[i].X(3) = pi;
//        r[i].set_battery(i * 10 + 10);
//        r[i].F << 0, 0, -1;
        r[i].F << 0, 0, -1, 0;
    }
    random_initial_position();
}

void Swarm::output(){
    printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, runtime);
    for (int i = 1; i <= n; i++){
        printf("Robot #%d: ", i);
        r[i].output();
    }
    printf("--------------\n");
}

void Swarm::random_initial_position() {
    for (int i = 1; i <= n; i++){
        r[i].set_position(wd.get_random_point());
    }
}

void Swarm::set_h() {
    std::function<double(Point, World)> nearest_dis = [=](Point _p, World _w){
        return _w.dist_to_charge_place(_p) / _w.charge_place[_w.nearest_charge_place(_p)].second;
    };
    for (int i = 1; i <= n; i++){
        auto dis_h = [=](VectorXd _x, double _t){
            return _x(r[i].batt_ord) -
            log(nearest_dis(Point(_x(r[i].x_ord), _x(r[i].y_ord)), wd));
        };
        CBF energy_cbf;
        energy_cbf.name = "energy_cbf";
        energy_cbf.h = dis_h;
        energy_cbf.alpha = [](double _h) {return _h;};
        energy_cbf.ctrl_var.resize(r[i].X.size());
        energy_cbf.ctrl_var << 1, 1, 1, 1;
        r[i].cbf_no_slack.push_back(energy_cbf);
    }
}

void Swarm::set_h_with_time(){
    std::function<double(Point, double)> angle_to_center = [=](Point _p, double _t) {
        double res = -1, ang = 0.0;
        for (auto i: wd.target){
            if (i.visible(_t)) {
                if (res == -1 || _p.distance_to(i.pos(_t)) < res){
                    res = _p.distance_to(i.pos(_t));
                    ang = _p.angle_to(i.pos(_t));
                }
            }
        }
//        std::cout << "ang = " << 180 / pi * ang << std::endl;
        return ang;
    };
    for (int i = 1; i <= n; i++){
        r[i].cbf_slack.clear();
        std::function<double(VectorXd, double)> camera_angle = [=](VectorXd _x, double _t){
            Point pos{_x(r[i].x_ord), _x(r[i].y_ord)};
            double res = -1, delta_angle = 0.0;
            for (auto tar: wd.target){
                if (!tar.visible(_t)) continue;
                double dis = pos.distance_to(tar.pos(_t));
//                if (dis < 0.5) continue;
                if (res == -1 || dis < res){
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
        r[i].cbf_slack.push_back(camera_cbf);

    }
}

void Swarm::init_log_path(char *_p) {
    data_log.open(_p, std::ios::app);
}

void Swarm::end_log() {
    data_log << std::fixed << std::setprecision(6) << data_j.dump(4);
    data_log.close();
}

void Swarm::para_log_once() {
    data_j["para"]["number"] = n;
    double x_lim[2], y_lim[2];
    get_x_limit(x_lim), get_y_limit(y_lim);
    data_j["para"]["lim"]["x"] = {x_lim[0], x_lim[1]};
    data_j["para"]["lim"]["y"] = {y_lim[0], y_lim[1]};
    get_x_limit(x_lim, 1.0), get_y_limit(y_lim, 1.0);
    for (int i = 1; i <= wd.w.n; i++){
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
    for (auto & i : wd.charge_place){
        data_j["para"]["charge"]["pos"].push_back({
            {"x", i.first.x},
            {"y", i.first.y}
        });
        data_j["para"]["charge"]["dist"].push_back(i.second);
    }
    for (auto & i: wd.target){
        data_j["para"]["target"].push_back({
            {"k", i.den_para["k"]},
            {"r", i.den_para["r"]}
        });
    }
}

void Swarm::log_once() {
    json tmp_j;
    tmp_j["runtime"] = runtime;
    for (int i = 1; i <= n; i++){
        tmp_j["robot"][i - 1] = {
            {"x", r[i].x()},
            {"y", r[i].y()},
            {"batt", r[i].batt()},
            {"camera", r[i].camera()},
            {"energy_cbf", r[i].cbf_no_slack[0].h(r[i].X, runtime)},
            {"camera_cbf", r[i].cbf_slack[0].h(r[i].X, runtime)}
        };
        for (auto h: r[i].cbf_no_slack){
            tmp_j["robot"][i - 1]["cbf_no_slack"].push_back(h.h(r[i].X, runtime));
        }
        for (auto h: r[i].cbf_slack){
            tmp_j["robot"][i - 1]["cbf_slack"].push_back(h.h(r[i].X, runtime));
        }
    }
    for (auto i: wd.target){
        if (i.visible(runtime)){
            tmp_j["target"].push_back({
                {"x", i.pos(runtime).x},
                {"y", i.pos(runtime).y},
                {"k", i.den_para["k"]},
                {"r", i.den_para["r"]}
            });
        }
    }
    data_j["state"].push_back(tmp_j);
}

void Swarm::time_forward(double _t) {
    for (int i = 1; i <= n; i++){
        VectorXd u{3};
        u << 0, 0, 0;
        r[i].time_forward(u, runtime, _t, wd);
    }
    runtime += _t;
}

void Swarm::cvt_forward(double _t) {
    CVT c = CVT(n, wd.w);
    for (int i = 1; i <= n; i++){
        c.pt[i] = r[i].xy();
    }
    c.cal_poly();
    c.cal_centroid([=](const Point& _p) {
        return wd.get_dens(runtime)(_p);
        }, spacing);

    json cvt_j, opt_j;

    for (int i = 1; i <= n; i++){
#ifdef OPT_DEBUG
        std::cout << "Robot #" << i << "(" << r[i].id << ")@ runtime: " << runtime << std::endl;
#endif
        cvt_j[i - 1]["num"] = c.pl[i].n + 1;
        for (int j = 1; j <= c.pl[i].n; j++) {
            cvt_j[i - 1]["pos"].push_back({{"x", c.pl[i].p[j].x}, {"y", c.pl[i].p[j].y}});
        }
        cvt_j[i - 1]["pos"].push_back({{"x", c.pl[i].p[1].x}, {"y", c.pl[i].p[1].y}});
        Point up = ((c.ct[i] - r[i].xy()) * 5).saturation(1.0);
        VectorXd u;
        u.resize(r[i].X.size());
        u.setZero();
        u(r[i].x_ord) = up.x;
        u(r[i].y_ord) = up.y;
//        std::cout << "Robot " << i << std::endl;
//        std::cout << u << std::endl;
        json robot_j = r[i].time_forward(u, runtime, _t, wd);
        opt_j.push_back(robot_j);
    }
    int sz = data_j["state"].size();
    data_j["state"][sz - 1]["cvt"] = cvt_j;
    data_j["state"][sz - 1]["opt"] = opt_j;
    runtime += _t;
}

void Swarm::get_pos(Point _p[]) {
    for (int i = 1; i <= n; i++){
        _p[i - 1] = r[i].xy();
    }
}

void Swarm::get_x_limit(double _x[], double inflation) {
    wd.w.get_x_limit(_x, inflation);
}

void Swarm::get_y_limit(double *_y, double inflation) {
    wd.w.get_y_limit(_y, inflation);
}