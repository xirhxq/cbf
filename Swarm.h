// Created by XIRHXQ on 12/30/21.

#ifndef CBF_MAIN_SWARM_H
#define CBF_MAIN_SWARM_H

#include "utils.h"
#include "computing_geometry/Polygon.h"
#include "World.h"
#include "Robot.h"

#include <fstream>

class Swarm {
public:
    int n;
    double runtime = 0.0;
    Robot r[maxp];
    World wd;
    double spacing = 0.1;
    std::ofstream data_log;
    json data_j;
public:
    Swarm(int _n, Point _p[], World& _wd);
    Swarm(int _n, World& _wd);

    void output();

    void random_initial_position();
    void set_h();
    void set_h_with_time();

    void init_log_path(char _p[]);
    void end_log();
    void para_log_once();
    void log_once();

    void time_forward(double _t);

    void cvt_forward(double _t);
//    void cvt_forward(double _t, const std::function<double(Point, double)>& f);
    void get_pos(Point _p[]);

    void get_x_limit(double _x[], double inflation = 1.2);
    void get_y_limit(double _y[], double inflation = 1.2);
};


#endif //CBF_MAIN_SWARM_H