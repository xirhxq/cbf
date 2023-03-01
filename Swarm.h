// Created by XIRHXQ on 12/30/21.

#ifndef CBF_MAIN_SWARM_H
#define CBF_MAIN_SWARM_H

#include "utils.h"
#include "computing_geometry/Polygon.h"
#include "computing_geometry/CVT.h"
#include "World.h"
#include "Robot.h"
#include "GridWorld.h"

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
    CVT c;
    GridWorld gw;
    json update_j;
public:
    Swarm(int _n, Point _p[], World& _wd);
    Swarm(int _n, World& _wd);

    void output();

    void random_initial_position();
    void random_initial_position(Polygon _p);
    void set_initial_position(std::vector<Point> _v);

    void set_energy_cbf();
    void set_camera_cbf();
    void set_comm_cbf();
    void set_safety_cbf();
    void set_cvt_cbf();

    void init_log_path(char _p[]);
    void end_log();
    void para_log_once();
    void log_once();

    void time_forward(double _t);
    void cal_cvt();
    void cvt_forward(double _t);
//    void cvt_forward(double _t, const std::function<double(Point, double)>& f);
    void get_pos(Point _p[]);

    void get_x_limit(double _x[], double inflation = 1.2);
    void get_y_limit(double _y[], double inflation = 1.2);

    void update_vis();

    void grid_world_output();
};


#endif //CBF_MAIN_SWARM_H