//
// Created by xirhxq on 2022/3/3.
//

#ifndef CBF_MAIN_ROBOT_H
#define CBF_MAIN_ROBOT_H

//#define OPT_DEBUG

#include "utils.h"
#include "computing_geometry/Point.h"
#include "CBF.h"
#include "World.h"

class Robot{
public:
    int id = 0;
    int x_ord = 0, y_ord = 1, batt_ord = 2, camera_ord = 3;
    std::vector<CBF> cbf_no_slack, cbf_slack;
    MatrixXd G;
    VectorXd F, X;

public:
    Robot();
    Robot(int);

    void set_position(Point);
    void set_battery(double);

    void set_xy_order(int, int);

    double x();
    double y();
    double batt();
    double camera();

    Point xy();

    json time_forward(VectorXd &_v, double runtime, double _dt, World _w);

    void output();
};


#endif //CBF_MAIN_ROBOT_H
