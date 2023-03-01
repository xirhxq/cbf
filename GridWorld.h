//
// Created by xirhxq on 2022/10/3.
//

#ifndef CBF_MAIN_GRIDWORLD_H
#define CBF_MAIN_GRIDWORLD_H

//#define GRIDWORLD_DEBUG

#include "utils.h"
#include "computing_geometry/Point.h"
#include "computing_geometry/Polygon.h"

class GridWorld {
public:
    std::pair<double, double> x_lim, y_lim;
    int x_num{}, y_num{};
    std::vector<bool> vis;
    double true_weight = 0.0, false_weight = 1.0;

public:
    GridWorld();
    GridWorld(pd _x_lim, int _x_num,
              pd _y_lim, int _y_num);

    void reset(bool _res = false);
    void output(char c = '*');

    int get_num_in_lim(double _a, pd _lim, int _sz, std::string _mode = "round");
    int get_num_in_xlim(double _x, std::string _mode = "round");
    int get_num_in_ylim(double _y, std::string _mode = "round");
    int get_ind(int _x_ind, int _y_ind) const;
    int get_x_ind(int _ind) const;
    int get_y_ind(int _ind) const;
    bool get_result(Point _p);
    bool get_result(int _ind);
    bool get_result(int _x_ind, int _y_ind);
    void set_result(Point _p, bool _res);
    void set_result(int _x_ind, int _y_ind, bool _res);
    void set_result(int _ind, bool _res);
    double get_pos_in_lim(int _ind, pd _lim, int _sz);
    double get_xpos_in_xlim(int _x_ind);
    double get_ypos_in_ylim(int _y_ind);
    Point get_point_in_area(int _x_ind, int _y_ind);
    Point get_point_in_area(int _ind);

    double get_res_in_polygon(Polygon _p);
    json set_res_in_polygon(Polygon _p, bool _res, bool _update_json = false);
    Point get_centroid_in_polygon(Polygon _p);

    void output_centroid_in_polygon(Polygon _p);
};


#endif //CBF_MAIN_GRIDWORLD_H
