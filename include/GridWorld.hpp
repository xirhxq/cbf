#ifndef CBF_MAIN_GRIDWORLD_HPP
#define CBF_MAIN_GRIDWORLD_HPP

//#define GRIDWORLD_DEBUG

#include "utils.h"
#include "computing_geometry/Point.hpp"
#include "computing_geometry/Polygon.hpp"

class GridWorld {
public:
    std::pair<double, double> x_lim, y_lim;
    int x_num{}, y_num{};
    std::vector<bool> vis;
    double true_weight = 0.0, false_weight = 1.0;

public:
    GridWorld() {

    }
    GridWorld(pd _x_lim, int _x_num, pd _y_lim, int _y_num) {
        x_lim = _x_lim;
        y_lim = _y_lim;
        x_num = _x_num;
        y_num = _y_num;
        vis.resize(_x_num * _y_num);
        reset();
    }

    void reset(bool _res = false) {
        for (auto a: vis) {
            a = _res;
        }
    }

    void output(char c = '*') {
        printf("< Gridworld of (%.2lf-%.2lf, %.2lf-%.2lf) -> (%d, %d) grids",
               x_lim.first, x_lim.second, y_lim.first, y_lim.second, x_num, y_num);
        printf("\n");
        for (int j = y_num - 1; j >= 0; j--) {
            for (int i = 0; i < x_num; i++) {
                printf("%c", (get_result(i, j) == true) ? c : '.');
            }
            printf("\n");
        }
        for (int x_ind = 0; x_ind < x_num; x_ind++) {
            printf("-");
        }
        printf(">\n");
    }

    int get_num_in_lim(double _a, pd _lim, int _sz, std::string _mode = "round") {
        double ratio = (_a - _lim.first) / (_lim.second - _lim.first);
        int ret = lround(ratio * _sz);
        if (_mode == "ceil") {
            ret = ceil(ratio * _sz);
        } else if (_mode == "floor") {
            ret = floor(ratio * _sz);
        }
        ret = std::max(std::min(ret, _sz - 1), 0);
//    printf("ratio = %.2lf ret = %d\n", ratio, ret);
        return ret;
    }

    int get_num_in_xlim(double _x, std::string _mode = "round") {
        return get_num_in_lim(_x, x_lim, x_num, _mode);
    }

    int get_num_in_ylim(double _y, std::string _mode = "round") {
        return get_num_in_lim(_y, y_lim, y_num, _mode);
    }

    int get_ind(int _x_ind, int _y_ind) const {
        int ind = _x_ind * y_num + _y_ind;
        return ind;
    }

    int get_x_ind(int _ind) const {
        int x_ind = _ind / y_num;
        return x_ind;
    }

    int get_y_ind(int _ind) const {
        int y_ind = _ind % y_num;
        return y_ind;
    }

    bool get_result(Point _p) {
        int x_ind = get_num_in_xlim(_p.x);
        int y_ind = get_num_in_ylim(_p.y);
        int ind = get_ind(x_ind, y_ind);
//    printf("(%d, %d) == (%d) -> %d\n", x_ind, y_ind, ind, bool(vis[ind]));
        return vis[ind] == true;
    }

    bool get_result(int _ind) {
        return vis[_ind] == true;
    }

    bool get_result(int _x_ind, int _y_ind) {
        _x_ind = std::max(0, std::min(_x_ind, x_num - 1));
        _y_ind = std::max(0, std::min(_y_ind, y_num - 1));
        int ind = get_ind(_x_ind, _y_ind);
        return vis[ind] == true;
    }

    void set_result(Point _p, bool _res) {
        int x_ind = get_num_in_xlim(_p.x);
        int y_ind = get_num_in_ylim(_p.y);
        vis[get_ind(x_ind, y_ind)] = _res;
    }

    void set_result(int _x_ind, int _y_ind, bool _res) {
        vis[get_ind(_x_ind, _y_ind)] = _res;
    }

    void set_result(int _ind, bool _res) {
        vis[_ind] = _res;
    }

    double get_pos_in_lim(int _ind, pd _lim, int _sz) {
        double ratio = 1.0 * _ind / _sz;
        double pos = _lim.first * (1.0 - ratio) + _lim.second * ratio;
        return pos;
    }

    double get_xpos_in_xlim(int _x_ind) {
        return get_pos_in_lim(_x_ind, x_lim, x_num);
    }

    double get_ypos_in_ylim(int _y_ind) {
        return get_pos_in_lim(_y_ind, y_lim, y_num);
    }

    Point get_point_in_area(int _x_ind, int _y_ind) {
        return {get_xpos_in_xlim(_x_ind), get_ypos_in_ylim(_y_ind)};
    }

    Point get_point_in_area(int _ind) {
        return {get_xpos_in_xlim(get_x_ind(_ind)), get_ypos_in_ylim(get_y_ind(_ind))};
    }

    double get_res_in_polygon(Polygon _p) {
        double res = 0;
        pd x_lim_pd = _p.get_x_limit(1.0), y_lim_pd;
        pd x_ind_pd, y_ind_pd;
        x_ind_pd.first = get_num_in_xlim(x_lim_pd.first, "ceil");
        x_ind_pd.second = get_num_in_xlim(x_lim_pd.second, "floor");
        for (int x_ind = x_ind_pd.first; x_ind <= x_ind_pd.second; x_ind++) {
            double x_pos = get_xpos_in_xlim(x_ind);
            y_lim_pd = _p.get_y_lim_at_certain_x(x_pos);
            y_ind_pd.first = get_num_in_ylim(y_lim_pd.first, "ceil");
            y_ind_pd.second = get_num_in_ylim(y_lim_pd.second, "floor");
            for (int y_ind = y_ind_pd.first; y_ind <= y_ind_pd.second; y_ind++) {
                res += (get_result(x_ind, y_ind) == true) ? true_weight : false_weight;
#ifdef GRIDWORLD_DEBUG
                printf("(%d, %d) -> %d res = %.2lf\n", x_ind, y_ind, get_result(x_ind, y_ind) == true, res);
#endif
            }
        }
        return res;
    }

    json set_res_in_polygon(Polygon _p, bool _res, bool _update_json = false) {
        json ret = json::array();
        pd x_lim_pd = _p.get_x_limit(1.0), y_lim_pd;
        pd x_ind_pd, y_ind_pd;
        x_ind_pd.first = get_num_in_xlim(x_lim_pd.first, "ceil");
        x_ind_pd.second = get_num_in_xlim(x_lim_pd.second, "floor");
//    printf("x_lim: (%.12lf, %.12lf)\tx_ind: (%lf, %lf)\n", x_lim_pd.first, x_lim_pd.second, x_ind_pd.first, x_ind_pd.second);
        for (int x_ind = x_ind_pd.first; x_ind <= x_ind_pd.second; x_ind++) {
            double x_pos = get_xpos_in_xlim(x_ind);
            if (x_pos > x_lim_pd.second || x_pos < x_lim_pd.first) continue;
//        printf("x_ind = %d, x_pos = %.12lf\n", x_ind, x_pos);
            y_lim_pd = _p.get_y_lim_at_certain_x(x_pos);
            y_ind_pd.first = get_num_in_ylim(y_lim_pd.first, "ceil");
            y_ind_pd.second = get_num_in_ylim(y_lim_pd.second, "floor");
            for (int y_ind = y_ind_pd.first; y_ind <= y_ind_pd.second; y_ind++) {
                if (_update_json && get_result(x_ind, y_ind) != _res) {
                    ret.push_back({{"x", x_ind},
                                   {"y", y_ind}});
                }
                set_result(x_ind, y_ind, _res);
            }
        }
        return ret;
    }

    json set_res_in_radii(Point _pt, double _r, bool _res, bool _update_json = false) {
        Point p[4] = {{_pt.x - _r, _pt.y - _r},
                      {_pt.x + _r, _pt.y - _r},
                      {_pt.x + _r, _pt.y + _r},
                      {_pt.x - _r, _pt.y + _r}};
        Polygon _p = Polygon(4, p);
        json ret = json::array();
        pd x_lim_pd = _p.get_x_limit(1.0), y_lim_pd;
        pd x_ind_pd, y_ind_pd;
        x_ind_pd.first = get_num_in_xlim(x_lim_pd.first, "ceil");
        x_ind_pd.second = get_num_in_xlim(x_lim_pd.second, "floor");
//    printf("x_lim: (%.12lf, %.12lf)\tx_ind: (%lf, %lf)\n", x_lim_pd.first, x_lim_pd.second, x_ind_pd.first, x_ind_pd.second);
        for (int x_ind = x_ind_pd.first; x_ind <= x_ind_pd.second; x_ind++) {
            double x_pos = get_xpos_in_xlim(x_ind);
            if (x_pos > x_lim_pd.second || x_pos < x_lim_pd.first) continue;
//        printf("x_ind = %d, x_pos = %.12lf\n", x_ind, x_pos);
            y_lim_pd = _p.get_y_lim_at_certain_x(x_pos);
            y_ind_pd.first = get_num_in_ylim(y_lim_pd.first, "ceil");
            y_ind_pd.second = get_num_in_ylim(y_lim_pd.second, "floor");
            for (int y_ind = y_ind_pd.first; y_ind <= y_ind_pd.second; y_ind++) {
                Point set_point = get_point_in_area(x_ind, y_ind);
                if (set_point.distance_to(_pt) > _r) continue;
                if (_update_json && get_result(x_ind, y_ind) != _res) {
                    ret.push_back({{"x", x_ind},
                                   {"y", y_ind}});
                }
                set_result(x_ind, y_ind, _res);
            }
        }
        return ret;
    }

    Point get_centroid_in_polygon(Polygon _p) {
        int total_cnt = 0, true_cnt = 0;
        double total_x_sum = 0.0, total_y_sum = 0.0, total_weight = get_res_in_polygon(_p);
        double x_sum = 0.0, y_sum = 0.0;
        pd x_lim_pd = _p.get_x_limit(1.0), y_lim_pd;
        pd x_ind_pd, y_ind_pd;
        x_ind_pd.first = get_num_in_xlim(x_lim_pd.first, "ceil");
        x_ind_pd.second = get_num_in_xlim(x_lim_pd.second, "floor");
        for (int x_ind = x_ind_pd.first; x_ind <= x_ind_pd.second; x_ind++) {
            double x_pos = get_xpos_in_xlim(x_ind);
            y_lim_pd = _p.get_y_lim_at_certain_x(x_pos);
            y_ind_pd.first = get_num_in_ylim(y_lim_pd.first, "ceil");
            y_ind_pd.second = get_num_in_ylim(y_lim_pd.second, "floor");
            for (int y_ind = y_ind_pd.first; y_ind <= y_ind_pd.second; y_ind++) {
                double y_pos = get_ypos_in_ylim(y_ind);
#ifdef GRIDWORLD_DEBUG
                printf("(%d, %d) -> (%.2lf, %.2lf) = %d\n", x_ind, y_ind, x_pos, y_pos, get_result(x_ind, y_ind));
#endif
                total_cnt++;
                total_x_sum += x_pos;
                total_y_sum += y_pos;
                if (get_result(x_ind, y_ind)) {
#ifdef GRIDWORLD_DEBUG
                    printf("true\n");
#endif
                    true_cnt++;
                    x_sum += x_pos * true_weight;
                    y_sum += y_pos * true_weight;
                } else {
#ifdef GRIDWORLD_DEBUG
                    printf("false\n");
#endif
                    x_sum += x_pos * false_weight;
                    y_sum += y_pos * false_weight;
                }
#ifdef GRIDWORLD_DEBUG
                printf("x/y sum = (%.2lf, %.2lf) x/y total sum = (%.2lf, %.2lf)\n",
                   x_sum, y_sum, total_x_sum, total_y_sum);
#endif
            }
        }
#ifdef GRIDWORLD_DEBUG
        printf("total weight = %.2lf x/y sum = (%.2lf, %.2lf)\n", total_weight, x_sum, y_sum);
#endif
        if (total_weight > 0) {
            return Point(x_sum / total_weight, y_sum / total_weight);
        } else {
            return Point(total_x_sum / total_cnt, total_y_sum / total_cnt);
        }
    }

    void output_centroid_in_polygon(Polygon _p) {
#ifdef GRIDWORLD_DEBUG
        printf("<");
    for (int x_ind = 0; x_ind < x_num; x_ind++){
        printf("-");
    }
    printf("\n");
    _p.output();
#endif
        auto vis_temp = vis;
        for (auto a: vis_temp) {
            a = false;
        }
        pd x_lim_pd = _p.get_x_limit(1.0), y_lim_pd;
        pd x_ind_pd, y_ind_pd;
        x_ind_pd.first = get_num_in_xlim(x_lim_pd.first, "ceil");
        x_ind_pd.second = get_num_in_xlim(x_lim_pd.second, "floor");
        for (int x_ind = x_ind_pd.first; x_ind <= x_ind_pd.second; x_ind++) {
            double x_pos = get_xpos_in_xlim(x_ind);
            y_lim_pd = _p.get_y_lim_at_certain_x(x_pos);
            y_ind_pd.first = get_num_in_ylim(y_lim_pd.first, "ceil");
            y_ind_pd.second = get_num_in_ylim(y_lim_pd.second, "floor");
            for (int y_ind = y_ind_pd.first; y_ind <= y_ind_pd.second; y_ind++) {
                vis_temp[get_ind(x_ind, y_ind)] = true;
            }
        }
#ifdef GRIDWORLD_DEBUG
        for (int y_ind = y_num - 1; y_ind >= 0; y_ind--){
        for (int x_ind = 0; x_ind < x_num; x_ind++){
            printf("%c", (vis_temp[get_ind(x_ind, y_ind)] == true)? 'x': '.');
        }
        printf("\n");
    }
    for (int x_ind = 0; x_ind < x_num; x_ind++){
        printf("-");
    }
    printf(">\n");
#endif
    }

};


#endif //CBF_MAIN_GRIDWORLD_HPP
