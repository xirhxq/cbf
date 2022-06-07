// Created by XIRHXQ on 12/29/21.

#include "utils.h"
#include "Polygon.h"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <functional>
//#define POLYGON_DEBUG

Polygon::Polygon() {
    n = 0;
}

Polygon::Polygon(int n_, Point *p_) {
    n = n_;
#ifdef POLYGON_DEBUG
    printf("Constructing Polygon:------\n");
#endif
    for (int i = 1; i <= n; i++){
        p[i] = p_[i - 1];
#ifdef POLYGON_DEBUG
        p[i].output();
#endif
    }
#ifdef POLYGON_DEBUG
    printf("--------\n");
#endif
    rearrange();
}

void Polygon::input() {
    scanf("%d", &n);
    for (int i = 1; i <= n; i++){
        p[i].input();
    }
    rearrange();
}

void Polygon::output() {
    printf("A Polygon with %d points:\n", n);
    for (int i = 1; i <= n;i ++){
        printf("--");
        printf("(%.4lf, %.4lf)\n", p[i].x, p[i].y);
    }
}

void Polygon::get_inner_point() {
    in = Point(0, 0);
    for (int i = 1; i <= n; i++){
        in = in + p[i];
    }
    in = in / n;
#ifdef POLYGON_DEBUG
    printf("Inner Point: ");
    in.output();
#endif
}

void Polygon::polar_sort() {
    std::sort(p + 1, p + n + 1,
              [&](Point &p_, Point &q_){return in.angle_to(p_) < in.angle_to(q_);});
#ifdef POLYGON_DEBUG
    printf("After Sorting:----\n");for (int i = 1; i <= n; i++) p[i].output();
    printf("------\n");
#endif
}

void Polygon::simplify() {
    bool del[maxp];
    memset(del, 0, sizeof(del));
    for (int i = 2; i <= n; i++){
        if (p[i] == p[i - 1]){
#ifdef POLYGON_DEBUG
            printf("Same As point %d, deleting point #%d  ", i - 1, i);
            p[i].output();
#endif
            del[i] = true;
            n--;
        }
    }
    for (int i = 1; i <= n; i++){
        Line lpre = Line(p[i], p[index_before(n, i, 1)]);
        Line lnxt = Line(p[i], p[index_after(n, i, 1)]);
        if (lpre.is_the_same_line(lnxt)){
#ifdef POLYGON_DEBUG
            printf("On the Segment of point #%d and point #%d, deleting point #%d  ",
                   index_before(n, i, 1), index_after(n, i, 1), i);
            p[i].output();
#endif
            del[i] = true;
        }
    }
    int id = 1;
    for (int i = 1; i <= n; i++){
        while (del[id]) {
            id++;
        }
        p[i] = p[id];
        id++;
    }
}

void Polygon::checking_convex() {
    for (int i = 1; i <= n; i++){
        Point vpre = p[index_before(n, i, 1)] - p[i];
        Point vnxt = p[index_after(n, i, 1)] - p[i];
        if (sgn(vpre ^ vnxt) >= 0){
            printf("Polygon not convex. %lf\n", vpre ^ vnxt);
        }
    }
}

void Polygon::rearrange() {
    get_inner_point();
    polar_sort();
    simplify();
    checking_convex();
}

Point Polygon::get_random_point() {
//    srand(time(NULL));
    int id = rand() % n + 1;
    double rd[3], rd_sum = 0.0;
    for (int i = 0; i < 3; i++){
        rd[i] = 1.0 * (rand() % 10000) / 10000.0;
        rd_sum += rd[i];
    }
    Point res = Point(0, 0);
    res = res + in * rd[0] / rd_sum;
    res = res + p[id] * rd[1] / rd_sum;
    res = res + p[index_after(n, id, 1)] * rd[2] / rd_sum;
    return res;
}

void Polygon::direct_add(const Point &p_) {
    p[++n] = p_;
#ifdef POLYGON_DEBUG
    printf("Add Point: ");p_.output();
#endif
}

void Polygon::direct_delete_at_position(const Point &p_) {
    for (int i = 1; i <= n; i++){
        if (p[i] == p_){
            n--;
            for (int j = i; j <= n; j++) p[j] = p[j + 1];
            break;
        }
    }
#ifdef POLYGON_DEBUG
    printf("Delete Point: ");p_.output();
#endif
}

void Polygon::direct_delete_at_index(int &index) {
    n--;
#ifdef POLYGON_DEBUG
    printf("Delete Point(index %d): ", index);p[index].output();
#endif
    for (int i = index; i <= n; i++){
        p[i] = p[i + 1];
    }
}

void Polygon::add_one_point(const Point &p_) {
    direct_add(p_);
    rearrange();
}

void Polygon::delete_one_point_at_position(const Point &p_) {
    direct_delete_at_position(p_);
    rearrange();
}

void Polygon::delete_one_point_at_index(int &index) {
    direct_delete_at_index(index);
    rearrange();
}

void Polygon::intersect_with_halfplane(Line &l_, int dir) {
#ifdef POLYGON_DEBUG
    printf("Intersect with line: ---------\n");
    l_.output();
    printf("----------------\n");
#endif
    Point cp[maxp], dl[maxp];
    int cp_cnt = 0, dl_cnt = 0;
    for (int i = 1; i <= n; i++){
        Line l = Line(p[i], p[index_after(n, i, 1)]);
        if ((!l.is_the_same_line(l_)) && l_.line_cross_seg(l)){
            cp[++cp_cnt] = l_.cross_point(l);
        }
    }
    for (int i = 1; i <= n; i++){
        if (l_.relation(p[i]) == -dir){
            dl[++dl_cnt] = p[i];
        }
    }
    for (int i = 1; i <= cp_cnt; i++) direct_add(cp[i]);
    for (int i = 1; i <= dl_cnt; i++) direct_delete_at_position(dl[i]);
    rearrange();
}

void Polygon::intersect_with_halfplane(Line &l_, Point &p_) {
    intersect_with_halfplane(l_, l_.relation(p_));
}

void Polygon::intersect_with_polygon(Polygon &p_) {
    for (int i = 1; i <= p_.n; i++){
        Line l = Line(p_.p[i], p_.p[index_after(p_.n, i, 1)]);
        intersect_with_halfplane(l, in);
    }
}

double Polygon::circumference() {
    double res = 0.0;
    for (int i = 1; i <= n; i++){
        res = res + Line(p[i], p[index_after(n, i, 1)]).len();
    }
    return res;
}

double Polygon::area() {
    double res = 0.0;
    for (int i = 1; i <= n; i++){
        res = res + fabs((p[i] - in) ^ (p[index_after(n, i ,1)] - in)) / 2.0;
    }
    return res;
}

double Polygon::area_with_function(const std::function<double(Point)>& f, double spacing = 0.1) {
    double res = 0.0;
    rearrange();
    Point vnow, vnxt, tmp_point[3], center;
    for (int q = 1; q <= n; q++){
        vnow = p[q] - in;
        vnxt = p[index_after(n, q, 1)] - in;
        int k = int(fmax(vnow.len(), vnxt.len())) + 1;
        for (int i = 0; i < k; i++){
            for (int j = 0; j < k - i; j++){
                tmp_point[0] = in + 1.0 * i / k * vnow + 1.0 * j / k * vnxt;
                tmp_point[1] = in + 1.0 * (i + 1) / k * vnow + 1.0 * j / k * vnxt;
                tmp_point[2] = in + 1.0 * i / k * vnow + 1.0 * (j + 1) / k * vnxt;
                center = (tmp_point[0] + tmp_point[1] + tmp_point[2]) / 3.0;
                res = res + fabs((tmp_point[0] - tmp_point[1]) ^ (tmp_point[1] - tmp_point[2]) / 2.0) * f(center);
            }
        }
        for (int i = 0; i < k - 1; i++){
            for (int j = 0; j < k - i - 1; j++){
                tmp_point[0] = in + 1.0 * (i + 1) / k * vnow + 1.0 * (j + 1) / k * vnxt;
                tmp_point[1] = in + 1.0 * (i + 1) / k * vnow + 1.0 * j / k * vnxt;
                tmp_point[2] = in + 1.0 * i / k * vnow + 1.0 * (j + 1) / k * vnxt;
                center = (tmp_point[0] + tmp_point[1] + tmp_point[2]) / 3.0;
                res = res + fabs((tmp_point[0] - tmp_point[1]) ^ (tmp_point[1] - tmp_point[2]) / 2.0) * f(center);
            }
        }
    }
    return res;
}

void Polygon::get_x_limit(double *x) {
    x[0] = x[1] = p[1].x;
    for (int i = 2; i <= n; i++){
        x[0] = std::fmin(x[0], p[i].x);
        x[1] = std::fmax(x[1], p[i].x);
    }
    double mid = (x[0] + x[1]) / 2.0;
    x[0] = mid + (x[0] - mid) * 1.2;
    x[1] = mid + (x[1] - mid) * 1.2;
}

void Polygon::get_y_limit(double *y) {
    y[0] = y[1] = p[1].y;
    for (int i = 2; i <= n; i++){
        y[0] = std::fmin(y[0], p[i].y);
        y[1] = std::fmax(y[1], p[i].y);
    }
    double mid = (y[0] + y[1]) / 2.0;
    y[0] = mid + (y[0] - mid) * 1.2;
    y[1] = mid + (y[1] - mid) * 1.2;
}