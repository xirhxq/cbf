#ifndef CBF_MAIN_LINE_HPP
#define CBF_MAIN_LINE_HPP

#include "Point.hpp"

class Line {
    Point s;
    Point e;
public:
    Line() {
        s = Point(0, 0);
        e = Point(1, 0);
        adjust();
    }

    Line(const Point &_s, const Point &_e) {
        s = _s;
        e = _e;
        adjust();
    }

    Line(const Point &_s, double a) {
        s = _s;
        if (sgn(a - pi / 2) == 0){
            e = s + Point(0, 1);
        }
        else{
            e = s + Point(1, tan(a));
        }
    }

    Line(double _a, double _b, double _c) {
        if (sgn(_a) == 0){
            s = Point(0, -_c / _b);
            e = Point(1, -_c / _b);
        }
        else if (sgn(_b) == 0){
            s = Point(-_c / _a, 0);
            e = Point(-_c / _a, 1);
        }
        else{
            s = Point(0, -_c / _b);
            e = Point(1, -(_c + _a) / _b);
        }
        adjust();
    }

    void input(){
        s.input();
        e.input();
    }

    void output() const {
        printf("A Line from (%.4lf, %.4lf) to (%.4lf, %.4lf)\n", s.x, s.y, e.x, e.y);
    }

    void adjust() {
        if (s > e){
            Point tmp_point = s;
            s = e;
            e = tmp_point;
        }
    }

    double len() {
        return s.distance_to(e);
    }

    double len2() {
        return (s - e).len2();
    }

    double angle() {
        return (e - s).angle();
    }

    int relation(const Point &p) {
        int c = sgn((p - s) ^ (p - e));
        if (c < 0) return -1;
        else if (c > 0) return 1;
        else return 0;
    }

    int relation_to_seg(const Point &p) {
        return sgn((p - s) ^ (e - s)) == 0 && sgn((p - s) * (p - e)) <= 0;
    }

    bool is_on_line(const Point &p) {
        return relation(p) == 0;
    }

    bool is_on_seg(const Point &p) {
        return relation_to_seg(p) == 0;
    }

    bool is_parallel(const Line &l) {
        return sgn((e - s) ^ (l.e - l.s)) == 0;
    }

    bool is_the_same_line(const Line &l) {
        return is_on_line(l.s) && is_on_line(l.e);
    }

    bool line_cross_line(const Line &l) {
        return !is_parallel(l);
    }

    Point cross_point(const Line &l) {
        double a1 = (l.e - l.s) ^ (s - l.s);
        double a2 = (l.e - l.s) ^ (e - l.s);
        return Point((s.x * a2 - e.x * a1) / (a2 - a1),
                     (s.y * a2 - e.y * a1) / (a2 - a1));
    }

    bool line_cross_seg(const Line &l) {
        return (!is_parallel(l)) && (relation(l.s) * relation(l.e) < 0);
    }

    bool seg_cross_seg(Line &l) {
        if (!is_parallel(l)) return false;
        Point cp = cross_point(l);
        return is_on_seg(cp) && l.is_on_seg(cp);
    }

    double dis_line_to_point(const Point &p) {
        return fabs((p - s) ^ (e - s)) / len();
    }

    double dis_seg_to_point(const Point &p) {
        if (sgn((p - s) * (e - s)) < 0 || sgn((p - e) * (s - e) < 0)){
            return fmin(s.distance_to(p), e.distance_to(p));
        }
        return dis_line_to_point(p);
    }

    double dis_seg_to_seg(Line &l) {
        return fmin(fmin(dis_seg_to_point(l.s), dis_seg_to_point(l.e)),
                    fmin(l.dis_seg_to_point(s), l.dis_seg_to_point(e)));
    }

    Point project_point(const Point &p) {
        return s + (((e - s) * ((e - s) * (p - s))) / (e - s).len2() );
    }

    Point symmetry_point(const Point &p) {
        Point q = project_point(p);
        return q * 2.0 - p;
    }

};

Line median_line(Point &p, Point &q){
    return Line((p + q) / 2.0, p.angle_to(q) + pi / 2);
}
#endif //CBF_MAIN_LINE_HPP
