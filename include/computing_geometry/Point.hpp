// Created by XIRHXQ on 12/29/21.

#ifndef CBF_MAIN_POINT_HPP
#define CBF_MAIN_POINT_HPP

#include "../utils.h"

class Point{

public:
    double x, y;

public:
    Point(){
    }

    Point(double _x, double _y){
        x = _x;
        y = _y;
    }

    Point(const Point &p){
        x = p.x;
        y = p.y;
    }

    Point(const Eigen::VectorXd& _v) {
        x = _v(0);
        y = _v(1);
    }

    Eigen::VectorXd vec() {
        Eigen::VectorXd res{2};
        res << x, y;
        return res;
    }

    void input(){
        scanf("%lf%lf", &x, &y);
    }

    void output() const{
        printf("A Point @ (%.4lf, %.4lf)\n", x, y);
    }

    bool operator == (const Point &b)const{
        return sgn(x - b.x) == 0 && sgn(y - b.y) == 0;
    }

    bool operator < (const Point &b)const{
        return sgn(x - b.x) == 0 ? sgn(y - b.y) < 0 : x < b.x;
    }

    bool operator > (const Point &b)const{
        return sgn(x - b.x) == 0 ? sgn(y - b.y) > 0 : x > b.x;
    }

    Point operator + (const Point &b)const{
        return Point(x + b.x, y + b.y);
    }

    Point operator - (const Point &b)const{
        return Point(x - b.x, y - b.y);
    }

    double operator ^ (const Point &b)const{
        return x * b.y - y * b.x;
    }

    double operator * (const Point &b)const{
        return x * b.x + y * b.y;
    }

    Point operator * (const double b)const{
        return Point(x * b, y * b);
    }

    Point operator / (const double b)const{
        return Point(x / b, y / b);
    }

    Point operator - ()const{
        return Point(-x, -y);
    }

    double len() const{
        return sqrt(x * x + y * y);
    }

    double len2() const{
        return x * x + y * y;
    }

    double angle() const{
        return atan2(y, x);
    }

    double angle_to(const Point &b) const{
        return (b - *this).angle();
    }

    double distance_to(const Point &b) const{
        return (b - *this).len();
    }

    Point normalize() const{
        return *this / this->len();
    }

    Point transform(const double &b) const{
        return *this / this->len() * b;
    }

    Point saturation(const double &b){
        if (this->len() > b){
            return this->transform(b);
        }
        else return *this;
    }

    Point rotate(double &b) const{
        double c = cos(this->angle()), s = sin(this->angle());
        return Point(x * c - y * s, x * s + y * c);
    }

    Point rotate_around(const Point &p, double a) const{
        Point v = *this - p;
        double c = cos(a), s = sin(a);
        return Point(p.x + v.x * c - v.y * s, p.y + v.x * s + v.y * c);
    }

};

Point operator * (double a, const Point &p){
    return Point(p.x * a, p.y * a);
}

#endif //CBF_MAIN_POINT_HPP