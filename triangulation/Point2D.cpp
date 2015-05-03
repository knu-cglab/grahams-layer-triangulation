//
//  Point2D.cpp
//
//  Created by Kotsur on 12.03.14.
//  Copyright (c) 2014 Dmytro Kotsur. All rights reserved.
//

#include "Point2D.h"
#include <cmath>

const real Point2D::Inf = std::numeric_limits<real>::infinity();

Point2D::Point2D_XY_Compare Point2D::xy_compare = Point2D::Point2D_XY_Compare();
Point2D::Point2D_YX_Compare Point2D::yx_compare = Point2D::Point2D_YX_Compare();


Point2D::Point2D(real _x, real _y) : x(_x), y(_y) {
}

Point2D::Point2D(const Point2D &point) : x(point.x), y(point.y) {
}

Point2D::Point2D(std::initializer_list<real> &init)
{
    if (init.size() == 1) {
        x = *init.begin();
    } else if (init.size() > 1) {
        auto it = init.begin();
        x = *it++;
        y = *it;
    }
}

real dotProduct(const Point2D &p1, const Point2D &p2) {
    return p1.x * p2.x + p1.y * p2.y;
}

real crossProduct(const Point2D &p1, const Point2D &p2) {
    return p1.x * p2.y - p1.y * p2.x;
}

bool operator<(const Point2D &p1, const Point2D &p2) {
    return Point2D::xy_compare(p1, p2);
}

Point2D operator+(const Point2D &p1, const Point2D &p2) {
    return Point2D(p1.x + p2.x, p1.y + p2.y);
}

Point2D operator-(const Point2D &p1, const Point2D &p2) {
    return Point2D(p1.x - p2.x, p1.y - p2.y);
}

Point2D operator/(const Point2D &p1, const Point2D &p2) {
    return Point2D(p1.x / p2.x, p1.y / p2.y);
}

Point2D operator*(const Point2D &p, real value) {
    return Point2D(p.x * value, p.y * value);
}

Point2D operator*(real value, const Point2D &p) {
    return Point2D(p.x * value, p.y * value);
}

Point2D operator/(const Point2D &p, real value) {
    return Point2D(p.x / value, p.y / value);
}

Point2D operator-(const Point2D &p) {
	return Point2D(-p.x, -p.y);
}

std::ostream &operator<<(std::ostream &stream, const Point2D &p) {
    stream << "(" << p.x << "," << p.y << ")";
    return stream;
}

std::vector<Point2D> &operator<<(std::vector<Point2D> &v, const Point2D &p) {
    v.push_back(p);
    return v;
}

Point2D &Point2D::operator-=(const Point2D &p) {
    x -= p.x;
    y -= p.y;
    return *this;
}

Point2D &Point2D::operator+=(const Point2D &p) {
    x += p.x;
    y += p.y;
    return *this;
}

Point2D &Point2D::operator*=(real value) {
    x *= value;
    y *= value;
    return *this;
}

Point2D &Point2D::operator/=(real value) {
    x /= value;
    y /= value;
    return *this;
}

Point2D &Point2D::operator=(const Point2D &p) {
    this->x = p.x;
    this->y = p.y;
    return *this;
}

real Point2D::operator[](int i) {
    if (i==0) return x;
    else return y;
}

void Point2D::setX(real x) {
    this->x = x;
}

void Point2D::setY(real y) {
    this->y = y;
}

bool Point2D::isVertical() {
    return (y == Inf && !isnan(double(x)) && x != Inf);
}

bool Point2D::isHorizontal() {
    return (x == Inf && !isnan(double(y)) && y != Inf);
}

bool Point2D::isValid() {
    if (x == Inf && y == Inf)
        return false;
    return (!isnan(x) && !isnan(y));
}

Point2D Point2D::normalized() {
    return (*this) / this->norm();
}

void Point2D::normalize() {
    real n = norm();
    x /= n;
    y /= n;
}

real Point2D::norm() {
    return sqrt(x * x + y * y);
}

real Point2D::norm2() {
    return x *x + y * y;
}

Point2D Point2D::getRotated90CW() {
    return Point2D(y, -x);
}

Point2D Point2D::getRotated90CCW() {
    return Point2D(-y, x);
}

bool Point2D::isLeftTurn(const Point2D &p1, const Point2D &p2,
						 const Point2D &p3) {
	return (crossProduct(p2 - p1, p3 - p2) > 0.0);
}

bool Point2D::isRightTurn(const Point2D &p1, const Point2D &p2,
						  const Point2D &p3) {
	return (crossProduct(p2 - p1, p3 - p2) < 0.0);
}

bool equal(const Point2D &p1, const Point2D &p2, real EPSILON) {
    return (fabs(p1.x - p2.x) < EPSILON && fabs(p1.y - p2.y) < EPSILON);
}

bool equal(real v1, real v2, real EPSILON) {
    return fabs(v1 - v2) < EPSILON;
}


real distance(const Point2D &p1, const Point2D &p2)
{
    return (p1 - p2).norm();
}
