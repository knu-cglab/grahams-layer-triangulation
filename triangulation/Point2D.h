/*
 * Point2D.h
 *
 *  Created on: Apr 21, 2015
 *      Author: dkotsur
 */

#ifndef POINT2D_H_
#define POINT2D_H_

#include "Defs.h"
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>

#define POINT_EPSILON 1.0e-8

class Point2D {

	struct Point2D_XY_Compare {
		bool operator()(const Point2D &p1, const Point2D &p2) {
			return (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y));
		}
	};

    struct Point2D_YX_Compare {
        bool operator()(const Point2D &p1, const Point2D &p2) {
            return (p1.y < p2.y || (p1.y == p2.y && p1.x < p2.x));
        }
    };

public:
    real x, y;

    const static real Inf;
	static Point2D_XY_Compare xy_compare;
    static Point2D_YX_Compare yx_compare;

    Point2D(real x = 0.0, real y = 0.0);
    Point2D(const Point2D &point);
    Point2D(Point2D&&) = default;
    Point2D(std::initializer_list<real> &init);

    friend real dotProduct(const Point2D &p1, const Point2D &p2);
    friend real crossProduct(const Point2D &p1, const Point2D &p2);

    friend Point2D operator+(const Point2D &p1, const Point2D &p2);
    friend Point2D operator-(const Point2D &p1, const Point2D &p2);
    friend Point2D operator/(const Point2D &p1, const Point2D &p2);
    friend Point2D operator*(const Point2D &p, real value);
    friend Point2D operator*(real value, const Point2D &p);
    friend Point2D operator/(const Point2D &p, real value);
	friend Point2D operator-(const Point2D &p);

    friend bool operator<(const Point2D &p1, const Point2D &p2);

    friend std::ostream &operator<<(std::ostream &stream, const Point2D &p);
    friend std::vector<Point2D> &operator<<(std::vector<Point2D> &v, const Point2D &p);

    Point2D &operator-=(const Point2D &p);
    Point2D &operator+=(const Point2D &p);
    Point2D &operator*=(real value);
    Point2D &operator/=(real value);
    Point2D &operator=(const Point2D &p);

    Point2D normalized();
    void normalize();
    real norm();
    real norm2();

    Point2D getRotated90CW();
    Point2D getRotated90CCW();

	static bool isLeftTurn(const Point2D &p1, const Point2D &p2, const Point2D &p3);
	static bool isRightTurn(const Point2D &p1, const Point2D &p2, const Point2D &p3);

    real operator[](int i);

    void setX(real x);
    void setY(real y);

    bool isVertical();
    bool isHorizontal();
    bool isValid();

};

bool equal(const Point2D &p1, const Point2D &p2, real EPSILON = POINT_EPSILON);
bool equal(real v1, real v2, real EPSILON = POINT_EPSILON);

real dotProduct(const Point2D &p1, const Point2D &p2);
real crossProduct(const Point2D &p1, const Point2D &p2);

real distance(const Point2D &p1, const Point2D &p2);

#endif /* POINT2D_H_ */
