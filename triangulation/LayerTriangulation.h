#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <vector>
#include <algorithm>

#include "Point2D.h"

class LayerTriangulation
{
public:
    LayerTriangulation(const std::vector<Point2D> &points);

    bool saveLaTeX(const std::string &filename, std::vector<Point2D> &points);

private:
    int  selectOrigin(const std::vector<Point2D> &points);

    // Simple triangulation
    void triangulate0(int layer0, int layer1, const std::vector<Point2D> &points);

    // Maximized minimal angle
    void triangulate1(int layer0, int layer1, const std::vector<Point2D> &points);

    void traingluateLastLayer(const std::vector<Point2D> &points);

    // Find outer convex polygon (0-level)
    void grahamScan0(const std::vector<int> &indices, const std::vector<Point2D> &points,
                    std::vector<int> &inner);

    // Find inner convex polygon (k-level, k > 0)
    void grahamScan1(const std::vector<int> &indices, const std::vector<Point2D> &points,
                     std::vector<int> &inner);

    void findLowestPoints(int layer_i, const std::vector<Point2D> &points);

public:

    std::vector<std::vector<int> > layers;
    std::vector<int> lowest;

    std::vector<std::pair<int,int> > edges;

};

#endif // TRIANGULATION_H
