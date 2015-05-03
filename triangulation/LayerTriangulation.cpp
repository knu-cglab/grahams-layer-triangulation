#include "LayerTriangulation.h"

#include <stack>
#include <deque>
#include <fstream>

inline bool is_collinear(const Point2D &p1, const Point2D &p2, const Point2D &p3) {
    return equal(crossProduct((p1 - p2), (p3 - p2)), 0.0);
}

inline real get_side(const Point2D &point1, const Point2D &point2, const Point2D &point3)
{
    return crossProduct(point2 - point1, point3 - point1);
}

inline bool is_ccw(const Point2D &origin, const Point2D &point1, const Point2D &point2) {
    return get_side(origin, point1, point2) >= 0;
}

inline real calc_angle(const Point2D &origin, const Point2D &point1, const Point2D &point2) {
    Point2D d1 = point1 - origin, d2 = point2 - origin;
    return acos(dotProduct(d1, d2) / (d1.norm() * d2.norm()));
}

inline real min_angle(const Point2D &point1, const Point2D &point2, const Point2D &point3) {
    real angle0 = calc_angle(point1, point2, point3), angle1 = calc_angle(point2, point3, point1), angle2 = calc_angle(point3, point1, point2);
    return std::min(angle0, std::min(angle1, angle2));
}

void print(const std::vector<int> &indices, const std::vector<Point2D> &points) {
    std::for_each(indices.begin(), indices.end(), [&points](int i) { std::cout << "--" << points[i] << " "; });
    std::cout << std::endl;
}

LayerTriangulation::LayerTriangulation(const std::vector<Point2D> &points)
{

    std::vector<int> indices(points.size(), 0);
    // Set up indices
    for (int index = 0; index < indices.size(); ++index) {
        indices[index] = index;
    }

    int origin_i = selectOrigin(points);
    std::swap(indices[0], indices[origin_i]);

    // Select the origin
    Point2D origin = points[indices[0]];

    // Calculate points in translated coordinate system
    std::vector<Point2D> translated(points.size(), Point2D());
    std::transform(points.begin(), points.end(), translated.begin(), [&origin](const Point2D &point) {
        return point - origin;
    });

    // Sort points counterclockwise
    std::sort(indices.begin() + 1, indices.end(), [&translated](int i1, int i2) {
        double angle0 = atan2(translated[i1].y, translated[i1].x), angle1 = atan2(translated[i2].y, translated[i2].x);
        return angle0 < angle1 || (equal(angle0, angle1) && translated[i1].norm() < translated[i2].norm());
    });

    // Extract layers
    std::vector<int> inner;
    do {
        inner.clear(); // Clear inner indices vector
        // Perform Graham scan

        if (layers.size() == 0)
            grahamScan0(indices, points, inner);
        else
            grahamScan1(indices, points, inner);

        // Debug print
        //print(layers.back(), points);

        indices.resize(inner.size());
        std::copy(inner.begin(), inner.end(), indices.begin());

    } while (indices.size() > 1);

    // Find the lowest point for each layer
    for (int layer_i = 0; layer_i < layers.size(); ++layer_i) {
        findLowestPoints(layer_i, points);
    }

    // Perform triangulation
    for (int layer_i = 1; layer_i < layers.size(); ++layer_i) {
        triangulate1(layer_i - 1, layer_i, points);
    }

    // Triangulate the last layer if it is possible
    traingluateLastLayer(points);

}

int LayerTriangulation::selectOrigin(const std::vector<Point2D> &points)
{
    int index = 0;
    for (int i = 1; i < points.size(); ++i) {
        if (Point2D::yx_compare(points[i], points[index])) {
            index = i;
        }
    }
    return index;
}

void LayerTriangulation::grahamScan0(const std::vector<int> &indices, const std::vector<Point2D> &points, std::vector<int> &inner)
{
    if (indices.size() == 0)
        return;

    layers.push_back(std::vector<int>());
    std::vector<int> &layer = layers.back();

    if (indices.size() < 3) {
        for (int index : indices)
            layer.push_back(index);
    } else if (indices.size() == 3) {
        layer.push_back(indices[0]);
        layer.push_back(indices[1]);
        layer.push_back(indices[2]);
    } else {

        std::vector<bool> mask(indices.size(), false);
        layer.reserve(indices.size());

        layer.push_back(0);
        layer.push_back(1);
        layer.push_back(2);

        for (size_t index = 3; index < indices.size(); ++index) {
            int prev_index = layer.back(); layer.pop_back();
            while (!is_ccw(points[indices[layer.back()]], points[indices[prev_index]], points[indices[index]])) {
                mask[prev_index] = true;
                prev_index = layer.back();
                layer.pop_back();
            }
            layer.push_back(prev_index);
            layer.push_back(index);
        }

        for (size_t index = 0; index < layer.size(); ++index) {
            layer[index] = indices[layer[index]];
        }

        inner.push_back(indices[0]);
        for (size_t index = 0; index < mask.size();  ++index) {
            if (mask[index])
                inner.push_back(indices[index]);
        }
    }
}

void LayerTriangulation::grahamScan1(const std::vector<int> &indices, const std::vector<Point2D> &points, std::vector<int> &inner)
{
    if (indices.size() <= 1)
        return;

    layers.push_back(std::vector<int>());
    std::vector<int> &layer = layers.back();

    if (indices.size() < 4) {
        for (size_t index = 1; index < indices.size(); ++index) {
            layer.push_back(indices[index]);
        }
    } else if (indices.size() == 4) {
        if (is_ccw(points[indices[1]], points[indices[2]], points[indices[3]])) {
            layer.push_back(indices[1]);
            layer.push_back(indices[2]);
            layer.push_back(indices[3]);
        } else {
            layer.push_back(indices[1]);
            layer.push_back(indices[3]);
            layer.push_back(indices[2]);
        }
    } else {

        std::deque<int> hull;
        std::vector<bool> mask(indices.size(), false);

        hull.push_back(0);
        hull.push_back(1);
        hull.push_back(2);

        for (size_t index = 3; index < indices.size(); ++index) {
            int prev_index = hull.back(); hull.pop_back();
            while (!is_ccw(points[indices[hull.back()]], points[indices[prev_index]], points[indices[index]])) {
                mask[prev_index] = true;
                prev_index = hull.back();
                hull.pop_back();
            }
            hull.push_back(prev_index);
            hull.push_back(index);
        }

        mask[1] = true;
        hull.pop_front();

        for (int index = (int)indices.size() - 1; index > 0; --index) {
            if (mask[index]) {
                int prev_index = hull.back(); hull.pop_back();
                while (!is_ccw(points[indices[hull.back()]], points[indices[prev_index]], points[indices[index]])) {
                    mask[prev_index] = true;
                    prev_index = hull.back();
                    hull.pop_back();
                }

                mask[prev_index] = false;
                mask[index] = false;

                hull.push_back(prev_index);
                hull.push_back(index);
            }
        }

        mask[1] = false;
        hull.pop_back();

        for (auto it = hull.begin(); it != hull.end(); ++it) {
            layer.push_back(indices[*it]);
        }

        inner.push_back(indices[0]);
        for (size_t index = 0; index < mask.size(); ++index) {
            if (mask[index])
                inner.push_back(indices[index]);
        }
    }
}

void LayerTriangulation::findLowestPoints(int layer_i, const std::vector<Point2D> &points)
{
    int lowest_i = 0;

    for (size_t i = 1; i < layers[layer_i].size(); ++i) {
        if (Point2D::yx_compare(points[layers[layer_i][i]], points[layers[layer_i][lowest_i]])) {
            lowest_i = i;
        }
    }
    lowest.push_back(lowest_i);

}

bool LayerTriangulation::saveLaTeX(const std::string &filename, std::vector<Point2D> &points)
{
    std::ofstream out(filename);
    if (!out)
        return false;

    out << "\\documentclass[border=10pt]{standalone}\n"
           "\\usepackage{tikz}\n\n"
            "\\begin{document}"
            "\\begin{tikzpicture}";

    out << " % Triangulated edges\n";
    for (auto edge : edges) {
        out << "\t\\draw " << points[edge.first] << " --" << points[edge.second] << ";\n";
    }

    out << " % Hull layers\n";
    for (size_t layer_i = 0; layer_i < layers.size(); ++layer_i) {
        out << "\t\\draw[red] " << points[layers[layer_i][0]];
        size_t layer_size = layers[layer_i].size();
        for (size_t point_i = 0; point_i < layer_size; ++point_i) {
            out << " --" << points[layers[layer_i][(point_i + 1) % layer_size]];
        }
        out << ";\n";
    }

    for (size_t point_i = 0; point_i < points.size(); ++point_i) {
        out << "\t\\fill " << points[point_i] << " circle[radius=2pt] node [black,above=4] { };\n";
    }

    out << "\\end{tikzpicture}\n"
           "\\end{document}";

    std::flush(out);

    return true;
}

void LayerTriangulation::triangulate0(int layer0, int layer1, const std::vector<Point2D> &points)
{
    int point0 = lowest[layer0];
    int point1 = lowest[layer1];

    std::vector<int> &idx0 = layers[layer0];
    std::vector<int> &idx1 = layers[layer1];

    do {
        edges.push_back(std::make_pair(idx0[point0], idx1[point1]));

        //std::cout << points[idx0[point0]] << " <-> " << points[idx1[point1]] << std::endl;
        //std::cout << idx0[point0] << " <-> " << idx1[point1] << std::endl;

        if (!is_ccw(points[idx0[point0]], points[idx1[point1]], points[idx1[(point1 + 1) % idx1.size()]])) {
            point1 = (point1 + 1) % idx1.size();
        } else {
            point0 = (point0 + 1) % idx0.size();
        }
    } while (point0 != lowest[layer0] || point1 != lowest[layer1]);

}

void LayerTriangulation::triangulate1(int layer0, int layer1, const std::vector<Point2D> &points)
{

    std::vector<int> &idx0 = layers[layer0];
    std::vector<int> &idx1 = layers[layer1];

    int point0 = lowest[layer0], point1 = 0;

    real min_dist = distance(points[idx0[point0]], points[idx1[point1]]);
    for (int index = 1; index < idx1.size(); ++index) {
        real dist = distance(points[idx0[point0]], points[idx1[index]]);
        if (dist < min_dist) {
            point1 = index;
            min_dist = dist;
        }
    }

    int end0 = point0, end1 = point1;
    do {

        edges.push_back(std::make_pair(idx0[point0], idx1[point1]));

        int next0 = (point0 + 1) % idx0.size(), next1 = (point1 + 1) % idx1.size();
        if (!is_ccw(points[idx0[point0]], points[idx1[point1]], points[idx1[next1]])) {
            // Check if we can build next triangle
            if (!is_ccw(points[idx0[next0]], points[idx1[point1]], points[idx1[next1]])) {
                // Check triangle with minimum angle
                real angle0 = std::min(min_angle(points[idx0[point0]], points[idx1[point1]], points[idx1[next1]]),
                              min_angle(points[idx0[point0]], points[idx1[next1]], points[idx0[next0]]));
                real angle1 = std::min(min_angle(points[idx0[point0]], points[idx0[next0]], points[idx1[point1]]),
                              min_angle(points[idx0[next0]], points[idx1[next1]], points[idx1[point1]]));
                if (angle0 > angle1) {
                    point1 = next1;
                } else {
                    point0 = next0;
                }
            } else {
                point1 = next1;
            }
        } else {
            point0 = next0;
        }
    } while (point0 != end0 || point1 != end1);

}

void LayerTriangulation::traingluateLastLayer(const std::vector<Point2D> &points)
{
    std::vector<int> &idx = layers.back();

    if (idx.size() > 3) {
        for (size_t index = 1; index < idx.size(); ++index) {
            edges.push_back(std::make_pair(idx[0], idx[index]));
        }
    }
}
