#include <iostream>
#include <fstream>
#include <vector>

#include <random>
#include <iterator>

using namespace std;

#include "Timer.h"

#include "Point2D.h"
#include "LayerTriangulation.h"

std::vector<Point2D> setup_data() {
    std::vector<Point2D> data = {
        {0.0, 0.0},
        {0.0, 25.0},
        {10.0, 0.0},
        {15.0, 10.0},
        {6.0, 6.0},
        {5.0, 10.0},
        {7.0, 12.0},
        {10.0, 30.0}
    };
    return data;
}


int main()
{

    std::vector<Point2D> data = setup_data();

    Timer timer;
    LayerTriangulation triangulation(data);
    std::cout << timer.elapsed() * 1000 << " ms" << std::endl;

    triangulation.saveLaTeX(“latex_output.tex”, data);

    return 0;
}

