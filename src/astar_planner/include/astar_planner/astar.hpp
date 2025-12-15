#pragma once

#include <vector>
#include <utility>

class AStar
{
public:
    using Grid  = std::vector<std::vector<int>>;
    using Point = std::pair<int, int>;

    // A* search
    std::vector<Point> search(
        const Point& start,
        const Point& goal,
        const Grid& grid);

    // Obstacle inflation (robot size handling)
    static void inflateObstacles(
        Grid& grid,
        int inflation_radius_cells);
};

