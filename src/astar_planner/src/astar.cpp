#include "astar_planner/astar.hpp"

#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>

struct Node
{
    int x, y;
    double g, h;
};

static double heuristic(int x1, int y1, int x2, int y2)
{
    return std::hypot(x1 - x2, y1 - y2);
}

std::vector<AStar::Point> AStar::search(
    const Point& start,
    const Point& goal,
    const Grid& grid)
{
    const int H = grid.size();
    const int W = grid[0].size();

    auto key = [W](int x, int y) { return y * W + x; };

    auto cmp = [](const Node& a, const Node& b)
    {
        return (a.g + a.h) > (b.g + b.h);
    };

    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);
    std::unordered_map<int, double> g_cost;
    std::unordered_map<int, Point> parent;

    open.push({start.first, start.second, 0.0,
               heuristic(start.first, start.second,
                         goal.first, goal.second)});
    g_cost[key(start.first, start.second)] = 0.0;

    const int dx[8] = {1,-1,0,0, 1,1,-1,-1};
    const int dy[8] = {0,0,1,-1, 1,-1,1,-1};

    while (!open.empty())
    {
        Node cur = open.top();
        open.pop();

        if (cur.x == goal.first && cur.y == goal.second)
            break;

        for (int i = 0; i < 8; i++)
        {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];

            if (nx < 0 || ny < 0 || nx >= W || ny >= H)
                continue;
            if (grid[ny][nx] == 1)
                continue;

            double ng = cur.g + std::hypot(dx[i], dy[i]);
            int nk = key(nx, ny);

            if (!g_cost.count(nk) || ng < g_cost[nk])
            {
                g_cost[nk] = ng;
                parent[nk] = {cur.x, cur.y};
                open.push({nx, ny, ng,
                           heuristic(nx, ny,
                                     goal.first, goal.second)});
            }
        }
    }

    // Path reconstruction
    std::vector<Point> path;
    Point cur = goal;

    while (cur != start)
    {
        path.push_back(cur);
        int k = key(cur.first, cur.second);
        if (!parent.count(k))
            break;
        cur = parent[k];
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

/* =====================================================
   Obstacle Inflation
   - robot radius를 grid cell 단위로 반영
   ===================================================== */
void AStar::inflateObstacles(
    Grid& grid,
    int r)
{
    const int H = grid.size();
    const int W = grid[0].size();

    Grid inflated = grid;

    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            if (grid[y][x] != 1)
                continue;

            for (int dy = -r; dy <= r; dy++)
            {
                for (int dx = -r; dx <= r; dx++)
                {
                    int ny = y + dy;
                    int nx = x + dx;

                    if (nx < 0 || ny < 0 || nx >= W || ny >= H)
                        continue;

                    // 원형 inflation (중요)
                    if (dx*dx + dy*dy <= r*r)
                        inflated[ny][nx] = 1;
                }
            }
        }
    }
    grid = inflated;
}

