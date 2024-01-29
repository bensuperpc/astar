#include <astar/astar.hpp>
#include <iostream>

auto main() -> int {
    // Create the template class with optional a type (e.g. uint32_t) and a boolean
    // if you want enable debug mode (AStar::AStar<uint32_t, true>)
    AStar::AStar pathFinder;

    // Define the map size (width, height)
    pathFinder.setWorldSize({10, 10});

    // Set the heuristic function (manhattan, euclidean, octagonal etc...), it is optional, default is euclidean
    pathFinder.setHeuristic(AStar::Heuristic::manhattan);

    // if you want to enable diagonal movement, it is optional, default is false
    pathFinder.setDiagonalMovement(true);

    // Add a obstacle point (5, 5) and (5, 6)
    pathFinder.addObstacle({5, 5});
    pathFinder.addObstacle({5, 6});

    // Find the path from (0, 0) to (9, 9), it it equal to 0, then the path is not found
    auto path = pathFinder.findPath({0, 0}, {9, 9});

    // Print the path
    for (auto& p : path) {
        std::cout << p.x << " " << p.y << std::endl;
    }

    return 0;
}
