#include <astar/astar.hpp>
#include <iostream>

auto main() -> int {
    // Create the template class with optional a type (e.g. uint32_t) and a boolean
    // if you want enable debug mode (AStar::AStar<uint32_t, true>)
    AStar::AStarFast pathFinder;

    // Set the heuristic function (manhattan, euclidean, octagonal etc...), it is optional, default is euclidean
    pathFinder.setHeuristic(AStar::Heuristic::manhattan);

    // if you want to enable diagonal movement, it is optional, default is false
    pathFinder.setDiagonalMovement(true);

    // Create world 9x9 filled with 0
    std::vector<uint32_t> world(9 * 9, 0);

    // set lambda function to check if is an obstacle (value == 1)
    auto isObstacle = [](uint32_t value) -> bool { return value == 1; };
    pathFinder.setObstacle(isObstacle);

    // Add a obstacle point (5, 5) and (5, 6)
    world[5 + 5 * 9] = 1;
    world[5 + 6 * 9] = 1;

    // Find the path from (0, 0) to (9, 9), it it equal to 0, then the path is not found
    // This version of findPath() is faster due direct access to the world
    auto path = pathFinder.findPath({0, 0}, {9, 9}, world, {9, 9});

    // Print the path
    for (auto& p : path) {
        std::cout << p.x << " " << p.y << std::endl;
    }

    return 0;
}
