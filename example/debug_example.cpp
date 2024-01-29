#include <iostream>

#include <astar/astar.hpp>

auto main() -> int {
    // Enable debug mode with template argument, this helps avoid performance issues on non-debug classes
    AStar::AStar<uint32_t, true> pathFinder;

    // Set lambda function to debug current node
    std::function<void(const AStar::Node<uint32_t>* node)> debugCurrentNode = [](const AStar::Node<uint32_t>* node) {
        std::cout << "Current node: " << node->pos.x << ", " << node->pos.y << std::endl;
    };
    pathFinder.setDebugCurrentNode(debugCurrentNode);

    // Set lambda function to debug open node
    std::function<void(const AStar::Node<uint32_t>* node)> debugOpenNode = [](const AStar::Node<uint32_t>* node) {
        std::cout << "Add to open list: " << node->pos.x << ", " << node->pos.y << std::endl;
    };
    pathFinder.setDebugOpenNode(debugOpenNode);

    // Define the map size (width, height)
    pathFinder.setWorldSize({10, 10});

    // Set the heuristic function (manhattan, euclidean, octagonal etc...), it is optional, default is euclidean
    pathFinder.setHeuristic(AStar::Heuristic::manhattan);

    // if you want to enable diagonal movement, it is optional, default is false
    pathFinder.setDiagonalMovement(true);

    // Add a obstacle point (5, 5) and (5, 6)
    pathFinder.addObstacle({5, 5});
    pathFinder.addObstacle({5, 6});

    // Find the path from (0, 0) to (9, 9)
    auto path = pathFinder.findPath({0, 0}, {9, 9});

    // Print the path
    for (auto& p : path) {
        std::cout << p.x << " " << p.y << std::endl;
    }

    return 0;
}
