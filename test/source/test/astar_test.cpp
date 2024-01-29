#include "astar/astar.hpp"

#include "gtest/gtest.h"

TEST(AStar, basic_path_1) {
    int mapWidth = 4;
    int mapHeight = 4;
    AStar::AStar pathFinder;
    pathFinder.setWorldSize({mapWidth, mapWidth});
    pathFinder.setHeuristic(AStar::Heuristic::euclidean);
    pathFinder.setDiagonalMovement(true);

    AStar::Vec2i source(0, 0);
    AStar::Vec2i target(mapWidth - 1, mapHeight - 1);

    std::cout << "AStar::AStar pathFinder;" << std::endl;
    auto path = pathFinder.findPath(source, target);

    EXPECT_EQ(path.size(), 4);

    for (size_t i = 0; i < path.size(); i++) {
        EXPECT_EQ(path[i].x, path.size() - i - 1);
        EXPECT_EQ(path[i].y, path.size() - i - 1);
    }
}

TEST(AStar, basic_path_2) {
    int mapWidth = 10;
    int mapHeight = 10;
    AStar::AStar pathFinder;
    pathFinder.setWorldSize({mapWidth, mapWidth});
    pathFinder.setHeuristic(AStar::Heuristic::euclidean);
    pathFinder.setDiagonalMovement(true);

    AStar::Vec2i source(0, 0);
    AStar::Vec2i target(mapWidth - 1, mapHeight - 1);

    auto path = pathFinder.findPath(source, target);

    EXPECT_EQ(path.size(), 10);

    for (size_t i = 0; i < path.size(); i++) {
        EXPECT_EQ(path[i].x, path.size() - i - 1);
        EXPECT_EQ(path[i].y, path.size() - i - 1);
    }
}

TEST(AStar, basic_diagonal_path_wrong_1) {
    int mapWidth = 10;
    int mapHeight = 10;
    AStar::AStar pathFinder;
    pathFinder.setWorldSize({mapWidth, mapHeight});
    pathFinder.setHeuristic(AStar::Heuristic::euclidean);
    pathFinder.setDiagonalMovement(true);

    AStar::Vec2i source(0, 0);
    AStar::Vec2i target(19, 19);

    auto path = pathFinder.findPath(source, target);

    EXPECT_EQ(path.size(), 0);
}

TEST(AStar, basic_diagonal_path_wrong_2) {
    int mapWidth = 10;
    int mapHeight = 10;
    AStar::AStar pathFinder;
    pathFinder.setWorldSize({mapWidth, mapHeight});
    pathFinder.setHeuristic(AStar::Heuristic::euclidean);
    pathFinder.setDiagonalMovement(true);

    pathFinder.addObstacle({0, 1});
    pathFinder.addObstacle({1, 1});
    pathFinder.addObstacle({1, 0});

    AStar::Vec2i source(0, 0);
    AStar::Vec2i target(9, 9);

    auto path = pathFinder.findPath(source, target);

    EXPECT_EQ(path.size(), 0);
}

auto main(int argc, char** argv) -> int {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
