#include <array>     // std::array
#include <chrono>    // std::chrono::system_clock
#include <cmath>     // std::abs
#include <cstdint>   // std::uint32_t
#include <iostream>  // std::cout, std::endl
#include <map>       // std::map
#include <memory>    // std::unique_ptr
#include <random>    // std::random_device, std::mt19937, std::uniform_int_distribution
#include <vector>    // std::vector

#include "astar/astar.hpp"
#include "generator/generator.hpp"

#include "raylib.h"

#define RAYGUI_IMPLEMENTATION
extern "C" {
#include "src/raygui.h"
}

auto main() -> int {
    // Set log level for Raylib
    SetTraceLogLevel(LOG_WARNING);

    const int screenWidth = 1920;
    const int screenHeight = 1080;

    const int mapWidth = 192;
    const int mapHeight = 108;

    const uint32_t targetFPS = 120;

    const uint32_t ImageUpdatePerSecond = 30;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Path finder by Bensuperpc");

    SetTargetFPS(targetFPS);

    float lacunarity = 1.6f;
    float octaves = 6;
    float gain = 3.5f;
    float frequency = 1.7f;
    float weighted_strength = 0.034f;
    float multiplier = 118;

    Generator generator_2(-972960945);
    generator_2.setLacunarity(lacunarity);
    generator_2.setOctaves((uint32_t)octaves);
    generator_2.setGain(gain);
    generator_2.setFrequency(frequency);
    generator_2.setWeightedStrength(0.0f);
    generator_2.setMultiplier((uint32_t)multiplier);

    AStar::AStar<uint32_t, false> pathFinder;
    pathFinder.setWorldSize({mapWidth, mapHeight});
    pathFinder.setHeuristic(AStar::Heuristic::manhattan);
    pathFinder.setDiagonalMovement(true);

    size_t manhattanPathSize = 0;
    size_t euclideanPathSize = 0;
    size_t euclideanNoSQRPathSize = 0;
    size_t octagonalPathSize = 0;
    size_t chebyshevPathSize = 0;
    size_t dijkstraPathSize = 0;

    std::vector<uint32_t> heightmap;

    heightmap = generator_2.generate2dMeightmap(0, 0, 0, mapWidth, 0, mapHeight);

    std::vector<uint8_t> blocks = std::vector<uint8_t>(mapWidth * mapHeight, 0);

    uint64_t framesCounter = 0;

    bool needUpdate = true;

    while (!WindowShouldClose()) {
        framesCounter++;
        if (IsKeyPressed(KEY_S)) {
            const std::string filename = "screenshot_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".png";
            TakeScreenshot(filename.c_str());
        }
        if (IsKeyPressed(KEY_R)) {
            generator_2.randomizeSeed();
            needUpdate = true;
        }

        if (framesCounter % (targetFPS / ImageUpdatePerSecond) == 0) {
            if (needUpdate) {
                needUpdate = false;
                generator_2.setLacunarity(lacunarity);
                generator_2.setOctaves((uint32_t)octaves);
                generator_2.setGain(gain);
                generator_2.setFrequency(frequency);
                generator_2.setWeightedStrength(weighted_strength);
                generator_2.setMultiplier((uint32_t)multiplier);

                pathFinder.clear();

                heightmap = generator_2.generate2dMeightmap(0, 0, 0, screenWidth, 0, screenHeight);

                for (uint64_t x = 0; x < mapWidth; x++) {
                    for (uint64_t y = 0; y < mapHeight; y++) {
                        uint64_t index = x + y * mapWidth;
                        uint8_t value = static_cast<uint8_t>(heightmap[index]);

                        if (value < 128) {
                            blocks[index] = 0;
                        } else {
                            blocks[index] = 1;
                            pathFinder.addObstacle({static_cast<int32_t>(x), static_cast<int32_t>(y)});
                        }
                    }
                }
                blocks[0] = 0;
                pathFinder.removeObstacle({0, 0});
                blocks[mapWidth * mapHeight - 1] = 0;
                pathFinder.removeObstacle({mapWidth - 1, mapHeight - 1});

                pathFinder.setHeuristic(AStar::Heuristic::manhattan);
                auto start1 = std::chrono::high_resolution_clock::now();
                auto path = pathFinder.findPath({0, 0}, {mapWidth - 1, mapHeight - 1});
                auto stop1 = std::chrono::high_resolution_clock::now();
                if (path.empty()) {
                    std::cout << "Path not found" << std::endl;
                }
                auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start1);
                std::cout << "Path search: " << duration1.count() << " microseconds" << std::endl;

                manhattanPathSize = path.size();
                for (auto& i : path) {
                    uint64_t index = i.x + i.y * mapWidth;
                    blocks[index] = 2;
                }

                pathFinder.setHeuristic(AStar::Heuristic::euclidean);
                path = pathFinder.findPath({0, 0}, {mapWidth - 1, mapHeight - 1});
                euclideanPathSize = path.size();

                for (auto& i : path) {
                    uint64_t index = i.x + i.y * mapWidth;
                    blocks[index] = 3;
                }

                pathFinder.setHeuristic(AStar::Heuristic::octagonal);
                path = pathFinder.findPath({0, 0}, {mapWidth - 1, mapHeight - 1});
                octagonalPathSize = path.size();

                for (auto& i : path) {
                    uint64_t index = i.x + i.y * mapWidth;
                    blocks[index] = 4;
                }

                pathFinder.setHeuristic(AStar::Heuristic::chebyshev);
                path = pathFinder.findPath({0, 0}, {mapWidth - 1, mapHeight - 1});
                chebyshevPathSize = path.size();

                for (auto& i : path) {
                    uint64_t index = i.x + i.y * mapWidth;
                    blocks[index] = 5;
                }

                pathFinder.setHeuristic(AStar::Heuristic::euclideanNoSQR);
                path = pathFinder.findPath({0, 0}, {mapWidth - 1, mapHeight - 1});
                euclideanNoSQRPathSize = path.size();

                for (auto& i : path) {
                    uint64_t index = i.x + i.y * mapWidth;
                    blocks[index] = 6;
                }

                pathFinder.setHeuristic(AStar::Heuristic::dijkstra);
                path = pathFinder.findPath({0, 0}, {mapWidth - 1, mapHeight - 1});
                dijkstraPathSize = path.size();

                for (auto& i : path) {
                    uint64_t index = i.x + i.y * mapWidth;
                    blocks[index] = 7;
                }
            }
        }

        ClearBackground(RAYWHITE);
        BeginDrawing();

        // Draw white if blocks[index] == 0 else black
        int size = 10;
        for (uint64_t x = 0; x < mapWidth; x++) {
            for (uint64_t y = 0; y < mapHeight; y++) {
                uint64_t index = x + y * mapWidth;
                if (blocks[index] == 0) {
                    DrawRectangle(static_cast<float>(x * size), static_cast<float>(y * size), size, size, WHITE);
                } else if (blocks[index] == 1) {
                    DrawRectangle(static_cast<float>(x * size), static_cast<float>(y * size), size, size, BLACK);
                } else if (blocks[index] == 2) {
                    DrawRectangle(static_cast<float>(x * size), static_cast<float>(y * size), size, size, RED);
                } else if (blocks[index] == 3) {
                    DrawRectangle(static_cast<float>(x * size), static_cast<float>(y * size), size, size, GREEN);
                } else if (blocks[index] == 4) {
                    DrawRectangle(static_cast<float>(x * size), static_cast<float>(y * size), size, size, BLUE);
                } else if (blocks[index] == 5) {
                    DrawRectangle(static_cast<float>(x * size), static_cast<float>(y * size), size, size, YELLOW);
                } else if (blocks[index] == 6) {
                    DrawRectangle(static_cast<float>(x * size), static_cast<float>(y * size), size, size, ORANGE);
                } else if (blocks[index] == 7) {
                    DrawRectangle(static_cast<float>(x * size), static_cast<float>(y * size), size, size, PURPLE);
                }
            }
        }

        // display FPS
        DrawRectangle(screenWidth - 90, 10, 80, 20, Fade(SKYBLUE, 0.95f));
        DrawText(TextFormat("FPS: %02d", GetFPS()), screenWidth - 80, 15, 15, DARKGRAY);

        DrawRectangle(0, 0, 275, 200, Fade(SKYBLUE, 0.95f));
        DrawRectangleLines(0, 0, 275, 200, BLUE);
        GuiSlider((Rectangle){70, 10, 165, 20}, "Lacunarity", TextFormat("%2.3f", lacunarity), &lacunarity, -0.0f, 5.0f);
        GuiSlider((Rectangle){70, 40, 165, 20}, "Octaves", TextFormat("%2.3f", octaves), &octaves, 1, 12);
        GuiSlider((Rectangle){70, 70, 165, 20}, "Gain", TextFormat("%2.3f", gain), &gain, -0.0f, 16.0f);
        GuiSlider((Rectangle){70, 100, 165, 20}, "Frequency", TextFormat("%2.3f", frequency), &frequency, -0.0f, 10.0f);
        GuiSlider((Rectangle){70, 130, 165, 20}, "Weight", TextFormat("%2.3f", weighted_strength), &weighted_strength, -5.0f, 5.0f);
        GuiSlider((Rectangle){70, 160, 165, 20}, "Multiplier", TextFormat("%2.3f", multiplier), &multiplier, 1, 512);

        // display info each color for each heuristic
        DrawRectangle(0, 200, 275, 190, Fade(SKYBLUE, 0.95f));
        DrawRectangleLines(0, 200, 275, 190, BLUE);

        std::string manhattanText = "Manhattan: " + std::to_string(manhattanPathSize);
        DrawRectangle(10, 210, 20, 20, RED);
        DrawText(manhattanText.c_str(), 40, 210, 20, DARKGRAY);

        std::string euclideanText = "Euclidean: " + std::to_string(euclideanPathSize);
        DrawRectangle(10, 240, 20, 20, GREEN);
        DrawText(euclideanText.c_str(), 40, 240, 20, DARKGRAY);

        std::string octagonalText = "Octagonal: " + std::to_string(octagonalPathSize);
        DrawRectangle(10, 270, 20, 20, BLUE);
        DrawText(octagonalText.c_str(), 40, 270, 20, DARKGRAY);

        std::string chebyshevText = "Chebyshev: " + std::to_string(chebyshevPathSize);
        DrawRectangle(10, 300, 20, 20, YELLOW);
        DrawText(chebyshevText.c_str(), 40, 300, 20, DARKGRAY);

        std::string euclideanNoSQRText = "EuclideanNoSQR: " + std::to_string(euclideanNoSQRPathSize);
        DrawRectangle(10, 330, 20, 20, ORANGE);
        DrawText(euclideanNoSQRText.c_str(), 40, 330, 20, DARKGRAY);

        std::string dijkstraText = "Dijkstra: " + std::to_string(dijkstraPathSize);
        DrawRectangle(10, 360, 20, 20, PURPLE);
        DrawText(dijkstraText.c_str(), 40, 360, 20, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
