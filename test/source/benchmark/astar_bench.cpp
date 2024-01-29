#include <array>
#include <string>
#include <string_view>
#include <vector>

#include <benchmark/benchmark.h>
#include "astar/astar.hpp"
#include "generator/generator.hpp"

static constexpr int64_t multiplier = 4;
static constexpr int64_t minRange = 16;
static constexpr int64_t maxRange = 256;
static constexpr int64_t minThreadRange = 1;
static constexpr int64_t maxThreadRange = 1;
static constexpr int64_t repetitions = 1;

static void DoSetup([[maybe_unused]] const benchmark::State& state) {}

static void DoTeardown([[maybe_unused]] const benchmark::State& state) {}

template <IntegerType T>
static void astar_bench(benchmark::State& state) {
    auto range = state.range(0);

    int mapWidth = range;
    int mapHeight = range;

    float lacunarity = 1.6f;
    float octaves = 6;
    float gain = 3.5f;
    float frequency = 1.7f;
    float weightedStrength = 0.034f;
    float multiplier = 118;

    Generator generator(-972960945);
    benchmark::DoNotOptimize(generator);
    generator.setLacunarity(lacunarity);
    generator.setOctaves((uint32_t)octaves);
    generator.setGain(gain);
    generator.setFrequency(frequency);
    generator.setWeightedStrength(weightedStrength);
    generator.setMultiplier((uint32_t)multiplier);

    std::vector<uint32_t> heightmap = generator.generate2dMeightmap(0, 0, 0, mapWidth, 0, mapHeight);
    benchmark::DoNotOptimize(heightmap);

    std::vector<uint8_t> blocks = std::vector<uint8_t>(mapWidth * mapHeight, 0);
    benchmark::DoNotOptimize(blocks);

    AStar::AStar<T, false> pathFinder;
    benchmark::DoNotOptimize(pathFinder);
    pathFinder.setWorldSize({mapWidth, mapHeight});
    pathFinder.setHeuristic(AStar::Heuristic::euclidean);
    pathFinder.setDiagonalMovement(true);

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

    AStar::Vec2i source(0, 0);
    AStar::Vec2i target(mapWidth - 1, mapHeight - 1);

    std::vector<AStar::Vec2i> path;
    benchmark::DoNotOptimize(path);

    for (auto _ : state) {
        path = pathFinder.findPath(source, target);
        state.PauseTiming();
        if (path.size() == 0) {
            state.SkipWithError("No path found");
        }
        state.ResumeTiming();

        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations());
    state.SetBytesProcessed(state.iterations() * sizeof(path));
}

BENCHMARK(astar_bench<uint32_t>)
    ->Name("astar_bench<uint32_t>")
    ->RangeMultiplier(multiplier)
    ->Range(minRange, maxRange)
    ->ThreadRange(minThreadRange, maxThreadRange)
    ->Unit(benchmark::kNanosecond)
    ->Setup(DoSetup)
    ->Teardown(DoTeardown)
    ->MeasureProcessCPUTime()
    ->UseRealTime()
    ->Repetitions(repetitions);

template <IntegerType T>
static void astar_bench_fast(benchmark::State& state) {
    auto range = state.range(0);

    int mapWidth = range;
    int mapHeight = range;

    float lacunarity = 1.6f;
    float octaves = 6;
    float gain = 3.5f;
    float frequency = 1.7f;
    float weighted_strength = 0.034f;
    float multiplier = 118;

    Generator generator(-972960945);
    benchmark::DoNotOptimize(generator);
    generator.setLacunarity(lacunarity);
    generator.setOctaves((uint32_t)octaves);
    generator.setGain(gain);
    generator.setFrequency(frequency);
    generator.setWeightedStrength(0.0f);
    generator.setMultiplier((uint32_t)multiplier);

    std::vector<uint32_t> heightmap = generator.generate2dMeightmap(0, 0, 0, mapWidth, 0, mapHeight);
    benchmark::DoNotOptimize(heightmap);

    std::vector<uint32_t> blocks = std::vector<uint32_t>(mapWidth * mapHeight, 0);
    benchmark::DoNotOptimize(blocks);

    AStar::AStarFast<T, false, uint32_t> pathFinder;
    benchmark::DoNotOptimize(pathFinder);
    pathFinder.setHeuristic(AStar::Heuristic::euclidean);
    pathFinder.setDiagonalMovement(true);

    for (uint64_t x = 0; x < mapWidth; x++) {
        for (uint64_t y = 0; y < mapHeight; y++) {
            uint64_t index = x + y * mapWidth;
            uint8_t value = static_cast<uint8_t>(heightmap[index]);

            if (value < 128) {
                blocks[index] = 0;
            } else {
                blocks[index] = 1;
            }
        }
    }

    blocks[0] = 0;
    blocks[mapWidth * mapHeight - 1] = 0;

    AStar::Vec2i source(0, 0);
    AStar::Vec2i target(mapWidth - 1, mapHeight - 1);

    std::vector<AStar::Vec2i> path;
    benchmark::DoNotOptimize(path);

    for (auto _ : state) {
        path = pathFinder.findPath(source, target, blocks, {mapWidth, mapHeight});
        state.PauseTiming();
        if (path.size() == 0) {
            state.SkipWithError("No path found");
        }
        state.ResumeTiming();

        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(state.iterations());
    state.SetBytesProcessed(state.iterations() * sizeof(path));
}

BENCHMARK(astar_bench_fast<uint32_t>)
    ->Name("astar_bench_fast<uint32_t>")
    ->RangeMultiplier(multiplier)
    ->Range(minRange, maxRange)
    ->ThreadRange(minThreadRange, maxThreadRange)
    ->Unit(benchmark::kNanosecond)
    ->Setup(DoSetup)
    ->Teardown(DoTeardown)
    ->MeasureProcessCPUTime()
    ->UseRealTime()
    ->Repetitions(repetitions);

// Run the benchmark
// BENCHMARK_MAIN();

int main(int argc, char** argv) {
    ::benchmark::Initialize(&argc, argv);
    ::benchmark::RunSpecifiedBenchmarks();
}
