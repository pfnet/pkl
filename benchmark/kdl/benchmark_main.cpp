#include <benchmark/benchmark.h>

extern void BM_PKL_FORWARD_DQ(benchmark::State& state);
extern void BM_PKL_FORWARD_HG(benchmark::State& state);
extern void BM_PKL_FORWARD_RP(benchmark::State& state);
extern void BM_KDL_FORWARD(benchmark::State& state);
extern void BM_PKL_JACOBIAN_DQ(benchmark::State& state);
extern void BM_PKL_JACOBIAN_HG(benchmark::State& state);
extern void BM_PKL_JACOBIAN_RP(benchmark::State& state);
extern void BM_KDL_JACOBIAN(benchmark::State& state);
extern void BM_PKL_INVERSE_DQ(benchmark::State& state);
extern void BM_PKL_INVERSE_HG(benchmark::State& state);
extern void BM_PKL_INVERSE_RP(benchmark::State& state);
extern void BM_KDL_INVERSE(benchmark::State& state);

BENCHMARK(BM_PKL_FORWARD_DQ);
BENCHMARK(BM_PKL_FORWARD_HG);
BENCHMARK(BM_PKL_FORWARD_RP);
BENCHMARK(BM_KDL_FORWARD);
BENCHMARK(BM_PKL_JACOBIAN_DQ);
BENCHMARK(BM_PKL_JACOBIAN_HG);
BENCHMARK(BM_PKL_JACOBIAN_RP);
BENCHMARK(BM_KDL_JACOBIAN);
BENCHMARK(BM_PKL_INVERSE_DQ);
BENCHMARK(BM_PKL_INVERSE_HG);
BENCHMARK(BM_PKL_INVERSE_RP);
BENCHMARK(BM_KDL_INVERSE);
BENCHMARK_MAIN();
