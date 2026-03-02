/**
 * sc_main.cpp — SystemC 仿真入口（全 runtime Config 版本）
 *
 * 用法：
 *   ./systolic_sc [--config path/to/sim_config.yaml]
 *                 [--mem-mode none|simple|ramulator]
 *
 * 所有参数从 sim_config.yaml 加载，--mem-mode 可覆盖文件中的 mem_mode 字段。
 */

#include <systemc>
#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include <random>
#include <algorithm>

#include "Config.h"
#include "Stats.h"
#include "Top.h"

using namespace SystolicSim;
using namespace sc_core;

// ── 随机生成 int16 数据 ────────────────────────────────────────
static void gen_rand_int16(int16_t* p, size_t n, int seed = 42) {
    std::mt19937 gen(seed);
    std::uniform_int_distribution<int> d(-128, 127);
    for (size_t i = 0; i < n; i++)
        p[i] = static_cast<int16_t>(d(gen));
}

// ─────────────────────────────────────────────────────────────
int sc_main(int argc, char* argv[]) {

    // ── 解析命令行 ────────────────────────────────────────────
    std::string cfg_path     = "config/base.yaml";
    std::string mem_mode_str;
    bool        mem_override = false;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if      (a == "--config"   && i + 1 < argc) { cfg_path     = argv[++i]; }
        else if (a == "--mem-mode" && i + 1 < argc) {
            mem_mode_str = argv[++i];
            mem_override = true;
        }
        else if (a == "--help") {
            std::cout << "Usage: systolic_sc [--config sim_config.yaml]"
                         " [--mem-mode none|simple|ramulator]\n";
            return 0;
        }
    }

    // ── 加载配置 ──────────────────────────────────────────────
    Config cfg = Config::load(cfg_path);

    // 命令行 mem-mode 覆盖文件值
    if (mem_override) {
        std::string m = mem_mode_str;
        std::transform(m.begin(), m.end(), m.begin(), ::toupper);
        if      (m == "NONE")      cfg.mem_mode = Config::MemMode::NONE;
        else if (m == "RAMULATOR") cfg.mem_mode = Config::MemMode::RAMULATOR;
        else                        cfg.mem_mode = Config::MemMode::SIMPLE;
    }

    // ── 打印欢迎信息 ──────────────────────────────────────────
    std::cout << "============================================\n";
    std::cout << "  SystemC Systolic Array Simulator\n";
    std::cout << "  Config: " << cfg_path << "\n";
    std::cout << "============================================\n\n";
    cfg.print();

    // ── 分配主机端数据 ────────────────────────────────────────
    size_t input_elems  = static_cast<size_t>(cfg.M * cfg.K);
    size_t weight_elems = static_cast<size_t>(cfg.K * cfg.N);
    size_t output_elems = static_cast<size_t>(cfg.M * cfg.N);

    std::vector<int16_t> h_in (input_elems,  0);
    std::vector<int16_t> h_wt (weight_elems, 0);
    std::vector<int16_t> h_out(output_elems, 0);

    std::cout << "Generating random INT16 input/weight data...\n";
    gen_rand_int16(h_in.data(),  h_in.size());
    gen_rand_int16(h_wt.data(),  h_wt.size(), 123);

    // 地址空间（抽象）
    constexpr uint64_t INPUT_ADDR  = 0x10000000ULL;
    constexpr uint64_t WEIGHT_ADDR = 0x20000000ULL;
    constexpr uint64_t OUTPUT_ADDR = 0x30000000ULL;

    // ── 统计对象 ──────────────────────────────────────────────
    Stats stats;
    stats.mem_mode = cfg.mem_mode;

    // ── SystemC 时钟（周期 = 1/freq ns）─────────────────────
    double period_ns = 1.0 / cfg.PE_FREQ_GHZ;
    std::cout << "period_ns = " << period_ns << std::endl;
    sc_clock clk("clk", period_ns, SC_NS);

    sc_signal<bool> sig_rst_n ("sig_rst_n");
    sc_signal<bool> sig_start ("sig_start");
    sc_signal<bool> sig_done  ("sig_done");

    // ── 实例化 Top ────────────────────────────────────────────
    Top top("top", cfg, stats);
    top.clk  (clk);
    top.rst_n(sig_rst_n);
    top.start(sig_start);
    top.done (sig_done);

    top.initialize_memory(cfg.ramulator_config);
    top.setup_gemm(INPUT_ADDR, WEIGHT_ADDR, OUTPUT_ADDR);

    // ── 复位序列 ──────────────────────────────────────────────
    sig_rst_n.write(false);
    sig_start.write(false);
    sc_start(4 * period_ns, SC_NS);   // 2 拍
    sig_rst_n.write(true);
    sc_start(2 * period_ns, SC_NS);   // 1 拍稳定

    // ── 启动仿真 ──────────────────────────────────────────────
    std::cout << "\n[sc_main] Starting simulation...\n";
    auto wall_t0 = std::chrono::high_resolution_clock::now();

    sig_start.write(true);
    sc_start(2 * period_ns, SC_NS);
    sig_start.write(false);

    // 主循环：等待 done 信号
    constexpr uint64_t MAX_CYCLES  = 500000000ULL;
    uint64_t           print_every = 1000000ULL;
    uint64_t           next_print  = print_every;

    int total_tiles = cfg.NUM_M_TILES() * cfg.NUM_K_TILES() * cfg.NUM_N_TILES();

    while (!sig_done.read()) {
        sc_start(2 * period_ns, SC_NS);

        if (stats.total_cycles >= next_print) {
            std::cout << "  Progress: "
                      << stats.total_cycles / 1000000 << " M cycles"
                      << "  tiles=" << stats.tiles_processed
                      << "/" << total_tiles << "\n";
            next_print += print_every;
        }

        if (stats.total_cycles > MAX_CYCLES) {
            std::cerr << "[ERROR] Exceeded "
                      << MAX_CYCLES / 1000000 << " M cycles, aborting.\n";
            break;
        }
    }

    // ── 打印结果 ──────────────────────────────────────────────
    auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - wall_t0).count();

    stats.print(cfg);
    std::cout << "\nWall-clock time : " << wall_ms << " ms\n";

    top.finalize();
    std::cout << "\nDone.\n";
    return 0;
}
