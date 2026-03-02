#pragma once

#include <cstdint>
#include <iostream>
#include <iomanip>
#include "Config.h"

namespace SystolicSim {

struct Stats {
    uint64_t total_cycles        = 0;
    uint64_t compute_cycles      = 0;
    uint64_t memory_stall_cycles = 0;
    uint64_t input_load_cycles   = 0;
    uint64_t weight_load_cycles  = 0;
    uint64_t output_write_cycles = 0;

    uint64_t total_memory_reads  = 0;
    uint64_t total_memory_writes = 0;
    uint64_t total_bytes_read    = 0;
    uint64_t total_bytes_written = 0;

    uint64_t total_macs      = 0;
    uint64_t pe_active_cycles= 0;
    uint64_t tiles_processed = 0;

    Config::MemMode mem_mode = Config::MemMode::SIMPLE;

    void reset() { *this = Stats{}; }

    double pe_utilization(const Config& cfg) const {
        if (total_cycles == 0) return 0.0;
        return static_cast<double>(pe_active_cycles) / total_cycles * 100.0;
    }

    double actual_bandwidth_gbps(const Config& cfg) const {
        if (total_cycles == 0) return 0.0;
        double bytes   = static_cast<double>(total_bytes_read + total_bytes_written);
        double time_s  = static_cast<double>(total_cycles) / (cfg.PE_FREQ_GHZ * 1e9);
        return bytes / time_s / 1e9;
    }

    double throughput_gops(const Config& cfg) const {
        if (total_cycles == 0) return 0.0;
        double time_s = static_cast<double>(total_cycles) / (cfg.PE_FREQ_GHZ * 1e9);
        return static_cast<double>(total_macs) * 2.0 / time_s / 1e9;
    }

    void print(const Config& cfg) const {
        const char* m =
            (mem_mode == Config::MemMode::NONE)      ? "NONE (zero-latency)" :
            (mem_mode == Config::MemMode::RAMULATOR)  ? "RAMULATOR (cycle-accurate)" :
                                                        "SIMPLE (bandwidth model)";

        std::cout << "\n========== Simulation Statistics ==========\n";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Memory Mode:          " << m << "\n";
        std::cout << "Data Width:           INT16 (accum INT32, sat→INT16)\n";

        std::cout << "\n--- Cycle Breakdown ---\n";
        std::cout << "Total Cycles:         " << total_cycles           << "\n";
        std::cout << "Compute Cycles:       " << compute_cycles         << "\n";
        std::cout << "Memory Stall Cycles:  " << memory_stall_cycles    << "\n";
        std::cout << "  - Input Load:       " << input_load_cycles      << "\n";
        std::cout << "  - Weight Load:      " << weight_load_cycles     << "\n";
        std::cout << "  - Output Write:     " << output_write_cycles    << "\n";

        std::cout << "\n--- Memory Statistics ---\n";
        std::cout << "Bytes Read:           " << total_bytes_read   / 1024.0 / 1024.0 << " MB\n";
        std::cout << "Bytes Written:        " << total_bytes_written / 1024.0 / 1024.0 << " MB\n";

        std::cout << "\n--- Compute Statistics ---\n";
        std::cout << "Total MACs:           " << total_macs / 1e9   << " G\n";
        std::cout << "Tiles Processed:      " << tiles_processed     << "\n";

        double bw   = actual_bandwidth_gbps(cfg);
        double gops = throughput_gops(cfg);
        double peak = cfg.NUM_PES() * cfg.PE_FREQ_GHZ * 2.0;

        std::cout << "\n--- Performance Metrics ---\n";
        std::cout << "PE Utilization:       " << pe_utilization(cfg)         << " %\n";
        std::cout << "Actual Bandwidth:     " << bw                          << " GB/s\n";
        std::cout << "Bandwidth Util:       " << bw / cfg.MEM_BANDWIDTH_GBPS * 100.0 << " %\n";
        std::cout << "Throughput:           " << gops                        << " GOPS\n";
        std::cout << "Peak Throughput:      " << peak                        << " GOPS\n";
        std::cout << "Efficiency:           " << gops / peak * 100.0         << " %\n";

        double ms = static_cast<double>(total_cycles) / (cfg.PE_FREQ_GHZ * 1e6);
        std::cout << "Execution Time:       " << ms                          << " ms\n";
        std::cout << "============================================\n";
    }
};

} // namespace SystolicSim
