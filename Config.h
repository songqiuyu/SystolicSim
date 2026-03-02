#pragma once

/**
 * Config.h — Runtime-loaded simulation configuration
 *
 * All parameters are plain member variables populated by Config::load().
 * No static constexpr — every value can be freely modified in sim_config.yaml.
 *
 * Only two values remain compile-time: MAX_PE_ROWS / MAX_PE_COLS, which
 * are used to bound fixed-size C arrays inside SC modules (e.g. PE*[R][C]).
 * They represent the physical upper limit of the hardware you are modelling.
 */

#pragma once
#include <cstdint>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <algorithm>
#include <stdexcept>

namespace SystolicSim {

struct Config {
    // ── Compile-time upper bounds (array sizing only) ─────────
    // Actual PE count is read from the config file at runtime.
    static constexpr int MAX_PE_ROWS = 64;
    static constexpr int MAX_PE_COLS = 64;

    // ── Systolic Array Dimensions ─────────────────────────────
    int PE_ROWS = 32;
    int PE_COLS = 32;
    int NUM_PES() const { return PE_ROWS * PE_COLS; }

    // ── GEMM Matrix Dimensions ────────────────────────────────
    int M = 512;
    int K = 1152;
    int N = 1152;

    // ── Tile Sizes ────────────────────────────────────────────
    int M_TILE = 512;
    int K_TILE = 32;
    int N_TILE = 32;

    // Derived tile counts (computed from fields above)
    int NUM_M_TILES() const { return M / M_TILE; }
    int NUM_K_TILES() const { return K / K_TILE; }
    int NUM_N_TILES() const { return N / N_TILE; }

    // Buffer sizes in bytes (int16_t → each element = 2 bytes)
    int INPUT_TILE_BYTES()  const { return M_TILE * K_TILE * 2; }
    int WEIGHT_TILE_BYTES() const { return K_TILE * N_TILE * 2; }
    int OUTPUT_TILE_BYTES() const { return M_TILE * N_TILE * 2; }

    // Accumulator: int32_t → 4 bytes per element
    int ACCUM_TILE_ELEMENTS() const { return NUM_N_TILES() * M_TILE * N_TILE; }

    // ── Clock Frequencies ─────────────────────────────────────
    double PE_FREQ_GHZ  = 0.5;    // 500 MHz
    double MEM_FREQ_GHZ = 1.6;    // DDR5-3200 command clock
    double CLOCK_RATIO() const { return MEM_FREQ_GHZ / PE_FREQ_GHZ; }

    // ── Memory ────────────────────────────────────────────────
    int    BURST_SIZE         = 64;    // bytes per DRAM burst
    double MEM_BANDWIDTH_GBPS = 51.2;  // peak (4-ch DDR5-3200)

    // ── Simple Memory Model ───────────────────────────────────
    double SIMPLE_READ_LATENCY_NS  = 80.0;
    double SIMPLE_WRITE_LATENCY_NS = 60.0;
    double SIMPLE_BW_GBPS          = 51.2;

    // ── Memory Backend ────────────────────────────────────────
    enum class MemMode { NONE, SIMPLE, RAMULATOR };
    MemMode mem_mode = MemMode::SIMPLE;

    // ── Ramulator config path ─────────────────────────────────
    std::string ramulator_config = "config/config_ddr5_4ch.yaml";

    // ── Data types (fixed by the 16-bit hardware design) ──────
    using DataType   = int16_t;   // input / weight
    using AccumType  = int32_t;   // partial sum (on-chip)
    using OutputType = int16_t;   // final output (saturated)

    // ─────────────────────────────────────────────────────────
    // Load configuration from a key=value text file.
    // Lines starting with '#' are ignored.
    // Unknown keys are silently skipped.
    // Throws std::runtime_error if the file cannot be opened.
    // ─────────────────────────────────────────────────────────
    static Config load(const std::string& path) {
        Config cfg;

        std::ifstream fin(path);
        if (!fin.is_open()) {
            std::cerr << "[Config] Warning: cannot open '" << path
                      << "', using defaults.\n";
            return cfg;          // return defaults rather than crash
        }

        std::map<std::string, std::string> kv;
        std::string line;
        while (std::getline(fin, line)) {
            // Strip leading whitespace
            size_t start = line.find_first_not_of(" \t\r\n");
            if (start == std::string::npos) continue;
            line = line.substr(start);

            // Skip comment lines
            if (line[0] == '#') continue;

            // Find '=' separator
            size_t eq = line.find('=');
            if (eq == std::string::npos) continue;

            std::string key = line.substr(0, eq);
            std::string val = line.substr(eq + 1);

            // Trim whitespace and inline comments
            auto trim = [](std::string& s) {
                size_t e = s.find('#');
                if (e != std::string::npos) s = s.substr(0, e);
                size_t b = s.find_first_not_of(" \t\r\n");
                size_t f = s.find_last_not_of(" \t\r\n");
                s = (b == std::string::npos) ? "" : s.substr(b, f - b + 1);
            };
            trim(key);
            trim(val);

            kv[key] = val;
        }

        // ── Apply parsed values ───────────────────────────────
        auto get_int    = [&](const std::string& k, int    def) -> int    {
            auto it = kv.find(k);
            return it != kv.end() ? std::stoi(it->second) : def;
        };
        auto get_double = [&](const std::string& k, double def) -> double {
            auto it = kv.find(k);
            return it != kv.end() ? std::stod(it->second) : def;
        };
        auto get_str    = [&](const std::string& k, const std::string& def) {
            auto it = kv.find(k);
            return it != kv.end() ? it->second : def;
        };

        cfg.PE_ROWS  = get_int("pe_rows",  cfg.PE_ROWS);
        cfg.PE_COLS  = get_int("pe_cols",  cfg.PE_COLS);

        // Clamp to compile-time maxima
        if (cfg.PE_ROWS > MAX_PE_ROWS) {
            std::cerr << "[Config] pe_rows=" << cfg.PE_ROWS
                      << " exceeds MAX_PE_ROWS=" << MAX_PE_ROWS
                      << ", clamped.\n";
            cfg.PE_ROWS = MAX_PE_ROWS;
        }
        if (cfg.PE_COLS > MAX_PE_COLS) {
            std::cerr << "[Config] pe_cols=" << cfg.PE_COLS
                      << " exceeds MAX_PE_COLS=" << MAX_PE_COLS
                      << ", clamped.\n";
            cfg.PE_COLS = MAX_PE_COLS;
        }

        cfg.M      = get_int("M",      cfg.M);
        cfg.K      = get_int("K",      cfg.K);
        cfg.N      = get_int("N",      cfg.N);
        cfg.M_TILE = get_int("M_tile", cfg.M_TILE);
        cfg.K_TILE = get_int("K_tile", cfg.K_TILE);
        cfg.N_TILE = get_int("N_tile", cfg.N_TILE);

        cfg.PE_FREQ_GHZ  = get_double("pe_freq_ghz",  cfg.PE_FREQ_GHZ);
        cfg.MEM_FREQ_GHZ = get_double("mem_freq_ghz", cfg.MEM_FREQ_GHZ);

        cfg.BURST_SIZE         = get_int   ("burst_size",  cfg.BURST_SIZE);
        cfg.MEM_BANDWIDTH_GBPS = get_double("mem_bw_gbps", cfg.MEM_BANDWIDTH_GBPS);

        cfg.SIMPLE_READ_LATENCY_NS  = get_double("simple_read_latency_ns",  cfg.SIMPLE_READ_LATENCY_NS);
        cfg.SIMPLE_WRITE_LATENCY_NS = get_double("simple_write_latency_ns", cfg.SIMPLE_WRITE_LATENCY_NS);
        cfg.SIMPLE_BW_GBPS          = get_double("simple_bw_gbps",          cfg.SIMPLE_BW_GBPS);

        // Memory mode
        std::string mode_str = get_str("mem_mode", "SIMPLE");
        std::transform(mode_str.begin(), mode_str.end(), mode_str.begin(), ::toupper);
        if      (mode_str == "NONE")      cfg.mem_mode = MemMode::NONE;
        else if (mode_str == "RAMULATOR") cfg.mem_mode = MemMode::RAMULATOR;
        else                              cfg.mem_mode = MemMode::SIMPLE;

        cfg.ramulator_config = get_str("ramulator_config", cfg.ramulator_config);

        return cfg;
    }

    // ── Print active configuration ─────────────────────────────
    void print() const {
        const char* m =
            (mem_mode == MemMode::NONE)      ? "NONE" :
            (mem_mode == MemMode::RAMULATOR)  ? "RAMULATOR" : "SIMPLE";

        std::cout << "=== Active Configuration ===\n";
        std::cout << "  Array:       " << PE_ROWS << " × " << PE_COLS
                  << " = " << NUM_PES() << " PEs\n";
        std::cout << "  GEMM:        M=" << M << " K=" << K << " N=" << N << "\n";
        std::cout << "  Tile:        M_tile=" << M_TILE
                  << " K_tile=" << K_TILE
                  << " N_tile=" << N_TILE << "\n";
        std::cout << "  Tiles:       M×K×N = "
                  << NUM_M_TILES() << "×" << NUM_K_TILES() << "×"
                  << NUM_N_TILES() << " = "
                  << NUM_M_TILES() * NUM_K_TILES() * NUM_N_TILES() << "\n";
        std::cout << "  PE Freq:     " << PE_FREQ_GHZ << " GHz\n";
        std::cout << "  Mem Mode:    " << m << "\n";
        std::cout << "  Bandwidth:   " << MEM_BANDWIDTH_GBPS << " GB/s\n";
        std::cout << "  Data Width:  INT16 (accum INT32, output sat→INT16)\n";
        std::cout << "============================\n";
    }
};

} // namespace SystolicSim
