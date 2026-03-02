#pragma once
#include <cstdint>

namespace SystolicSim {
    struct Config {
        int PE_ROWS;
        int PE_COLS;

        // Compile-time maximums for static arrays
        static constexpr int MAX_PE_ROWS = 128;
        static constexpr int MAX_PE_COLS = 128;

        Config(int rows = 32, int cols = 32) : PE_ROWS(rows), PE_COLS(cols) {}
    };
}
