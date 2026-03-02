#pragma once

/**
 * SystolicArray.h — Runtime-configurable 32×32 Systolic Array (SystemC)
 *
 * Key rules for SystemC:
 *  - sc_signal is NOT copyable/movable → must use sc_vector, not std::vector
 *  - Member init order must match declaration order (avoid -Wreorder)
 *  - rows_ / cols_ declared BEFORE sc_vector ports so they're ready first
 */

#include <systemc>
#include <vector>
#include <string>
#include "Config.h"
#include "PE.h"

namespace SystolicSim {

SC_MODULE(SystolicArray) {

    // ── Runtime dimensions (declared FIRST so initializer order is safe) ──
    // These are set from Config before any sc_vector is constructed.
    const int rows_;
    const int cols_;

    // ── Ports (sized at construction time) ─────────────────────────────────
    sc_core::sc_in<bool>  clk;
    sc_core::sc_in<bool>  rst_n;
    sc_core::sc_in<bool>  weight_load;

    sc_core::sc_vector<sc_core::sc_in<Config::DataType>>   weight_flat; // rows*cols
    sc_core::sc_vector<sc_core::sc_in<Config::DataType>>   row_input;   // rows
    sc_core::sc_in<bool>  row_input_valid;

    sc_core::sc_vector<sc_core::sc_out<Config::AccumType>> col_output;  // cols
    sc_core::sc_out<bool> tile_done;

    // ── Constructor ────────────────────────────────────────────────────────
    SC_HAS_PROCESS(SystolicArray);

    SystolicArray(sc_core::sc_module_name name, const Config& cfg)
        : sc_core::sc_module(name)
        // rows_/cols_ FIRST (they are declared first above)
        , rows_(cfg.PE_ROWS)
        , cols_(cfg.PE_COLS)
        // Ports
        , weight_flat("weight_flat", cfg.PE_ROWS * cfg.PE_COLS)
        , row_input  ("row_input",   cfg.PE_ROWS)
        , col_output ("col_output",  cfg.PE_COLS)
        // Internal signals (sc_vector avoids copy/move issues)
        , sig_input_ ("sig_input",  cfg.PE_ROWS * cfg.PE_COLS)
        , sig_psum_  ("sig_psum",   cfg.PE_ROWS * cfg.PE_COLS)
        , sig_left_  ("sig_left",   cfg.PE_ROWS)
        , sig_psum_zero_("sig_psum_zero", cfg.PE_COLS)
    {
        // Initialize stagger_q_ size to prevent out-of-bounds in thread
        stagger_q_.assign(rows_, std::vector<Config::DataType>());

        // Initialise PE pointer array to null
        for (int r = 0; r < Config::MAX_PE_ROWS; r++)
            for (int c = 0; c < Config::MAX_PE_COLS; c++)
                pe_[r][c] = nullptr;

        // Instantiate and bind PEs
        for (int r = 0; r < rows_; r++) {
            for (int c = 0; c < cols_; c++) {
                int idx = r * cols_ + c;
                std::string nm = "pe_" + std::to_string(r) + "_" + std::to_string(c);
                pe_[r][c] = new PE(nm.c_str());

                pe_[r][c]->clk(clk);
                pe_[r][c]->rst_n(rst_n);
                pe_[r][c]->weight_in(weight_flat[idx]);
                pe_[r][c]->weight_load(weight_load);

                // Input: col 0 from stagger signal, others from left neighbour
                if (c == 0) pe_[r][c]->input_in(sig_left_[r]);
                else        pe_[r][c]->input_in(sig_input_[r * cols_ + (c - 1)]);
                pe_[r][c]->input_out(sig_input_[idx]);

                // Psum: row 0 from zero-driver, others from top neighbour
                if (r == 0) pe_[r][c]->psum_in(sig_psum_zero_[c]);
                else        pe_[r][c]->psum_in(sig_psum_[(r - 1) * cols_ + c]);
                pe_[r][c]->psum_out(sig_psum_[idx]);
            }
        }

        SC_METHOD(drive_outputs);
        sensitive << clk.pos();

        SC_THREAD(stagger_ctrl);
        sensitive << clk.pos();
    }

    ~SystolicArray() {
        for (int r = 0; r < rows_; r++)
            for (int c = 0; c < cols_; c++)
                if (pe_[r][c]) { delete pe_[r][c]; pe_[r][c] = nullptr; }
    }

    // Called by Scheduler before each tile
    void set_num_input_rows(int n) {
        num_rows_  = n;
        rows_fed_  = 0;
        cycle_cnt_ = 0;
        done_flag_ = false;
        // Clear stagger queues
        stagger_q_.assign(rows_, {});
    }

private:
    // ── PE array (bounded by compile-time maxima) ──────────────────────────
    PE* pe_[Config::MAX_PE_ROWS][Config::MAX_PE_COLS];

    // ── Internal signals (sc_vector — sc_signal is not copyable!) ─────────
    sc_core::sc_vector<sc_core::sc_signal<Config::DataType>>  sig_input_;
    sc_core::sc_vector<sc_core::sc_signal<Config::AccumType>> sig_psum_;
    sc_core::sc_vector<sc_core::sc_signal<Config::DataType>>  sig_left_;
    sc_core::sc_vector<sc_core::sc_signal<Config::AccumType>> sig_psum_zero_; // always 0

    // ── Stagger state ──────────────────────────────────────────────────────
    std::vector<std::vector<Config::DataType>> stagger_q_;
    int      num_rows_  = 0;
    int      rows_fed_  = 0;
    uint64_t cycle_cnt_ = 0;
    bool     done_flag_ = false;

    // ── Processes ──────────────────────────────────────────────────────────
    void drive_outputs() {
        // Bottom row → col_output
        for (int c = 0; c < cols_; c++)
            col_output[c].write(sig_psum_[(rows_ - 1) * cols_ + c].read());
        // Keep zero-driver signals at 0
        for (int c = 0; c < cols_; c++)
            sig_psum_zero_[c].write(0);
    }

    void stagger_ctrl() {
        while (true) {
            wait();

            if (!rst_n.read()) {
                for (int r = 0; r < rows_; r++) sig_left_[r].write(0);
                tile_done.write(false);
                cycle_cnt_ = 0; rows_fed_ = 0; done_flag_ = false;
                continue;
            }

            tile_done.write(false);
            cycle_cnt_++;

            // Emit front of each row's stagger queue
            for (int r = 0; r < rows_; r++) {
                if (!stagger_q_[r].empty()) {
                    sig_left_[r].write(stagger_q_[r].front());
                    stagger_q_[r].erase(stagger_q_[r].begin());
                } else {
                    sig_left_[r].write(0);
                }
            }

            // Accept new input row from Scheduler
            if (row_input_valid.read() && rows_fed_ < num_rows_) {
                for (int r = 0; r < rows_; r++) {
                    // Pad queue to achieve stagger delay of `r` cycles for row r
                    while ((int)stagger_q_[r].size() < r)
                        stagger_q_[r].push_back(0);
                    stagger_q_[r].push_back(row_input[r].read());
                }
                rows_fed_++;
            }

            // Assert tile_done once all data has drained through the array
            uint64_t done_at = static_cast<uint64_t>(num_rows_ - 1)
                             + static_cast<uint64_t>(rows_ - 1)
                             + static_cast<uint64_t>(cols_ - 1) + 1;
            if (cycle_cnt_ >= done_at && rows_fed_ >= num_rows_ && !done_flag_) {
                tile_done.write(true);
                done_flag_ = true;
            }
        }
    }
};

} // namespace SystolicSim
