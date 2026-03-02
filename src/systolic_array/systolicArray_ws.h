#pragma once
#include <systemc>
#include <vector>
#include <string>
#include "PE16bit.h"
#include "Config.h"

namespace SystolicSim
{
    SC_MODULE(systolicArray_ws)
    {
        const int rows_;
        const int cols_;

        sc_core::sc_in<bool> clk;
        sc_core::sc_in<bool> rst_n;
        sc_core::sc_in<bool> weight_load;
        sc_core::sc_vector<sc_core::sc_in<int16_t>> weight_flat;

        sc_core::sc_vector<sc_core::sc_in<int16_t>> col_input;   // COLS
        sc_core::sc_in<bool> col_input_valid;

        sc_core::sc_vector<sc_core::sc_out<int64_t>> row_output; // ROWS
        sc_core::sc_out<bool> tile_done;
        
        sc_core::sc_in<bool> weight_switch;

        SC_HAS_PROCESS(systolicArray_ws);

        systolicArray_ws(sc_core::sc_module_name name, const Config& cfg)
            : sc_core::sc_module(name),
              rows_(cfg.PE_ROWS),
              cols_(cfg.PE_COLS),
              weight_flat("weight_flat", cfg.PE_ROWS * cfg.PE_COLS),
              col_input("col_input", cfg.PE_COLS),
              row_output("row_output", cfg.PE_ROWS),
              sig_psum_("sig_psum", cfg.PE_ROWS * cfg.PE_COLS),
              sig_top_("sig_top", cfg.PE_COLS),
              sig_psum_zero_("sig_psum_zero", cfg.PE_ROWS)
        {
            stagger_q_.assign(cols_, std::vector<int16_t>()); // Stagger in Columns now

            for (int r = 0; r < Config::MAX_PE_ROWS; r++)
                for (int c = 0; c < Config::MAX_PE_COLS; c++)
                    pe_[r][c] = nullptr;

            // 去连接PEs
            for (int r = 0; r < rows_; r++)
            {
                for (int c = 0; c < cols_; c++)
                {
                    // idx是去算flat后的下标
                    int idx = r * cols_ + c;
                    std::string nm = "pe_" + std::to_string(r) + "_" + std::to_string(c);
                    pe_[r][c] = new PE16bit(nm.c_str());

                    pe_[r][c]->clk(clk);
                    pe_[r][c]->rst_n(rst_n);
                    pe_[r][c]->weight_load(weight_load);
                    pe_[r][c]->weight_switch(weight_switch);

                    pe_[r][c]->weight_in(weight_flat[idx]);

                    // === 垂直广播 (Column Broadcast) ===
                    // 外界打好 Stagger 的列输入 sig_top_[c] 直接广播给该列所有的行 PE
                    pe_[r][c]->input_in(sig_top_[c]);

                    // === Psum 水平流动 (Row-wise Accumulation) Left -> Right ===
                    if (c == 0)
                        pe_[r][c]->psum_in(sig_psum_zero_[r]);
                    else
                        pe_[r][c]->psum_in(sig_psum_[r * cols_ + (c - 1)]);
                    pe_[r][c]->psum_out(sig_psum_[idx]);
                }
            }

            SC_METHOD(drive_outputs);
            sensitive << clk.pos();

            SC_THREAD(stagger_ctrl);
            sensitive << clk.pos();
        }

        ~systolicArray_ws()
        {
            for (int r = 0; r < rows_; r++)
                for (int c = 0; c < cols_; c++)
                    if (pe_[r][c])
                    {
                        delete pe_[r][c];
                        pe_[r][c] = nullptr;
                    }
        }

        void set_num_input_rows(int n)
        {
            num_inputs_ = n; // Technically these are temporal beats of input vectors
            inputs_fed_ = 0;
            cycle_cnt_ = 0;
            done_flag_ = false;
            // Clear stagger queues
            stagger_q_.assign(cols_, {});
        }

    private:
        PE16bit *pe_[Config::MAX_PE_ROWS][Config::MAX_PE_COLS];

        sc_core::sc_vector<sc_core::sc_signal<int64_t>> sig_psum_;
        sc_core::sc_vector<sc_core::sc_signal<int16_t>> sig_top_;     // was sig_left_
        sc_core::sc_vector<sc_core::sc_signal<int64_t>> sig_psum_zero_; // always 0

        std::vector<std::vector<int16_t>> stagger_q_;
        int num_inputs_ = 0;
        int inputs_fed_ = 0;
        uint64_t cycle_cnt_ = 0;
        bool done_flag_ = false;

        // stagger control 是用来控制脉动阵列的
        void drive_outputs()
        {
            // Rightmost column -> row_output
            for (int r = 0; r < rows_; r++)
                row_output[r].write(sig_psum_[r * cols_ + (cols_ - 1)].read());

            // Leftmost padding zeros
            for (int r = 0; r < rows_; r++)
                sig_psum_zero_[r].write(0);
        }

        void stagger_ctrl()
        {
            while (true)
            {
                wait();

                if (!rst_n.read())
                {
                    for (int c = 0; c < cols_; c++)
                        sig_top_[c].write(0);
                    tile_done.write(false);
                    cycle_cnt_ = 0; inputs_fed_ = 0;
                    done_flag_ = false;
                    continue;
                }

                tile_done.write(false);
                cycle_cnt_++;

                for (int c = 0; c < cols_; c++)
                {
                    if (!stagger_q_[c].empty())
                    {
                        sig_top_[c].write(stagger_q_[c].front());
                        stagger_q_[c].erase(stagger_q_[c].begin());
                    }
                    else
                    {
                        sig_top_[c].write(0);
                    }
                }

                if (col_input_valid.read() && inputs_fed_ < num_inputs_)
                {
                    for (int c = 0; c < cols_; c++)
                    {
                        // Pad queue to achieve stagger delay of `c` cycles for col c
                        while ((int)stagger_q_[c].size() < c)
                            stagger_q_[c].push_back(0);
                        stagger_q_[c].push_back(col_input[c].read());
                    }
                    inputs_fed_++;
                }

                // Because rows act in parallel (Broadcast), rows_ dimension is not delayed
                // Pipeline depth depends on COLS (for psum) + num_inputs_ (temporal depth).
                uint64_t done_at = static_cast<uint64_t>(num_inputs_ - 1) + static_cast<uint64_t>(cols_ - 1) + 2;
                if (cycle_cnt_ >= done_at && inputs_fed_ >= num_inputs_ && !done_flag_)
                {
                    tile_done.write(true);
                    done_flag_ = true;
                }
            }
        }
    };
}