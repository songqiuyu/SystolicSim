#include <systemc>
#include <vector>
#include <iostream>
#include "systolic_array/systolicArray_ws.h"
#include "memory/buffer.h"
#include "Config.h"

int sc_main(int argc, char* argv[]) {
    using namespace SystolicSim;

    // You can modify rows and cols here dynamically
    Config cfg(3, 3); 
    int total_pes = cfg.PE_ROWS * cfg.PE_COLS;
    int buffer_depth = 64;
    int input_depth = buffer_depth; // Full depth

    sc_core::sc_clock clk("clk", 10, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n("rst_n");

    // === Signals for interconnecting modules ===
    // We now have independent valid/load for each column's weight buffer? 
    // Wait, the systolic array "weight_load" is a single signal.
    sc_core::sc_signal<bool> weight_load("weight_load");
    sc_core::sc_vector<sc_core::sc_signal<int16_t>> weight_flat("weight_flat", total_pes);
    sc_core::sc_signal<bool> weight_switch("weight_switch");
    
    sc_core::sc_vector<sc_core::sc_signal<int16_t>> col_input("col_input", cfg.PE_COLS);
    sc_core::sc_signal<bool> col_input_valid("col_input_valid");
    
    sc_core::sc_vector<sc_core::sc_signal<int64_t>> row_output("row_output", cfg.PE_ROWS);
    sc_core::sc_signal<bool> tile_done("tile_done");

    // === Buffer Signals ===
    // Weight Buffer per column
    std::vector<sc_core::sc_signal<bool>> wb_wr_en(cfg.PE_COLS);
    std::vector<sc_core::sc_signal<bool>> wb_wr_ready(cfg.PE_COLS);
    std::vector<sc_core::sc_signal<bool>> wb_rd_en(cfg.PE_COLS);
    std::vector<sc_core::sc_signal<bool>> wb_rd_valid(cfg.PE_COLS);
    sc_core::sc_vector<sc_core::sc_signal<int16_t>> wb_wr_data("wb_wr_data", total_pes);

    for (int c = 0; c < cfg.PE_COLS; c++) {
        wb_wr_en[c].name();
        wb_wr_ready[c].name();
        wb_rd_en[c].name();
        wb_rd_valid[c].name();
    }

    sc_core::sc_signal<bool> ib_wr_en("ib_wr_en");
    sc_core::sc_vector<sc_core::sc_signal<int16_t>> ib_wr_data("ib_wr_data", cfg.PE_COLS);
    sc_core::sc_signal<bool> ib_wr_ready("ib_wr_ready");
    sc_core::sc_signal<bool> ib_rd_en("ib_rd_en");

    // Output buffer signals
    sc_core::sc_signal<bool> ob_wr_en("ob_wr_en");
    sc_core::sc_signal<bool> ob_wr_ready("ob_wr_ready");
    sc_core::sc_signal<bool> ob_rd_en("ob_rd_en");
    sc_core::sc_signal<bool> ob_rd_valid("ob_rd_valid");
    sc_core::sc_vector<sc_core::sc_signal<int64_t>> ob_rd_data("ob_rd_data", cfg.PE_ROWS);

    // === Instantiate Modules ===
    systolicArray_ws dut("dut", cfg);
    dut.clk(clk);
    dut.rst_n(rst_n);
    dut.weight_load(weight_load);
    dut.weight_switch(weight_switch);
    dut.col_input_valid(col_input_valid);
    dut.tile_done(tile_done);

    std::vector<PingPongBuffer<int16_t>*> weight_bufs(cfg.PE_COLS);
    for (int c = 0; c < cfg.PE_COLS; c++) {
        std::string name = "weight_buf_col_" + std::to_string(c);
        weight_bufs[c] = new PingPongBuffer<int16_t>(name.c_str(), cfg.PE_ROWS, buffer_depth);
        weight_bufs[c]->clk(clk);
        weight_bufs[c]->rst_n(rst_n);
        weight_bufs[c]->wr_en(wb_wr_en[c]);
        weight_bufs[c]->wr_ready(wb_wr_ready[c]);
        weight_bufs[c]->rd_en(wb_rd_en[c]);
        weight_bufs[c]->rd_valid(wb_rd_valid[c]);
        for (int r = 0; r < cfg.PE_ROWS; r++) {
            weight_bufs[c]->wr_data[r](wb_wr_data[r * cfg.PE_COLS + c]);
            weight_bufs[c]->rd_data[r](weight_flat[r * cfg.PE_COLS + c]);
        }
    }

    PingPongBuffer<int16_t> input_buf("input_buf", cfg.PE_COLS, buffer_depth);
    input_buf.clk(clk);
    input_buf.rst_n(rst_n);
    input_buf.wr_en(ib_wr_en);
    input_buf.wr_ready(ib_wr_ready);
    input_buf.rd_en(ib_rd_en);
    input_buf.rd_valid(col_input_valid); 

    // Output buffer implementation, 512-bit width => int64_t width relies on PE_ROWS
    PingPongBuffer<int64_t> output_buf("output_buf", cfg.PE_ROWS, buffer_depth);
    output_buf.clk(clk);
    output_buf.rst_n(rst_n);
    output_buf.wr_en(ob_wr_en);
    output_buf.wr_ready(ob_wr_ready);
    output_buf.rd_en(ob_rd_en);
    output_buf.rd_valid(ob_rd_valid);
    
    for (int i = 0; i < total_pes; i++) {
        dut.weight_flat[i](weight_flat[i]);
    }
    for (int i = 0; i < cfg.PE_COLS; i++) {
        input_buf.wr_data[i](ib_wr_data[i]);
        input_buf.rd_data[i](col_input[i]);
        dut.col_input[i](col_input[i]);
    }
    for (int i = 0; i < cfg.PE_ROWS; i++) {
        dut.row_output[i](row_output[i]);
        output_buf.wr_data[i](row_output[i]);
        output_buf.rd_data[i](ob_rd_data[i]);
    }

    sc_core::sc_trace_file *tf = sc_core::sc_create_vcd_trace_file("wave");
    tf->set_time_unit(1, sc_core::SC_NS);
    // ==== Systolic Array top-level ====
    sc_core::sc_trace(tf, clk, "clk");
    sc_core::sc_trace(tf, rst_n, "rst_n");
    sc_core::sc_trace(tf, weight_load, "weight_load");
    sc_core::sc_trace(tf, weight_switch, "weight_switch");
    sc_core::sc_trace(tf, col_input_valid, "col_input_valid");
    sc_core::sc_trace(tf, tile_done, "tile_done");
    
    // ==== Input Buffer ====
    sc_core::sc_trace(tf, ib_wr_en, "ib_wr_en");
    sc_core::sc_trace(tf, ib_wr_ready, "ib_wr_ready");
    sc_core::sc_trace(tf, ib_rd_en, "ib_rd_en");
    for (int i = 0; i < cfg.PE_COLS; i++) {
        sc_core::sc_trace(tf, ib_wr_data[i], "IB_WR_DATA_" + std::to_string(i));
        sc_core::sc_trace(tf, col_input[i], "IB_RD_DATA_cols_" + std::to_string(i));
    }
    
    // ==== Output Buffer ====
    sc_core::sc_trace(tf, ob_wr_en, "ob_wr_en");
    sc_core::sc_trace(tf, ob_wr_ready, "ob_wr_ready");
    sc_core::sc_trace(tf, ob_rd_valid, "ob_rd_valid");
    for (int i = 0; i < cfg.PE_ROWS; i++) {
        sc_core::sc_trace(tf, row_output[i], "OB_WR_DATA_rows_" + std::to_string(i));
        sc_core::sc_trace(tf, ob_rd_data[i], "OB_RD_DATA_" + std::to_string(i));
    }

    // ==== Weight Buffers ====
    for (int c = 0; c < cfg.PE_COLS; c++) {
        sc_core::sc_trace(tf, wb_wr_en[c], "wb_wr_en_" + std::to_string(c));
        sc_core::sc_trace(tf, wb_rd_en[c], "wb_rd_en_" + std::to_string(c));
        sc_core::sc_trace(tf, wb_rd_valid[c], "wb_rd_valid_" + std::to_string(c));
    }
    for (int i = 0; i < total_pes; i++) {
        sc_core::sc_trace(tf, wb_wr_data[i], "WB_WR_DATA_" + std::to_string(i));
        sc_core::sc_trace(tf, weight_flat[i], "WB_RD_DATA_pes_" + std::to_string(i));
    }

    // Initialize
    rst_n.write(0);
    weight_switch.write(0);
    for(int c=0; c<cfg.PE_COLS; c++) {
        wb_wr_en[c].write(0);
        wb_rd_en[c].write(0);
    }
    ib_wr_en.write(0);
    ib_rd_en.write(0);
    ob_wr_en.write(0);
    ob_rd_en.write(0);
    for (int i = 0; i < total_pes; i++) wb_wr_data[i].write(0);
    for (int i = 0; i < cfg.PE_COLS; i++) ib_wr_data[i].write(0);

    // Provide a dummy module reading wb_rd_valid to drive weight_load
    // Instead of module, we'll manually drive weight_load in the testbench loop for the moment,
    // or just wire weight_load to wb_rd_valid[0] because they are symmetric.
    // wait, weight_load is just connected via testbench below.

    sc_core::sc_start(15, sc_core::SC_NS);
    rst_n.write(1);
    sc_core::sc_start(10, sc_core::SC_NS);

    // ============================================
    // 1. DMA loads weights into Weight Buffers (Fill Ping full depth)
    // ============================================
    // "数据就是1、2、3这种比较小的，不一样的"
    for(int c=0; c<cfg.PE_COLS; c++) wb_wr_en[c].write(1);
    
    for (int feed = 0; feed < buffer_depth; ++feed) {
        for (int r = 0; r < cfg.PE_ROWS; r++) {
            for (int c = 0; c < cfg.PE_COLS; c++) {
                int val = (feed % 3) + 1 + r + c; // small different values like 1, 2, 3...
                wb_wr_data[r * cfg.PE_COLS + c].write(val);
            }
        }
        sc_core::sc_start(10, sc_core::SC_NS);
    }
    for(int c=0; c<cfg.PE_COLS; c++) wb_wr_en[c].write(0);
    sc_core::sc_start(10, sc_core::SC_NS);

    // Consumer reads weights once? Wait, weight buffer provides one vector for the PE array to load.
    // But weight load in systolic array is 1 cycle per layer right now.
    // We just pop one row from ping buffer since it's full now.
    for(int c=0; c<cfg.PE_COLS; c++) wb_rd_en[c].write(1);
    sc_core::sc_start(10, sc_core::SC_NS);
    for(int c=0; c<cfg.PE_COLS; c++) wb_rd_en[c].write(0);
    
    // Drive weight_load to systolic array
    weight_load.write(1);
    sc_core::sc_start(10, sc_core::SC_NS);
    weight_load.write(0);

    // Switch shadow to active to enable computing
    weight_switch.write(1);
    sc_core::sc_start(10, sc_core::SC_NS); 
    weight_switch.write(0);

    // ============================================
    // 2. DMA loads inputs into Input Buffer (Fill Ping full depth)
    // ============================================
    dut.set_num_input_rows(buffer_depth);
    
    ib_wr_en.write(1);
    for (int feed = 0; feed < buffer_depth; ++feed) {
        for (int c = 0; c < cfg.PE_COLS; c++) {
            int val = (feed % 2) + 1 + c; // small numbers like 1, 2...
            ib_wr_data[c].write(val);
        }
        sc_core::sc_start(10, sc_core::SC_NS);
    }
    ib_wr_en.write(0);
    // Give memory 1 cycle to settle to pong bank
    sc_core::sc_start(10, sc_core::SC_NS);

    // ============================================
    // 3. Systolic Array consumes from Input Buffer & Output buffers collect
    // ============================================
    for (int i = 0; i < input_depth; ++i) {
        ib_rd_en.write(1);
        sc_core::sc_start(10, sc_core::SC_NS);
    }
    ib_rd_en.write(0);

    // Run until done
    while (!tile_done.read()) {
        // Collect into output buffer simply to show it is wired up
        ob_wr_en.write(1); // Writing every cycle just as an example since there is no out_valid yet
        sc_core::sc_start(10, sc_core::SC_NS);
    }
    ob_wr_en.write(0);
    
    // extra cycles to capture the end wave
    sc_core::sc_start(50, sc_core::SC_NS);

    sc_core::sc_close_vcd_trace_file(tf);

    std::cout << "Simulation completed. Waveform saved to wave.vcd" << std::endl;
    
    for (int c = 0; c < cfg.PE_COLS; c++) {
        delete weight_bufs[c];
    }
    return 0;
}
