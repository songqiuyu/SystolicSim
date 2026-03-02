#pragma once

/**
 * Top.h — Top-level SC_MODULE (全 SystemC 信号化版本)
 *
 * 实例化并连接所有子模块：
 *   SystolicArray  — PE 网格
 *   PingPongBuffer — ibuf / wbuf / obuf  (独立乒乓缓冲)
 *   AccumBuffer    — 片上 int32 累加器
 *   MemoryInterface — DDR 后端 SC_MODULE
 *   Scheduler      — 控制 FSM SC_THREAD
 *
 * 所有控制信号均用 sc_signal<> 连接，可被 GTKWave 波形观测。
 */

#include <systemc>
#include "Config.h"
#include "Stats.h"
#include "PE.h"
#include "SystolicArray.h"
#include "Buffer.h"
#include "MemoryInterface.h"
#include "Scheduler.h"

namespace SystolicSim {

SC_MODULE(Top) {
    // ── Primary ports ──────────────────────────────────────────
    sc_core::sc_in<bool>  clk;
    sc_core::sc_in<bool>  rst_n;
    sc_core::sc_in<bool>  start;
    sc_core::sc_out<bool> done;

    // ── Sub-modules ────────────────────────────────────────────
    SystolicArray   array;
    PingPongBuffer  ibuf;   // Input tile buffer
    PingPongBuffer  wbuf;   // Weight tile buffer
    PingPongBuffer  obuf;   // Output tile buffer
    AccumBuffer     abuf;   // On-chip int32 accumulator
    MemoryInterface memctrl;
    Scheduler       sched;

    // ================================================================
    // Internal signals
    // ================================================================

    // ── Array ↔ Top ───────────────────────────────────────────
    sc_core::sc_vector<sc_core::sc_signal<Config::DataType>>  sig_wt_flat;
    sc_core::sc_signal<bool>                                   sig_wt_load;
    sc_core::sc_vector<sc_core::sc_signal<Config::DataType>>  sig_row_in;
    sc_core::sc_signal<bool>                                   sig_row_valid;
    sc_core::sc_vector<sc_core::sc_signal<Config::AccumType>> sig_col_out;
    sc_core::sc_signal<bool>                                   sig_tile_done;

    // ── MemoryInterface control ────────────────────────────────
    sc_core::sc_signal<bool>                   sig_rd_req;
    sc_core::sc_signal<sc_dt::sc_uint<64>>     sig_rd_addr;
    sc_core::sc_signal<int>                    sig_rd_size;
    sc_core::sc_signal<bool>                   sig_rd_done;
    sc_core::sc_signal<bool>                   sig_wr_req;
    sc_core::sc_signal<sc_dt::sc_uint<64>>     sig_wr_addr;
    sc_core::sc_signal<int>                    sig_wr_size;
    sc_core::sc_signal<bool>                   sig_wr_done;
    sc_core::sc_signal<bool>                   sig_mem_busy;

    // ── InputBuffer handshake ──────────────────────────────────
    sc_core::sc_signal<bool>  sig_ibuf_wr_req;
    sc_core::sc_signal<bool>  sig_ibuf_wr_ready;
    sc_core::sc_signal<bool>  sig_ibuf_rd_valid;
    sc_core::sc_signal<bool>  sig_ibuf_rd_ack;

    // ── WeightBuffer handshake ─────────────────────────────────
    sc_core::sc_signal<bool>  sig_wbuf_wr_req;
    sc_core::sc_signal<bool>  sig_wbuf_wr_ready;
    sc_core::sc_signal<bool>  sig_wbuf_rd_valid;
    sc_core::sc_signal<bool>  sig_wbuf_rd_ack;

    // ── OutputBuffer handshake ─────────────────────────────────
    sc_core::sc_signal<bool>  sig_obuf_wr_req;
    sc_core::sc_signal<bool>  sig_obuf_wr_ready;
    sc_core::sc_signal<bool>  sig_obuf_rd_valid;
    sc_core::sc_signal<bool>  sig_obuf_rd_ack;

    // ── AccumBuffer control ────────────────────────────────────
    sc_core::sc_signal<bool>              sig_abuf_clear;
    sc_core::sc_signal<bool>              sig_abuf_acc_valid;
    sc_core::sc_signal<int>               sig_abuf_acc_idx;
    sc_core::sc_signal<Config::AccumType> sig_abuf_acc_val;
    sc_core::sc_signal<bool>              sig_abuf_wb_req;
    sc_core::sc_signal<int>               sig_abuf_wb_count;
    sc_core::sc_signal<bool>              sig_abuf_wb_done;

    // ================================================================
    SC_HAS_PROCESS(Top);

    Top(sc_core::sc_module_name name,
        const Config& cfg,
        Stats& stats)
        : sc_core::sc_module(name)
        // Sub-modules
        , array  ("array",   cfg)
        , ibuf   ("ibuf",    static_cast<size_t>(cfg.INPUT_TILE_BYTES()))
        , wbuf   ("wbuf",    static_cast<size_t>(cfg.WEIGHT_TILE_BYTES()))
        , obuf   ("obuf",    static_cast<size_t>(cfg.OUTPUT_TILE_BYTES()))
        , abuf   ("abuf",    static_cast<size_t>(cfg.ACCUM_TILE_ELEMENTS()))
        , memctrl("memctrl", cfg, stats)
        , sched  ("sched",   cfg, array, memctrl, ibuf, wbuf, obuf, abuf, stats)
        // sc_vector signals
        , sig_wt_flat("sig_wt_flat", cfg.PE_ROWS * cfg.PE_COLS)
        , sig_row_in ("sig_row_in",  cfg.PE_ROWS)
        , sig_col_out("sig_col_out", cfg.PE_COLS)
    {
        // ── SystolicArray ─────────────────────────────────────
        array.clk(clk);  array.rst_n(rst_n);
        array.weight_flat(sig_wt_flat);
        array.weight_load(sig_wt_load);
        array.row_input(sig_row_in);
        array.row_input_valid(sig_row_valid);
        array.col_output(sig_col_out);
        array.tile_done(sig_tile_done);

        // ── MemoryInterface ───────────────────────────────────
        memctrl.clk(clk);  memctrl.rst_n(rst_n);
        memctrl.rd_req(sig_rd_req);  memctrl.rd_addr(sig_rd_addr);
        memctrl.rd_size(sig_rd_size); memctrl.rd_done(sig_rd_done);
        memctrl.wr_req(sig_wr_req);  memctrl.wr_addr(sig_wr_addr);
        memctrl.wr_size(sig_wr_size); memctrl.wr_done(sig_wr_done);
        memctrl.busy(sig_mem_busy);

        // ── InputBuffer ───────────────────────────────────────
        ibuf.clk(clk);  ibuf.rst_n(rst_n);
        ibuf.wr_req(sig_ibuf_wr_req);   ibuf.wr_ready(sig_ibuf_wr_ready);
        ibuf.rd_ack(sig_ibuf_rd_ack);   ibuf.rd_valid(sig_ibuf_rd_valid);

        // ── WeightBuffer ──────────────────────────────────────
        wbuf.clk(clk);  wbuf.rst_n(rst_n);
        wbuf.wr_req(sig_wbuf_wr_req);   wbuf.wr_ready(sig_wbuf_wr_ready);
        wbuf.rd_ack(sig_wbuf_rd_ack);   wbuf.rd_valid(sig_wbuf_rd_valid);

        // ── OutputBuffer ──────────────────────────────────────
        obuf.clk(clk);  obuf.rst_n(rst_n);
        obuf.wr_req(sig_obuf_wr_req);   obuf.wr_ready(sig_obuf_wr_ready);
        obuf.rd_ack(sig_obuf_rd_ack);   obuf.rd_valid(sig_obuf_rd_valid);

        // ── AccumBuffer ───────────────────────────────────────
        abuf.clk(clk);  abuf.rst_n(rst_n);
        abuf.clear_req(sig_abuf_clear);
        abuf.acc_valid(sig_abuf_acc_valid);
        abuf.acc_idx(sig_abuf_acc_idx);
        abuf.acc_val(sig_abuf_acc_val);
        abuf.wb_req(sig_abuf_wb_req);
        abuf.wb_count(sig_abuf_wb_count);
        abuf.wb_done(sig_abuf_wb_done);

        // ── Scheduler ─────────────────────────────────────────
        sched.clk(clk);  sched.rst_n(rst_n);
        sched.start(start);  sched.done(done);

        // Mem channel
        sched.mem_rd_req(sig_rd_req);   sched.mem_rd_addr(sig_rd_addr);
        sched.mem_rd_size(sig_rd_size); sched.mem_rd_done(sig_rd_done);
        sched.mem_wr_req(sig_wr_req);   sched.mem_wr_addr(sig_wr_addr);
        sched.mem_wr_size(sig_wr_size); sched.mem_wr_done(sig_wr_done);

        // Buffer handshakes
        sched.ibuf_wr_req(sig_ibuf_wr_req);   sched.ibuf_wr_ready(sig_ibuf_wr_ready);
        sched.ibuf_rd_valid(sig_ibuf_rd_valid); sched.ibuf_rd_ack(sig_ibuf_rd_ack);

        sched.wbuf_wr_req(sig_wbuf_wr_req);   sched.wbuf_wr_ready(sig_wbuf_wr_ready);
        sched.wbuf_rd_valid(sig_wbuf_rd_valid); sched.wbuf_rd_ack(sig_wbuf_rd_ack);

        sched.obuf_wr_req(sig_obuf_wr_req);   sched.obuf_wr_ready(sig_obuf_wr_ready);
        sched.obuf_rd_valid(sig_obuf_rd_valid); sched.obuf_rd_ack(sig_obuf_rd_ack);

        // AccumBuffer
        sched.abuf_clear(sig_abuf_clear);
        sched.abuf_acc_valid(sig_abuf_acc_valid);
        sched.abuf_acc_idx(sig_abuf_acc_idx);
        sched.abuf_acc_val(sig_abuf_acc_val);
        sched.abuf_wb_req(sig_abuf_wb_req);
        sched.abuf_wb_count(sig_abuf_wb_count);
        sched.abuf_wb_done(sig_abuf_wb_done);

        sched.tile_done(sig_tile_done);

        // 注入 SystolicArray 信号指针（Scheduler 直接驱动 PE 信号）
        sched.connect_array_signals(
            &sig_wt_flat, &sig_wt_load, &sig_row_in,
            &sig_row_valid, &sig_col_out, &sig_tile_done);
    }

    void setup_gemm(uint64_t in_addr, uint64_t wt_addr, uint64_t out_addr) {
        sched.setup_gemm(in_addr, wt_addr, out_addr);
    }

    bool initialize_memory(const std::string& config_path) {
        return memctrl.initialize(config_path);
    }

    void finalize() {
        memctrl.finalize();
    }
};

} // namespace SystolicSim
