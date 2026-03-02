#pragma once

/**
 * Scheduler.h — SC_MODULE FSM controller (全信号化版本)
 *
 * 不再直接调用 Buffer/MemoryInterface 的 C++ 方法进行数据搬运。
 * 所有控制通过 sc_signal 端口握手完成；数据指针在握手前通过
 * set_rd_dst / set_wr_src 抽象地传给 MemoryInterface。
 *
 * 连接关系（在 Top 中绑定）:
 *   Scheduler → MemoryInterface : rd_req/rd_addr/rd_size/rd_done
 *                                  wr_req/wr_addr/wr_size/wr_done
 *   Scheduler → InputBuf        : wr_req / wr_ready  (Sched 作为 DMA写端)
 *   InputBuf  → SystolicArray   : rd_valid  (Array 从 buffer 读数)
 *   Scheduler → WeightBuf       : 同上
 *   Scheduler → OutputBuf       : rd_ack (告知 output buf 可以写回)
 *   Scheduler → AccumBuf        : clear/acc_valid/acc_idx/acc_val/wb_req/wb_done
 */

#include <systemc>
#include <vector>
#include <cstring>
#include <iostream>
#include "Config.h"
#include "Stats.h"
#include "Buffer.h"
#include "MemoryInterface.h"
#include "SystolicArray.h"

namespace SystolicSim {

SC_MODULE(Scheduler) {
    // ── Global ports ───────────────────────────────────────────
    sc_core::sc_in<bool>  clk;
    sc_core::sc_in<bool>  rst_n;
    sc_core::sc_in<bool>  start;
    sc_core::sc_out<bool> done;

    // ── MemoryInterface control ports ──────────────────────────
    // Read channel
    sc_core::sc_out<bool>                   mem_rd_req;
    sc_core::sc_out<sc_dt::sc_uint<64>>     mem_rd_addr;
    sc_core::sc_out<int>                    mem_rd_size;
    sc_core::sc_in<bool>                    mem_rd_done;
    // Write channel
    sc_core::sc_out<bool>                   mem_wr_req;
    sc_core::sc_out<sc_dt::sc_uint<64>>     mem_wr_addr;
    sc_core::sc_out<int>                    mem_wr_size;
    sc_core::sc_in<bool>                    mem_wr_done;

    // ── InputBuffer ports (Scheduler = Producer) ───────────────
    sc_core::sc_out<bool> ibuf_wr_req;
    sc_core::sc_in<bool>  ibuf_wr_ready;
    sc_core::sc_in<bool>  ibuf_rd_valid;   // 监听：array 可以消费
    sc_core::sc_out<bool> ibuf_rd_ack;     // (通常由 array 驱动，这里备用)

    // ── WeightBuffer ports ─────────────────────────────────────
    sc_core::sc_out<bool> wbuf_wr_req;
    sc_core::sc_in<bool>  wbuf_wr_ready;
    sc_core::sc_in<bool>  wbuf_rd_valid;
    sc_core::sc_out<bool> wbuf_rd_ack;

    // ── OutputBuffer ports (Scheduler = Consumer) ──────────────
    sc_core::sc_in<bool>  obuf_wr_ready;  // output buf 有空槽
    sc_core::sc_out<bool> obuf_wr_req;    // 写回触发
    sc_core::sc_in<bool>  obuf_rd_valid;  // (debug/monitor)
    sc_core::sc_out<bool> obuf_rd_ack;

    // ── AccumBuffer ports ──────────────────────────────────────
    sc_core::sc_out<bool>             abuf_clear;
    sc_core::sc_out<bool>             abuf_acc_valid;
    sc_core::sc_out<int>              abuf_acc_idx;
    sc_core::sc_out<Config::AccumType>abuf_acc_val;
    sc_core::sc_out<bool>             abuf_wb_req;
    sc_core::sc_out<int>              abuf_wb_count;
    sc_core::sc_in<bool>              abuf_wb_done;

    // ── SystolicArray control ports (reuse Top-level signals) ──
    sc_core::sc_in<bool>              tile_done;

    // ── Constructor ────────────────────────────────────────────
    SC_HAS_PROCESS(Scheduler);

    Scheduler(sc_core::sc_module_name name,
              const Config& cfg,
              SystolicArray& array,
              MemoryInterface& memctrl,
              PingPongBuffer& ibuf,
              PingPongBuffer& wbuf,
              PingPongBuffer& obuf,
              AccumBuffer&    abuf,
              Stats& stats)
        : sc_core::sc_module(name)
        , cfg_(cfg), array_(array), memctrl_(memctrl)
        , ibuf_(ibuf), wbuf_(wbuf), obuf_(obuf), abuf_(abuf)
        , stats_(stats)
        , col_psum_(cfg.PE_COLS, 0)
    {
        SC_THREAD(run_fsm);
        sensitive << clk.pos();
    }

    void setup_gemm(uint64_t in_addr, uint64_t wt_addr, uint64_t out_addr) {
        input_base_  = in_addr;
        weight_base_ = wt_addr;
        output_base_ = out_addr;
    }

    // Signal pointers to SystolicArray (set by Top)
    void connect_array_signals(
        sc_core::sc_vector<sc_core::sc_signal<Config::DataType>>*  wt_sigs,
        sc_core::sc_signal<bool>*                                   wt_load,
        sc_core::sc_vector<sc_core::sc_signal<Config::DataType>>*  row_in,
        sc_core::sc_signal<bool>*                                   row_valid,
        sc_core::sc_vector<sc_core::sc_signal<Config::AccumType>>* col_out,
        sc_core::sc_signal<bool>*                                   t_done)
    {
        wt_sigs_    = wt_sigs;
        wt_load_    = wt_load;
        row_in_     = row_in;
        row_valid_  = row_valid;
        col_out_    = col_out;
        tile_done_  = t_done;
    }

private:
    const Config&    cfg_;
    SystolicArray&   array_;
    MemoryInterface& memctrl_;
    PingPongBuffer&  ibuf_, &wbuf_, &obuf_;
    AccumBuffer&     abuf_;
    Stats&           stats_;

    // SystolicArray signal pointers
    sc_core::sc_vector<sc_core::sc_signal<Config::DataType>>*  wt_sigs_   = nullptr;
    sc_core::sc_signal<bool>*                                   wt_load_   = nullptr;
    sc_core::sc_vector<sc_core::sc_signal<Config::DataType>>*  row_in_    = nullptr;
    sc_core::sc_signal<bool>*                                   row_valid_ = nullptr;
    sc_core::sc_vector<sc_core::sc_signal<Config::AccumType>>* col_out_   = nullptr;
    sc_core::sc_signal<bool>*                                   tile_done_ = nullptr;

    uint64_t input_base_  = 0;
    uint64_t weight_base_ = 0;
    uint64_t output_base_ = 0;

    int tiles_computed_ = 0;
    int tiles_written_  = 0;

    std::vector<Config::AccumType> col_psum_;

    // FSM helpers
    void mem_read (uint64_t addr, size_t size, uint8_t* dst);
    void mem_write(uint64_t addr, size_t size, const uint8_t* src);
    void load_input (int m, int k);
    void load_weight(int k, int n);
    void do_compute (int k, int n);
    void do_accumulate(int n);
    void writeback  (int m, int k, int n);
    void run_fsm();
};

} // namespace SystolicSim
