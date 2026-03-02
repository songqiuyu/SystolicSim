#include "Scheduler.h"

namespace SystolicSim {

// ─────────────────────────────────────────────────────────────
// mem_read: 发出读请求 → 等待 mem_rd_done 脉冲
// ─────────────────────────────────────────────────────────────
void Scheduler::mem_read(uint64_t addr, size_t size, uint8_t* dst) {
    // 1. 告诉 MemCtrl 往哪里写（抽象数据路径）
    memctrl_.set_rd_dst(dst);

    // 2. 驱动请求端口
    mem_rd_addr.write(static_cast<sc_dt::sc_uint<64>>(addr));
    mem_rd_size.write(static_cast<int>(size));
    mem_rd_req.write(true);

    // 3. 等待完成脉冲（MemCtrl SC_METHOD 上升沿处理）
    do { stats_.total_cycles++; stats_.memory_stall_cycles++; wait(); }
    while (!mem_rd_done.read());

    mem_rd_req.write(false);
}

// ─────────────────────────────────────────────────────────────
// mem_write: 发出写请求 → 等待 mem_wr_done 脉冲
// ─────────────────────────────────────────────────────────────
void Scheduler::mem_write(uint64_t addr, size_t size, const uint8_t* src) {
    memctrl_.set_wr_src(src);

    mem_wr_addr.write(static_cast<sc_dt::sc_uint<64>>(addr));
    mem_wr_size.write(static_cast<int>(size));
    mem_wr_req.write(true);

    do { stats_.total_cycles++; stats_.output_write_cycles++; wait(); }
    while (!mem_wr_done.read());

    mem_wr_req.write(false);
}

// ─────────────────────────────────────────────────────────────
// load_input: DDR → InputBuffer
// ─────────────────────────────────────────────────────────────
void Scheduler::load_input(int m, int k) {
    uint64_t addr = input_base_
        + static_cast<uint64_t>(m) * cfg_.M_TILE * cfg_.K * 2
        + static_cast<uint64_t>(k) * cfg_.K_TILE * 2;

    // 等待 InputBuffer 有空槽
    while (!ibuf_wr_ready.read()) { wait(); }

    // DMA 读：数据直接填入 ibuf 的当前写槽
    mem_read(addr, static_cast<size_t>(cfg_.INPUT_TILE_BYTES()), ibuf_.get_write_ptr());

    // 通知 InputBuffer：槽已填完
    ibuf_wr_req.write(true);
    wait();  // PingPongBuffer SC_METHOD 在下一拍 commit
    ibuf_wr_req.write(false);

    stats_.input_load_cycles++;
}

// ─────────────────────────────────────────────────────────────
// load_weight: DDR → WeightBuffer
// ─────────────────────────────────────────────────────────────
void Scheduler::load_weight(int k, int n) {
    uint64_t addr = weight_base_
        + static_cast<uint64_t>(k) * cfg_.K_TILE * cfg_.N * 2
        + static_cast<uint64_t>(n) * cfg_.N_TILE * 2;

    while (!wbuf_wr_ready.read()) { wait(); }

    mem_read(addr, static_cast<size_t>(cfg_.WEIGHT_TILE_BYTES()), wbuf_.get_write_ptr());

    wbuf_wr_req.write(true);
    wait();
    wbuf_wr_req.write(false);

    stats_.weight_load_cycles++;
}

// ─────────────────────────────────────────────────────────────
// do_compute: 运行脉动阵列一个 tile（广播权重 + 送入数据 + 等待完成）
// ─────────────────────────────────────────────────────────────
void Scheduler::do_compute(int /*k*/, int n) {
    const int rows = cfg_.PE_ROWS;
    const int cols = cfg_.PE_COLS;

    // ── 1. 广播权重（等待 wbuf 有数据）─────────────────────────
    while (!wbuf_rd_valid.read()) { wait(); }

    const int16_t* wt = reinterpret_cast<const int16_t*>(wbuf_.get_read_ptr());
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            (*wt_sigs_)[r * cols + c].write(wt[r * cols + c]);

    wt_load_->write(true);
    wait();
    wt_load_->write(false);

    // 释放 weight buffer 槽
    wbuf_rd_ack.write(true);
    wait();
    wbuf_rd_ack.write(false);

    // ── 2. 等待 ibuf 有数据，送入输入行────────────────────────
    while (!ibuf_rd_valid.read()) { wait(); }

    const int16_t* in = reinterpret_cast<const int16_t*>(ibuf_.get_read_ptr());
    array_.set_num_input_rows(cfg_.M_TILE);

    row_valid_->write(false);
    for (int row = 0; row < cfg_.M_TILE; row++) {
        for (int r = 0; r < rows; r++)
            (*row_in_)[r].write(in[row * cfg_.K_TILE + r]);
        row_valid_->write(true);
        stats_.compute_cycles++;
        stats_.pe_active_cycles++;
        stats_.total_macs += static_cast<uint64_t>(rows) * cols;
        stats_.total_cycles++;
        wait();
    }
    row_valid_->write(false);

    // ── 3. 等待 tile_done ─────────────────────────────────────
    while (!tile_done_->read()) {
        stats_.compute_cycles++;
        stats_.total_cycles++;
        wait();
    }
    stats_.compute_cycles++;
    stats_.total_cycles++;

    // 释放 input buffer 槽
    if (n == cfg_.NUM_N_TILES() - 1) {
        ibuf_rd_ack.write(true);
        wait();
        ibuf_rd_ack.write(false);
    }

    // ── 4. 采样底行输出 ────────────────────────────────────────
    for (int c = 0; c < cols; c++)
        col_psum_[c] = (*col_out_)[c].read();
}

// ─────────────────────────────────────────────────────────────
// do_accumulate: 把本次 tile 的列输出累加进 AccumBuffer
// ─────────────────────────────────────────────────────────────
void Scheduler::do_accumulate(int n) {
    const int cols = cfg_.PE_COLS;
    int off = n * cfg_.M_TILE * cfg_.N_TILE;

    // AccumBuffer SC_METHOD 每拍最多处理一次累加
    for (int row = 0; row < cfg_.M_TILE; row++) {
        for (int col = 0; col < cols; col++) {
            abuf_acc_idx.write(off + row * cfg_.N_TILE + col);
            abuf_acc_val.write(col_psum_[col]);
            abuf_acc_valid.write(true);
            stats_.total_cycles++;
            wait();  // 一拍一次累加（与 AccumBuffer SC_METHOD 同步）
        }
    }
    abuf_acc_valid.write(false);
}

// ─────────────────────────────────────────────────────────────
// writeback: AccumBuffer → OutputBuffer → DDR
// ─────────────────────────────────────────────────────────────
void Scheduler::writeback(int m, int k, int n) {
    if (k != cfg_.NUM_K_TILES() - 1) return;  // 只在 K 循环最后一轮写回

    // ── 1. AccumBuffer 写回到 OutputBuffer ────────────────────
    while (!obuf_wr_ready.read()) { wait(); }  // 等 obuf 有空槽

    abuf_wb_count.write(cfg_.M_TILE * cfg_.N_TILE);
    abuf_wb_req.write(true);
    do { wait(); } while (!abuf_wb_done.read());
    abuf_wb_req.write(false);

    // 实际数据搬运：AccumBuffer → OutputBuffer（抽象数据路径）
    abuf_.write_back_to(obuf_.get_write_ptr(),
                        static_cast<size_t>(cfg_.M_TILE * cfg_.N_TILE));

    // 通知 OutputBuffer：槽已填
    obuf_wr_req.write(true);
    wait();
    obuf_wr_req.write(false);

    // ── 2. OutputBuffer → DDR ─────────────────────────────────
    while (!obuf_rd_valid.read()) { wait(); }

    uint64_t out_addr = output_base_
        + static_cast<uint64_t>(m) * cfg_.M_TILE * cfg_.N * 2
        + static_cast<uint64_t>(n) * cfg_.N_TILE * 2;

    mem_write(out_addr,
              static_cast<size_t>(cfg_.OUTPUT_TILE_BYTES()),
              obuf_.get_read_ptr());

    obuf_rd_ack.write(true);
    wait();
    obuf_rd_ack.write(false);

    tiles_written_++;
    stats_.tiles_processed++;

    // ── 3. 清零 AccumBuffer 供下一轮 ─────────────────────────
    abuf_clear.write(true);
    wait();
    abuf_clear.write(false);
}

// ─────────────────────────────────────────────────────────────
// run_fsm: 主 SC_THREAD
// ─────────────────────────────────────────────────────────────
void Scheduler::run_fsm() {
    done.write(false);
    mem_rd_req.write(false); mem_wr_req.write(false);
    ibuf_wr_req.write(false); ibuf_rd_ack.write(false);
    wbuf_wr_req.write(false); wbuf_rd_ack.write(false);
    obuf_wr_req.write(false); obuf_rd_ack.write(false);
    abuf_clear.write(false); abuf_acc_valid.write(false);
    abuf_wb_req.write(false);

    while (!start.read()) wait();

    std::cout << "\n[Scheduler] GEMM started\n";
    cfg_.print();

    // ── N-major 分块: M → K → N ───────────────────────────────
    for (int m = 0; m < cfg_.NUM_M_TILES(); m++) {
        for (int k = 0; k < cfg_.NUM_K_TILES(); k++) {
            for (int n = 0; n < cfg_.NUM_N_TILES(); n++) {

                // Input(M,K) 在 N 循环第一次才需加载，之后复用
                if (n == 0) {
                    std::cout << "] Load Input m=" << m << " k=" << k << "\n";
                    load_input(m, k);
                }

                std::cout << "[Scheduler] Load Weight k=" << k << " n=" << n << "\n";
                load_weight(k, n);

                std::cout << "[Scheduler] Compute m=" << m << " k=" << k << " n=" << n << "\n";
                do_compute(k, n);

                do_accumulate(n);
                tiles_computed_++;

                writeback(m, k, n);
            }
        }
    }

    std::cout << "\n[Scheduler] Done. "
              << "tiles=" << tiles_computed_
              << " writes=" << tiles_written_ << "\n";
    done.write(true);
}

} // namespace SystolicSim
