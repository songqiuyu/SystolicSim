#pragma once

/**
 * Buffer.h — SystemC SC_MODULE 版本的片上 Buffer
 *
 * 两个模块：
 *   PingPongBuffer — 双缓冲，Producer(DMA) 和 Consumer(Scheduler) 乒乓交替
 *   AccumBuffer    — 片上 int32 累加器，多轮 K-tile 累加后饱和截位写回
 *
 * 设计原则：
 *   - 控制路径（握手信号）用 sc_in/sc_out 端口，参与 SystemC 仿真时序
 *   - 数据路径（大块 tile 数据）用内部 vector + 方法指针访问
 *     （可视为 abstract datapath / word-level TLM）
 *   - 所有状态转换在时钟上升沿触发（SC_METHOD pos()）
 */

#include <systemc>
#include <vector>
#include <cstring>
#include <cstdint>
#include <iostream>
#include "Config.h"

namespace SystolicSim {


SC_MODULE(PingPongBuffer) {

    // ── Ports ─────────────────────────────────────────────────
    // Ping-Pong Buffer的结构，时钟和复位
    sc_core::sc_in<bool>  clk;
    sc_core::sc_in<bool>  rst_n;

    // 生产者（DMA/MemoryInterface）侧
    sc_core::sc_in<bool>  wr_req;    // 生产者：我要写入
    sc_core::sc_out<bool> wr_ready;  // 缓冲区：有空槽可接收

    // 消费者（Scheduler/Compute）侧
    sc_core::sc_in<bool>  rd_ack;    // 消费者：我已读完，释放这块
    sc_core::sc_out<bool> rd_valid;  // 缓冲区：有数据可读

    // 构造器（接收 runtime 容量）
    SC_HAS_PROCESS(PingPongBuffer);

    PingPongBuffer(sc_core::sc_module_name name, size_t capacity_bytes)
        : sc_core::sc_module(name)
        , cap_(capacity_bytes)
    {
        ping_.resize(cap_, 0);      // 定义双缓冲的容量
        pong_.resize(cap_, 0);      // 定义双缓冲的容量

        SC_METHOD(update_status);   // 上升沿执行
        sensitive << clk.pos();
    }

    // ── 数据访问方法（生产者调用）────────────────────────────
    // 返回当前写入槽指针（仅在 wr_ready 为高时有效）
    uint8_t* get_write_ptr() {
        return (wi_ == 0) ? ping_.data() : pong_.data();
    }

    // 生产者填完数据后调用：标记满，切换写指针
    // 对应硬件：DMA burst 结束的握手完成信号
    void commit_write() {
        if (wi_ == 0) ping_full_ = true;
        else          pong_full_ = true;
        wi_ ^= 1;
    }

    // ── 数据访问方法（消费者调用）────────────────────────────
    // 返回当前读取槽指针（仅在 rd_valid 为高时有效）
    const uint8_t* get_read_ptr() const {
        return (ri_ == 0) ? ping_.data() : pong_.data();
    }

    // 消费者读完后调用：清空该槽，切换读指针
    // 对应 rd_ack 握手被 update_status 捕获后的等效操作
    void commit_read() {
        if (ri_ == 0) ping_full_ = false;
        else          pong_full_ = false;
        ri_ ^= 1;
    }

    size_t capacity() const { return cap_; }

    void reset_state() {
        ping_full_ = pong_full_ = false;
        ri_ = wi_ = 0;
        std::fill(ping_.begin(), ping_.end(), 0u);
        std::fill(pong_.begin(), pong_.end(), 0u);
        wr_ready.write(true);
        rd_valid.write(false);
    }

private:
    size_t               cap_;
    std::vector<uint8_t> ping_, pong_;
    bool ping_full_ = false, pong_full_ = false;
    int  ri_ = 0,  wi_ = 0;    // read / write 使用哪个槽 (0=ping, 1=pong)

    // ── SC_METHOD：每个时钟沿更新握手输出 ──────────────────────
    void update_status() {
        if (!rst_n.read()) {
            ping_full_ = pong_full_ = false;
            ri_ = wi_ = 0;
            wr_ready.write(true);
            rd_valid.write(false);
            return;
        }

        // 生产者握手：req && ready → 提交写入（下一拍生效）
        bool cur_wr_slot_full = (wi_ == 0) ? ping_full_ : pong_full_;
        if (wr_req.read() && !cur_wr_slot_full) {
            // 外部 DMA 在 get_write_ptr() 位置已完成填数
            // 此处仅负责翻转 full 标记和切换写指针
            commit_write();
        }

        // 消费者握手：rd_ack 为高 → 消费完成，释放读槽
        bool cur_rd_slot_full = (ri_ == 0) ? ping_full_ : pong_full_;
        if (rd_ack.read() && cur_rd_slot_full) {
            commit_read();
        }

        // 更新输出端口
        bool write_slot_available = !((wi_ == 0) ? ping_full_ : pong_full_);
        bool read_data_available  =   (ri_ == 0) ? ping_full_ : pong_full_;

        wr_ready.write(write_slot_available);
        rd_valid.write(read_data_available);
    }
};


// =============================================================================
// AccumBuffer SC_MODULE
//
// 片上 int32 累加器，存储所有 N-tile 的中间结果。
//
// 接口：
//   clear_req  — 清零整块 accum（新 M-tile 开始前）
//   acc_valid  — 当拍有累加请求
//   acc_idx    — 累加目标地址（flatten: n*M_TILE*N_TILE + row*N_TILE + col）
//   acc_val    — 要累加的 int32 值
//   wb_req     — 发起写回：saturate int32 → int16 到 wb_dst 指向的字节 buffer
//   wb_done    — 写回完成（持续高 1 拍）
//
// 注意：wb_dst 是字节指针（同 PingPongBuffer::get_write_ptr()），
//       不走 sc_signal；写回操作在 SC_THREAD 里同步完成。
// =============================================================================

SC_MODULE(AccumBuffer) {

    // ── Ports ─────────────────────────────────────────────────
    sc_core::sc_in<bool>                    clk;
    sc_core::sc_in<bool>                    rst_n;

    // 清零
    sc_core::sc_in<bool>                    clear_req;

    // 每周期一次累加（SC_METHOD 触发）
    sc_core::sc_in<bool>                    acc_valid;
    sc_core::sc_in<int>                     acc_idx;
    sc_core::sc_in<Config::AccumType>       acc_val;

    // 写回触发
    sc_core::sc_in<bool>                    wb_req;     // 请求写回
    sc_core::sc_in<int>                     wb_count;   // 要写回多少个元素
    sc_core::sc_out<bool>                   wb_done;    // 完成脉冲

    // ── 构造器 ────────────────────────────────────────────────
    SC_HAS_PROCESS(AccumBuffer);

    AccumBuffer(sc_core::sc_module_name name, size_t num_elements)
        : sc_core::sc_module(name)
        , num_elements_(num_elements)
    {
        data_.resize(num_elements_, 0);

        SC_METHOD(on_clock);
        sensitive << clk.pos();
    }

    // ── 写回方法：Scheduler 调用，传入 output PingPong 的写指针 ──
    // 在 wb_req 握手完成后，Scheduler 调此方法实际搬数据
    void write_back_to(uint8_t* dst, size_t count) {
        size_t n = (count < num_elements_) ? count : num_elements_;
        for (size_t i = 0; i < n; i++) {
            int64_t v = data_[i];
            if (v >  32767LL) v =  32767LL;
            if (v < -32768LL) v = -32768LL;
            int16_t x = static_cast<int16_t>(v);
            std::memcpy(dst + i * 2, &x, 2);
        }
    }

    // 直接读某个 int32 元素（供 Scheduler 采样 PE 输出后手动积累）
    Config::AccumType read_elem(size_t idx) const {
        return (idx < data_.size()) ? data_[idx] : 0;
    }

    void write_elem(size_t idx, Config::AccumType val) {
        if (idx < data_.size()) data_[idx] = val;
    }

    size_t size() const { return num_elements_; }

private:
    size_t num_elements_;
    std::vector<Config::AccumType> data_;   // int32_t 片上存储
    bool   wb_done_next_ = false;

    // ── SC_METHOD：时钟沿执行累加 / 清零 ──────────────────────
    void on_clock() {
        wb_done.write(false);   // 默认不输出 done

        if (!rst_n.read()) {
            std::fill(data_.begin(), data_.end(), 0);
            return;
        }

        // 1. 清零请求优先级最高
        if (clear_req.read()) {
            std::fill(data_.begin(), data_.end(), 0);
            return;
        }

        // 2. 累加（每个时钟最多一次，与 PE 输出同步）
        if (acc_valid.read()) {
            int    idx = acc_idx.read();
            Config::AccumType val = acc_val.read();

            if (idx >= 0 && static_cast<size_t>(idx) < data_.size()) {
                int64_t sum = static_cast<int64_t>(data_[idx]) + val;
                // int32 饱和保护
                if (sum >  2147483647LL) sum =  2147483647LL;
                if (sum < -2147483648LL) sum = -2147483648LL;
                data_[idx] = static_cast<Config::AccumType>(sum);
            }
        }

        // 3. 写回请求：拉高 wb_done 通知 Scheduler 可以调 write_back_to()
        //    （实际数据搬运由 Scheduler 在下一拍调方法完成）
        if (wb_req.read()) {
            wb_done.write(true);
        }
    }
};

// ── 便捷别名（由 cfg 参数在 Top 构造时确定大小）──────────────────
// 用法示例（在 Top.h 或 sc_main 中）：
//   PingPongBuffer ibuf("ibuf", cfg.INPUT_TILE_BYTES());
//   AccumBuffer    abuf("abuf", cfg.ACCUM_TILE_ELEMENTS());

} // namespace SystolicSim
