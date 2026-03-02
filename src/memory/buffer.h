#pragma once

#include <systemc>
#include <vector>
#include <cstdint>

namespace SystolicSim {

/**
 * PingPongBuffer
 * 
 * 专门为了对接 Systolic Array 设计的 512-bit (默认 32 x 16-bit) 乒乓缓冲区。
 * 它保证了“一个 Bank 在被运算模块（Consumer）读取流水线的同时，另一个 Bank 可以被
 * 存储/DMA模块（Producer）写入”。
 * 
 * 特点:
 * 1. WIDTH 控制位宽，默认 32 个 16-bit 元素（等于 512 bit）。
 * 2. 深度 depth 由构造函数传入，表示一个 Bank 能存多少行 512-bit 数据。
 * 3. 读写互相独立，遵循 RTL 握手协议 (wr_en/wr_ready, rd_en/rd_valid)。
 */
template <typename T = int16_t>
struct PingPongBuffer : public sc_core::sc_module {
public:
    // ── 全局信号 ───────────────────────────────────────────────
    sc_core::sc_in<bool> clk;
    sc_core::sc_in<bool> rst_n;

    // ── 生产者端 (Producer: DMA/Memory -> Buffer) ──────────────
    sc_core::sc_in<bool> wr_en;                             // 写请求
    sc_core::sc_vector<sc_core::sc_in<T>> wr_data;    // 写入数据输入
    sc_core::sc_out<bool> wr_ready;                         // 高电平表示当前写入 Bank 未满，可以接收数据

    // ── 消费者端 (Consumer: Buffer -> Systolic Array) ──────────
    sc_core::sc_in<bool> rd_en;                             // 读请求
    sc_core::sc_vector<sc_core::sc_out<T>> rd_data;   // 读出数据输出
    sc_core::sc_out<bool> rd_valid;                         // 高电平表示当前读取 Bank 已经准备好被读取

    SC_HAS_PROCESS(PingPongBuffer);

    PingPongBuffer(sc_core::sc_module_name name, int width, int depth)
        : sc_core::sc_module(name),
          wr_data("wr_data", width),
          rd_data("rd_data", width),
          width_(width),
          depth_(depth)
    {
        // 初始化两个 Bank，每个 Bank 大小为 depth_ * width_
        ping_bank_.resize(depth_, std::vector<T>(width_, 0));
        pong_bank_.resize(depth_, std::vector<T>(width_, 0));

        SC_METHOD(on_clock);
        sensitive << clk.pos();
    }

private:
    int width_;
    int depth_;
    std::vector<std::vector<T>> ping_bank_;
    std::vector<std::vector<T>> pong_bank_;

    // 内部寄存器
    int  wr_bank_ = 0;  // 0 = Ping, 1 = Pong
    int  rd_bank_ = 0;  // 0 = Ping, 1 = Pong
    int  wr_ptr_  = 0;  // 0 to depth_ - 1
    int  rd_ptr_  = 0;  // 0 to depth_ - 1

    bool ping_full_ = false;
    bool pong_full_ = false;

    void on_clock() {
        if (!rst_n.read()) {
            wr_bank_   = 0;
            rd_bank_   = 0;
            wr_ptr_    = 0;
            rd_ptr_    = 0;
            ping_full_ = false;
            pong_full_ = false;

            wr_ready.write(false);
            rd_valid.write(false);
            for (int i = 0; i < width_; i++) {
                rd_data[i].write(0);
            }
            return;
        }

        // ==========================
        // 1. 消费者逻辑 (Read Path)
        // ==========================
        bool current_rd_full = (rd_bank_ == 0) ? ping_full_ : pong_full_;
        
        if (current_rd_full && rd_en.read()) {
            rd_ptr_++;
            if (rd_ptr_ >= depth_) {
                // 当前 Bank 已经被完全读完了，释放当前 Bank
                rd_ptr_ = 0;
                if (rd_bank_ == 0) ping_full_ = false;
                else               pong_full_ = false;
                
                rd_bank_ ^= 1; // 切换到下一个 Bank
            }
        }

        // ==========================
        // 2. 生产者逻辑 (Write Path)
        // ==========================
        bool current_wr_full = (wr_bank_ == 0) ? ping_full_ : pong_full_;
        
        if (!current_wr_full && wr_en.read()) {
            // 将 512-bit (32 values) 同时写入
            for (int i = 0; i < width_; i++) {
                if (wr_bank_ == 0)
                    ping_bank_[wr_ptr_][i] = wr_data[i].read();
                else
                    pong_bank_[wr_ptr_][i] = wr_data[i].read();
            }
            
            wr_ptr_++;
            if (wr_ptr_ >= depth_) {
                // 当前 Bank 写满了，锁住该 Bank 等待 Consumer 来读
                wr_ptr_ = 0;
                if (wr_bank_ == 0) ping_full_ = true;
                else               pong_full_ = true;
                
                wr_bank_ ^= 1; // 切换到另一个 Bank 继续写
            }
        }

        // ==========================
        // 3. 更新输出端口状态
        // ==========================
        // 注意：这是用最新的状态来更新物理端口，也就是下一个时钟周期的组合逻辑
        bool next_wr_full = (wr_bank_ == 0) ? ping_full_ : pong_full_;
        bool next_rd_full = (rd_bank_ == 0) ? ping_full_ : pong_full_;

        wr_ready.write(!next_wr_full); // 只要写Bank没满，就可以继续准备接收
        rd_valid.write(next_rd_full);  // 只要读Bank里有完整的数据块，就可以提示下游来读

        // 读数据直接拉线到端口，形成组合逻辑的数据透传
        if (next_rd_full) {
            for (int i = 0; i < width_; i++) {
                if (rd_bank_ == 0)
                    rd_data[i].write(ping_bank_[rd_ptr_][i]);
                else
                    rd_data[i].write(pong_bank_[rd_ptr_][i]);
            }
        } else {
            for (int i = 0; i < width_; i++) {
                rd_data[i].write(0);
            }
        }
    }
};

} // namespace SystolicSim
