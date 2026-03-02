#pragma once

#include <systemc>
#include <queue>
#include <iostream>
#include <unordered_map>

// Ramulator 2 headers
#include "base/base.h"
#include "base/config.h"
#include "memory_system/memory_system.h"
#include "base/request.h"
#include "frontend/frontend.h"

namespace SystolicSim {

// 作为挂接代理的 Dummy FrontEnd，不需特殊逻辑
class DummyFrontEnd : public Ramulator::IFrontEnd {
public:
    void connect_memory_system(Ramulator::IMemorySystem* memory_system) override {
        m_memory_system = memory_system;
    }
    bool is_finished() override { return false; }
    void tick() override {}
};

/**
 * @brief RamulatorWrapper: Ramulator 2 与 SystemC 的桥梁
 * 
 * 真正的 512-bit 通道实现。该模块不仅转发了时序事件给 Ramulator，而且模拟了内存
 * 数据存储 (Backing Store), 在收到读写请求时能够存取真实的 512-bit 数据。
 */
SC_MODULE(RamulatorWrapper) {
public:
    sc_core::sc_in<bool> clk;
    sc_core::sc_in<bool> rst_n;

    // --- 读接口 (Read Request Interface) ---
    sc_core::sc_in<bool>      rd_en;         // 读请求有效
    sc_core::sc_in<uint64_t>  rd_addr;       // 读地址 (应按 64 byte = 512 bit 对齐)
    sc_core::sc_out<bool>     rd_ready;      // 高电平表示接口不忙，可以接收读请求
    
    // --- 读响应用于取回数据 (Read Response Interface) ---
    sc_core::sc_out<bool>     rd_resp_valid; // 【关键】由于 DDR 有延迟，当它变为1时说明下面的数据到了
    sc_core::sc_out<uint64_t> rd_resp_addr;  // 已取回数据对应的物理地址
    sc_core::sc_out<sc_dt::sc_biguint<512>> rd_resp_data; // 实打实的 512-bit 读数据位宽！

    // --- 写接口 (Write Request Interface) ---
    sc_core::sc_in<bool>      wr_en;         // 写请求有效
    sc_core::sc_in<uint64_t>  wr_addr;       // 写地址
    sc_core::sc_in<sc_dt::sc_biguint<512>>  wr_data; // 512-bit 的写入数据位宽
    sc_core::sc_out<bool>     wr_ready;      // 高电平表示接口可以接收写请求
    
    // --- 写响应接口 (Write Response Interface) ---
    sc_core::sc_out<bool>     wr_resp_valid; // 写入请求被真实地执行完了
    sc_core::sc_out<uint64_t> wr_resp_addr;

    SC_HAS_PROCESS(RamulatorWrapper);

    /**
     * @param name SystemC Module 名称
     * @param config_path DDR 配置的 yaml 路径
     */
    RamulatorWrapper(sc_core::sc_module_name name, const std::string& config_path) 
        : sc_core::sc_module(name) {
        
        std::cout << "[RamulatorWrapper] Creating wrapper, name: " << name << std::endl;
        rd_ready.initialize(false);
        wr_ready.initialize(false);
        rd_resp_valid.initialize(false);
        wr_resp_valid.initialize(false);
        rd_resp_addr.initialize(0);
        wr_resp_addr.initialize(0);
        rd_resp_data.initialize(0);

        std::cout << "[RamulatorWrapper] Parse YAML config from: " << config_path << std::endl;
        // 1. 读取 YAML 配置文件
        YAML::Node config = Ramulator::Config::parse_config_file(config_path, {});
        
        std::cout << "[RamulatorWrapper] Create DummyFrontEnd" << std::endl;
        // 2. 建立 FrontEnd & MemorySystem
        frontend_ = new DummyFrontEnd();
        
        std::cout << "[RamulatorWrapper] Create MemorySystem via Factory" << std::endl;
        memory_system_ = Ramulator::Factory::create_memory_system(config);

        std::cout << "[RamulatorWrapper] Connect components" << std::endl;
        // 3. 让它们互相连接
        frontend_->connect_memory_system(memory_system_);
        memory_system_->connect_frontend(frontend_);

        std::cout << "[RamulatorWrapper] Setup SystemC method" << std::endl;
        SC_METHOD(on_clock);
        sensitive << clk.pos();
        dont_initialize();
        std::cout << "[RamulatorWrapper] Wrapper initialization complete" << std::endl;
    }

    ~RamulatorWrapper() {
        // We do not auto finalize or delete here to avoid Segfaults at program termination.
        // User should call `finalize()` manually before simulator exits.
        // if (frontend_) delete frontend_; 
    }

    void finalize() {
        if (frontend_) frontend_->finalize();
        if (memory_system_) memory_system_->finalize();
    }

private:
    DummyFrontEnd* frontend_ = nullptr;
    Ramulator::IMemorySystem* memory_system_ = nullptr;

    // ----- Backing Store 机制 -----
    // 用于真实保存 512-bit 的数据。因为 Ramulator 只算时长不存数据。
    std::unordered_map<uint64_t, sc_dt::sc_biguint<512>> backing_store_;

    struct ReqInfo {
        uint64_t addr;
        bool is_write;
        sc_dt::sc_biguint<512> data;
    };
    std::queue<ReqInfo> req_buffer_;       // 缓存暂未发给 DDR 的请求
    const size_t MAX_BUF_SIZE = 32; 

    // 这些负责将“回调发来的完成事件”输送到 RTL 总线的输出口
    struct ReadResp {
        uint64_t addr;
        sc_dt::sc_biguint<512> data;
    };
    std::queue<ReadResp> read_resp_queue_;
    std::queue<uint64_t> write_resp_queue_;

    void on_clock() {
        if (!rst_n.read()) {
            rd_ready.write(false);
            wr_ready.write(false);
            rd_resp_valid.write(false);
            wr_resp_valid.write(false);
            
            while(!req_buffer_.empty()) req_buffer_.pop();
            while(!read_resp_queue_.empty()) read_resp_queue_.pop();
            while(!write_resp_queue_.empty()) write_resp_queue_.pop();
            return;
        }

        // ========== 1. 从 SystemC 线上“嗅探”外接设备产生的读写请求 ==========
        if (rd_en.read() && req_buffer_.size() < MAX_BUF_SIZE) {
            req_buffer_.push({rd_addr.read(), false, 0});
        }
        if (wr_en.read() && req_buffer_.size() < MAX_BUF_SIZE) {
            req_buffer_.push({wr_addr.read(), true, wr_data.read()});
        }

        // ========== 2. 向 Ramulator 抛出时序模拟请求 ==========
        if (!req_buffer_.empty()) {
            auto& top = req_buffer_.front();
            bool accepted = false;
            
            if (top.is_write) {
                // 收到写请求时，立即把数据存在 Backing Store 哈希表里
                backing_store_[top.addr] = top.data;

                // 回调函数：当在周期级精确的模拟后真正写完时，它会被激发
                auto wr_cb = [this](Ramulator::Request& req) {
                    this->write_resp_queue_.push(req.addr);
                };
                Ramulator::Request req(top.addr, Ramulator::Request::Type::Write, 0, wr_cb);
                accepted = memory_system_->send(req);
            } else {
                // 读请求：准备在未来的某个周期通过回调拿到数据
                auto rd_cb = [this](Ramulator::Request& req) {
                    sc_dt::sc_biguint<512> fetched_data = 0;
                    if (this->backing_store_.find(req.addr) != this->backing_store_.end()) {
                        fetched_data = this->backing_store_[req.addr];
                    } else {
                        // 如果读的是我们没写过的地方（例如默认用0补齐），警告（但没关系可正常运行）
                        // std::cerr << "[Warning] Read uninitialized addr: 0x" << std::hex << req.addr << std::dec << std::endl;
                    }
                    this->read_resp_queue_.push({req.addr, fetched_data});
                };
                Ramulator::Request req(top.addr, Ramulator::Request::Type::Read, 0, rd_cb);
                accepted = memory_system_->send(req);
            }

            if (accepted) {
                req_buffer_.pop();
            }
        }

        // ========== 3. 对内存系统进行时钟打拍 ============
        frontend_->tick();
        memory_system_->tick();

        // ========== 4. 把积攒的“已完成”数据以 RTL 协议发向输出总线 ===========
        if (!read_resp_queue_.empty()) {
            rd_resp_valid.write(true);
            rd_resp_addr.write(read_resp_queue_.front().addr);
            rd_resp_data.write(read_resp_queue_.front().data); 
            read_resp_queue_.pop();
        } else {
            rd_resp_valid.write(false);
        }

        if (!write_resp_queue_.empty()) {
            wr_resp_valid.write(true);
            wr_resp_addr.write(write_resp_queue_.front());
            write_resp_queue_.pop();
        } else {
            wr_resp_valid.write(false);
        }

        // ========== 5. 更新就绪握手信号 ===========
        bool can_accept = (req_buffer_.size() < MAX_BUF_SIZE);
        rd_ready.write(can_accept);
        wr_ready.write(can_accept);
    }
};

} // namespace SystolicSim
