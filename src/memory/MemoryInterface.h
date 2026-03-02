#pragma once
// MemoryInterface是与外部内存交互的接口，这里实现一个用SystemC的

#include <systemc>


#ifdef USE_RAMULATOR
namespace Ramulator { class IFrontEnd; class IMemorySystem; class Request; }
#endif 


namespace SystolicSim {
    SC_MODULE(MemoryInterface) {
        sc_core::sc_in<bool> clk;
        sc_core::sc_in<bool> rst_n;


        // READ CHANNEL
        sc_core::sc_in<bool>                rd_req;         // 读请求
        sc_core::sc_in<sc_dt::sc_uint<64>>  rd_addr;        // 64位地址
        sc_core::sc_in<int>                 rd_size;        // 读尺寸
        sc_core::sc_out<bool>               rd_done;        // 读结束

        // WRITE CHANNEL
        sc_core::sc_in<bool>                wr_req;         // 写请求
        sc_core::sc_in<sc_dt::sc_uint<64>>  wr_addr;        // 写地址
        sc_core::sc_in<int>                 wr_size;        // 写尺寸
        sc_core::sc_out<bool>               wr_done;        // 写结束

        // STATUS
        sc_core::sc_out<bool>               busy;           // 

        // Constructor
        SC_HAS_PROCESS(MemoryInterface);

        MemoryInterface(sc_core::sc_module_name name,
                        const Config& cfg,      
                        Status& stats);
        ~MemoryInterface();

        void set_rd_dst(uint8_t*       p)  { rd_dst_ = p; }
        void set_wr_src(const uint8_t* p)  { wr_src_ = p; }

        bool initialize(const std::string& config_path = "");
        void finalize();

        Config::MemMode mode() const { return mode_; }
        uint64_t        cycle() const { return cycle_; }

    private:
        const Config&   cfg_;
        Stats&          stats_;
        Config::MemMode mode_;
        uint64_t        cycle_  = 0;

        uint8_t*        rd_dst_ = nullptr;
        const uint8_t*  wr_src_ = nullptr;

        bool prev_rd_req_ = false;
        bool prev_wr_req_ = false;

        struct PendReq { bool is_write; uint64_t complete_cycle; };
        std::queue<PendReq> rd_q_, wr_q_;
        uint64_t  next_bus_free_   = 0;
        double    bytes_per_cycle_ = 0.0;
        uint64_t  rd_lat_cycles_   = 0;
        uint64_t  wr_lat_cycles_   = 0;
        size_t    pending_         = 0;

        void compute_params();

    #ifdef USE_RAMULATOR


    #endif
    
    }
}
