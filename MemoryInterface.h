#pragma once

/**
 * MemoryInterface.h — SC_MODULE memory backend
 *
 * Replaces the plain-C++ class with a proper SC_MODULE.
 * tick() is now an SC_METHOD (clk.pos sensitive).
 * Control interface uses sc_in/sc_out ports (rd/wr request+done).
 * Data path remains abstract (pointer via set_rd_dst/set_wr_src).
 *
 * Three backends: NONE / SIMPLE / RAMULATOR (unchanged logic).
 */

#include <systemc>
#include <functional>
#include <queue>
#include <list>
#include <string>
#include <cstdint>
#include "Config.h"
#include "Stats.h"

#ifdef USE_RAMULATOR
namespace Ramulator { class IFrontEnd; class IMemorySystem; class Request; }
#endif

namespace SystolicSim {

SC_MODULE(MemoryInterface) {

    // ── Ports ──────────────────────────────────────────────────
    sc_core::sc_in<bool>  clk;
    sc_core::sc_in<bool>  rst_n;

    // Read channel
    sc_core::sc_in<bool>                   rd_req;   // assert to start read
    sc_core::sc_in<sc_dt::sc_uint<64>>     rd_addr;
    sc_core::sc_in<int>                    rd_size;  // bytes
    sc_core::sc_out<bool>                  rd_done;  // 1-cycle pulse on completion

    // Write channel
    sc_core::sc_in<bool>                   wr_req;
    sc_core::sc_in<sc_dt::sc_uint<64>>     wr_addr;
    sc_core::sc_in<int>                    wr_size;
    sc_core::sc_out<bool>                  wr_done;

    // Status
    sc_core::sc_out<bool>                  busy;     // any requests in flight

    // ── Constructor ────────────────────────────────────────────
    SC_HAS_PROCESS(MemoryInterface);

    MemoryInterface(sc_core::sc_module_name name,
                    const Config& cfg,
                    Stats& stats);
    ~MemoryInterface();

    // ── Abstract data path (called before asserting req) ───────
    // Scheduler sets these pointers so the controller knows where
    // data lands / comes from (not modelled on sc_signal).
    void set_rd_dst(uint8_t*       p)  { rd_dst_ = p; }
    void set_wr_src(const uint8_t* p)  { wr_src_ = p; }

    // ── Initialise (Ramulator config path, no-op for NONE/SIMPLE)
    bool initialize(const std::string& config_path = "");
    void finalize();

    Config::MemMode mode() const { return mode_; }
    uint64_t        cycle() const { return cycle_; }

private:
    const Config&   cfg_;
    Stats&          stats_;
    Config::MemMode mode_;
    uint64_t        cycle_  = 0;

    // Abstract data pointers
    uint8_t*        rd_dst_ = nullptr;
    const uint8_t*  wr_src_ = nullptr;

    // Edge-detect for req signals (enqueue only on rising edge)
    bool prev_rd_req_ = false;
    bool prev_wr_req_ = false;

    // ── SIMPLE / NONE internals ────────────────────────────────
    struct PendReq { bool is_write; uint64_t complete_cycle; };
    std::queue<PendReq> rd_q_, wr_q_;
    uint64_t  next_bus_free_   = 0;
    double    bytes_per_cycle_ = 0.0;
    uint64_t  rd_lat_cycles_   = 0;
    uint64_t  wr_lat_cycles_   = 0;
    size_t    pending_         = 0;

    void compute_params();

    // ── RAMULATOR internals ────────────────────────────────────
#ifdef USE_RAMULATOR
    Ramulator::IFrontEnd*    frontend_   = nullptr;
    Ramulator::IMemorySystem* mem_sys_   = nullptr;
    struct BurstTracker {
        size_t total = 0, sent = 0, done = 0;
        int    req_type = 0;
        bool   is_write = false;
        std::queue<uint64_t> addrs;
    };
    std::list<std::shared_ptr<BurstTracker>> trackers_;
    static constexpr int MAX_BURST_PER_TICK = 1;
    void process_ramulator_bursts();
#endif

    // ── SC_METHOD ──────────────────────────────────────────────
    void on_clk();
};

} // namespace SystolicSim
