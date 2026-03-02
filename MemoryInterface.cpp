#include "MemoryInterface.h"
#include <iostream>
#include <cmath>

#ifdef USE_RAMULATOR
#include "base/base.h"
#include "base/request.h"
#include "base/factory.h"
#include "frontend/frontend.h"
#include "memory_system/memory_system.h"
using namespace Ramulator;

class SimpleInterface : public IFrontEnd, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, SimpleInterface, "SimpleInterface",
        "Simple external interface for SystolicSim.")
public:
    void init() override {}
    void tick() override {}
    bool is_finished() override { return true; }
    bool receive_external_requests(int type, Addr_t addr, int src,
                                   std::function<void(Request&)> cb) override {
        Request req(addr, type, src, cb);
        return m_memory_system->send(req);
    }
};
#endif

namespace SystolicSim {

// ─────────────────────────────────────────────────────────────
MemoryInterface::MemoryInterface(sc_core::sc_module_name name,
                                 const Config& cfg, Stats& stats)
    : sc_core::sc_module(name)
    , cfg_(cfg), stats_(stats)
    , mode_(cfg.mem_mode)
{
    compute_params();

    SC_METHOD(on_clk);
    sensitive << clk.pos();
}

MemoryInterface::~MemoryInterface() { finalize(); }

// ─────────────────────────────────────────────────────────────
void MemoryInterface::compute_params() {
    double hz          = cfg_.PE_FREQ_GHZ * 1e9;
    bytes_per_cycle_   = cfg_.SIMPLE_BW_GBPS * 1e9 / hz;
    rd_lat_cycles_     = static_cast<uint64_t>(
        std::ceil(cfg_.SIMPLE_READ_LATENCY_NS  * cfg_.PE_FREQ_GHZ));
    wr_lat_cycles_     = static_cast<uint64_t>(
        std::ceil(cfg_.SIMPLE_WRITE_LATENCY_NS * cfg_.PE_FREQ_GHZ));
}

// ─────────────────────────────────────────────────────────────
bool MemoryInterface::initialize(const std::string& config_path) {
    const char* mstr =
        (mode_ == Config::MemMode::NONE)      ? "NONE (zero-latency)" :
        (mode_ == Config::MemMode::RAMULATOR)  ? "RAMULATOR" : "SIMPLE";
    std::cout << "[MemCtrl] Mode: " << mstr << "\n";

    if (mode_ == Config::MemMode::NONE || mode_ == Config::MemMode::SIMPLE) {
        std::cout << "[MemCtrl] BW=" << cfg_.SIMPLE_BW_GBPS << " GB/s"
                  << "  RdLat=" << rd_lat_cycles_ << " cyc"
                  << "  WrLat=" << wr_lat_cycles_ << " cyc\n";
        return true;
    }
#ifdef USE_RAMULATOR
    try {
        YAML::Node ycfg = Ramulator::Config::parse_config_file(config_path, {});
        frontend_ = Factory::create_frontend(ycfg);
        mem_sys_  = Factory::create_memory_system(ycfg);
        if (!frontend_ || !mem_sys_) throw std::runtime_error("factory failed");
        frontend_->connect_memory_system(mem_sys_);
        mem_sys_->connect_frontend(frontend_);
        std::cout << "[MemCtrl] Ramulator init: " << config_path << "\n";
        return true;
    } catch(const std::exception& e) {
        std::cerr << "[MemCtrl] Ramulator failed: " << e.what() << " → SIMPLE\n";
        mode_ = Config::MemMode::SIMPLE;
        return true;
    }
#else
    std::cerr << "[MemCtrl] RAMULATOR not compiled in, falling back to SIMPLE\n";
    mode_ = Config::MemMode::SIMPLE;
    return true;
#endif
}

// ─────────────────────────────────────────────────────────────
void MemoryInterface::finalize() {
#ifdef USE_RAMULATOR
    if (frontend_) { frontend_->finalize(); frontend_ = nullptr; }
    if (mem_sys_)  { mem_sys_->finalize();  mem_sys_  = nullptr; }
#endif
}

// ─────────────────────────────────────────────────────────────
// SC_METHOD — called every clk.pos() by SystemC scheduler
// ─────────────────────────────────────────────────────────────
void MemoryInterface::on_clk() {
    // Default outputs low
    rd_done.write(false);
    wr_done.write(false);

    if (!rst_n.read()) {
        while (!rd_q_.empty()) rd_q_.pop();
        while (!wr_q_.empty()) wr_q_.pop();
        pending_      = 0;
        next_bus_free_= 0;
        prev_rd_req_  = false;
        prev_wr_req_  = false;
        busy.write(false);
        return;
    }

    cycle_++;

    // ── Rising-edge detect: enqueue new requests ──────────────
    bool cur_rd = rd_req.read();
    bool cur_wr = wr_req.read();

    if (cur_rd && !prev_rd_req_) {
        uint64_t addr = static_cast<uint64_t>(rd_addr.read());
        size_t   sz   = static_cast<size_t>(rd_size.read());
        stats_.total_memory_reads++;
        stats_.total_bytes_read += sz;

        if (mode_ == Config::MemMode::NONE) {
            rd_q_.push({ false, cycle_ + 1 });
            pending_++;
        } else if (mode_ == Config::MemMode::SIMPLE) {
            uint64_t xfer = static_cast<uint64_t>(
                std::ceil(static_cast<double>(sz) / bytes_per_cycle_));
            uint64_t done_at = std::max(cycle_ + rd_lat_cycles_ + xfer, next_bus_free_);
            next_bus_free_   = done_at + xfer;
            rd_q_.push({ false, done_at });
            pending_++;
        }
#ifdef USE_RAMULATOR
        else {
            auto tr = std::make_shared<BurstTracker>();
            tr->total    = (sz + static_cast<size_t>(cfg_.BURST_SIZE) - 1)
                         / static_cast<size_t>(cfg_.BURST_SIZE);
            tr->req_type = Request::Type::Read;
            tr->is_write = false;
            for (size_t off = 0; off < sz; off += static_cast<size_t>(cfg_.BURST_SIZE))
                tr->addrs.push(addr + off);
            trackers_.push_back(tr);
            pending_++;
        }
#else
        (void)addr;
#endif
    }

    if (cur_wr && !prev_wr_req_) {
        uint64_t addr = static_cast<uint64_t>(wr_addr.read());
        size_t   sz   = static_cast<size_t>(wr_size.read());
        stats_.total_memory_writes++;
        stats_.total_bytes_written += sz;

        if (mode_ == Config::MemMode::NONE) {
            wr_q_.push({ true, cycle_ + 1 });
            pending_++;
        } else if (mode_ == Config::MemMode::SIMPLE) {
            uint64_t xfer    = static_cast<uint64_t>(
                std::ceil(static_cast<double>(sz) / bytes_per_cycle_));
            uint64_t done_at = std::max(cycle_ + wr_lat_cycles_ + xfer, next_bus_free_);
            next_bus_free_   = done_at + xfer;
            wr_q_.push({ true, done_at });
            pending_++;
        }
#ifdef USE_RAMULATOR
        else {
            auto tr = std::make_shared<BurstTracker>();
            tr->total    = (sz + static_cast<size_t>(cfg_.BURST_SIZE) - 1)
                         / static_cast<size_t>(cfg_.BURST_SIZE);
            tr->req_type = Request::Type::Write;
            tr->is_write = true;
            for (size_t off = 0; off < sz; off += static_cast<size_t>(cfg_.BURST_SIZE))
                tr->addrs.push(addr + off);
            trackers_.push_back(tr);
            pending_++;
        }
#else
        (void)addr;
#endif
    }

    prev_rd_req_ = cur_rd;
    prev_wr_req_ = cur_wr;

    // ── NONE / SIMPLE: check queue completions ─────────────────
    if (mode_ == Config::MemMode::NONE || mode_ == Config::MemMode::SIMPLE) {
        if (!rd_q_.empty() && rd_q_.front().complete_cycle <= cycle_) {
            rd_done.write(true);
            rd_q_.pop();
            if (pending_ > 0) pending_--;
        }
        if (!wr_q_.empty() && wr_q_.front().complete_cycle <= cycle_) {
            wr_done.write(true);
            wr_q_.pop();
            if (pending_ > 0) pending_--;
        }
    }

    // ── RAMULATOR ─────────────────────────────────────────────
#ifdef USE_RAMULATOR
    if (mode_ == Config::MemMode::RAMULATOR) {
        process_ramulator_bursts();
        if (frontend_) frontend_->tick();
        if (mem_sys_)  mem_sys_->tick();
    }
#endif

    busy.write(pending_ > 0);
}

// ─────────────────────────────────────────────────────────────
#ifdef USE_RAMULATOR
void MemoryInterface::process_ramulator_bursts() {
    int sent = 0;
    for (auto& tr : trackers_) {
        while (!tr->addrs.empty() && sent < MAX_BURST_PER_TICK) {
            uint64_t ba = tr->addrs.front();
            auto on_done = [this, tr](Request&) {
                tr->done++;
                if (tr->done >= tr->total) {
                    if (tr->is_write) wr_done.write(true);
                    else              rd_done.write(true);
                }
            };
            bool ok = frontend_->receive_external_requests(
                tr->req_type, ba, 0, on_done);
            if (ok) {
                tr->addrs.pop(); tr->sent++; sent++;
                if (tr->is_write) {
                    tr->done++;
                    if (tr->done >= tr->total) wr_done.write(true);
                }
            } else break;
        }
        if (sent >= MAX_BURST_PER_TICK) break;
    }
    trackers_.remove_if([this](const std::shared_ptr<BurstTracker>& t) {
        bool done = t->done >= t->total;
        if (done && pending_ > 0) pending_--;
        return done;
    });
}
#endif

} // namespace SystolicSim
