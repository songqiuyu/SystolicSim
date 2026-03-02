#include <systemc>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "memory/ramulator_wrapper.h"

using namespace SystolicSim;

SC_MODULE(Testbench) {
    sc_core::sc_in<bool> clk;
    sc_core::sc_out<bool> rst_n;

    // Write interface
    sc_core::sc_out<bool> wr_en;
    sc_core::sc_out<uint64_t> wr_addr;
    sc_core::sc_out<sc_dt::sc_biguint<512>> wr_data;
    sc_core::sc_in<bool> wr_ready;
    sc_core::sc_in<bool> wr_resp_valid;
    sc_core::sc_in<uint64_t> wr_resp_addr;

    // Read interface
    sc_core::sc_out<bool> rd_en;
    sc_core::sc_out<uint64_t> rd_addr;
    sc_core::sc_in<bool> rd_ready;
    sc_core::sc_in<bool> rd_resp_valid;
    sc_core::sc_in<uint64_t> rd_resp_addr;
    sc_core::sc_in<sc_dt::sc_biguint<512>> rd_resp_data;

    sc_core::sc_out<bool> test_done;

    std::ofstream log_file;

    SC_HAS_PROCESS(Testbench);

    Testbench(sc_core::sc_module_name name) : sc_core::sc_module(name) {
        log_file.open("ddr_test_results.log");
        if (!log_file.is_open()) {
            std::cerr << "Failed to open ddr_test_results.log for writing." << std::endl;
        }

        SC_THREAD(run_test);
        sensitive << clk.pos();
        
        SC_METHOD(monitor_read);
        sensitive << clk.pos();
        dont_initialize();
        
        SC_METHOD(monitor_write);
        sensitive << clk.pos();
        dont_initialize();
    }

    ~Testbench() {
        if (log_file.is_open()) {
            log_file.close();
        }
    }

    void run_test() {
        // Init
        rst_n.write(0);
        wr_en.write(0);
        rd_en.write(0);
        test_done.write(0);
        wait(20, sc_core::SC_NS); // wait 2 cycles
        rst_n.write(1);
        wait(20, sc_core::SC_NS);

        log_file << "==================== STARTING DDR TEST ====================" << std::endl;

        const int BURST_LENGTH = 16;
        uint64_t base_addr = 0x8000; // DDR Base Address Mapped

        // ==========================================
        // Test 1: Burst Write (512-bit / 64 bytes per beat)
        // ==========================================
        log_file << ">>> Initiating Burst Write sequence..." << std::endl;
        for (int i = 0; i < BURST_LENGTH; ++i) {
            uint64_t current_addr = base_addr + (i * 64); // Address must inc by 64 bytes for 512 bits
            sc_dt::sc_biguint<512> current_data = i + 1; // Arbitrary sequential data
            
            // To make the data look visually distinct we populate the upper bits too 
            sc_dt::sc_biguint<512> pattern = 0xABCD0000;
            current_data = (pattern << 64) | current_data;

            while (!wr_ready.read()) {
                wait();
            }
            log_file << "@" << sc_core::sc_time_stamp() << " [Testbench] Sending WRITE Req to address: 0x" 
                     << std::hex << current_addr << " Data: " << current_data.to_string(sc_dt::SC_HEX) 
                     << std::dec << std::endl;
            
            wr_addr.write(current_addr);
            wr_data.write(current_data);
            wr_en.write(1);
            wait();
            wr_en.write(0);
            
            // Allow some cycles between requests, similar to AXI bursting
            if (i % 4 == 3) wait(10, sc_core::SC_NS); 
        }

        log_file << ">>> Waiting for Burst Writes to soak in DDR array..." << std::endl;
        wait(500, sc_core::SC_NS);

        // ==========================================
        // Test 2: Burst Read matching the sequence 
        // ==========================================
        log_file << "<<< Initiating Burst Read sequence..." << std::endl;
        for (int i = 0; i < BURST_LENGTH; ++i) {
            uint64_t current_addr = base_addr + (i * 64);

            while (!rd_ready.read()) {
                wait();
            }
            log_file << "@" << sc_core::sc_time_stamp() << " [Testbench] Sending READ Req to address: 0x" 
                     << std::hex << current_addr << std::dec << std::endl;
            
            rd_addr.write(current_addr);
            rd_en.write(1);
            wait();
            rd_en.write(0);
        }

        // Wait for all responses to come back. DDR read latency can be hundreds of cycles.
        log_file << "--- Waiting for DDR responses to be traced back ---" << std::endl;
        wait(20000, sc_core::SC_NS);

        log_file << "==================== FINISHED DDR TEST ====================" << std::endl;
        test_done.write(1);
    }

    void monitor_read() {
        if (rst_n.read() && rd_resp_valid.read()) {
            log_file << "@" << sc_core::sc_time_stamp() << " [Testbench] <<<<< RECEIVED READ Response from DDR." 
                     << " Addr: 0x" << std::hex << rd_resp_addr.read() 
                     << " | Data: " << rd_resp_data.read().to_string(sc_dt::SC_HEX) << std::dec << std::endl;
            
            // Echo to console to show progress
            std::cout << "@" << sc_core::sc_time_stamp() << " [Testbench] Read Resp | Addr: 0x" 
                      << std::hex << rd_resp_addr.read() << std::dec << std::endl;
        }
    }

    void monitor_write() {
        if (rst_n.read() && wr_resp_valid.read()) {
            log_file << "@" << sc_core::sc_time_stamp() << " [Testbench] >>>>> RECEIVED WRITE Completion." 
                     << " Addr: 0x" << std::hex << wr_resp_addr.read() << std::dec << std::endl;
        }
    }
};

int sc_main(int argc, char* argv[]) {
    sc_core::sc_clock clk("clk", 10, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n("rst_n");

    sc_core::sc_signal<bool> wr_en("wr_en"), rd_en("rd_en");
    sc_core::sc_signal<uint64_t> wr_addr("wr_addr"), rd_addr("rd_addr");
    sc_core::sc_signal<sc_dt::sc_biguint<512>> wr_data("wr_data");
    
    sc_core::sc_signal<bool> wr_ready("wr_ready"), rd_ready("rd_ready");
    sc_core::sc_signal<bool> wr_resp_valid("wr_resp_valid"), rd_resp_valid("rd_resp_valid");
    sc_core::sc_signal<uint64_t> wr_resp_addr("wr_resp_addr"), rd_resp_addr("rd_resp_addr");
    sc_core::sc_signal<sc_dt::sc_biguint<512>> rd_resp_data("rd_resp_data");

    sc_core::sc_signal<bool> test_done("test_done");

    // 注意：如果是从 build 目录执行，yaml 路径要是相对 build 的。
    // 如果想要更鲁棒，可以作为 argv 传进来。我们先硬编码为相对于 build
    RamulatorWrapper ddr("ddr3_mem", "../SystolicSim/config/ddr3_config.yaml");
    ddr.clk(clk);
    ddr.rst_n(rst_n);

    ddr.wr_en(wr_en);
    ddr.wr_addr(wr_addr);
    ddr.wr_data(wr_data);
    ddr.wr_ready(wr_ready);
    ddr.wr_resp_valid(wr_resp_valid);
    ddr.wr_resp_addr(wr_resp_addr);

    ddr.rd_en(rd_en);
    ddr.rd_addr(rd_addr);
    ddr.rd_ready(rd_ready);
    ddr.rd_resp_valid(rd_resp_valid);
    ddr.rd_resp_addr(rd_resp_addr);
    ddr.rd_resp_data(rd_resp_data);

    Testbench tb("tb");
    tb.clk(clk);
    tb.rst_n(rst_n);
    
    tb.wr_en(wr_en);
    tb.wr_addr(wr_addr);
    tb.wr_data(wr_data);
    tb.wr_ready(wr_ready);
    tb.wr_resp_valid(wr_resp_valid);
    tb.wr_resp_addr(wr_resp_addr);

    tb.rd_en(rd_en);
    tb.rd_addr(rd_addr);
    tb.rd_ready(rd_ready);
    tb.rd_resp_valid(rd_resp_valid);
    tb.rd_resp_addr(rd_resp_addr);
    tb.rd_resp_data(rd_resp_data);

    tb.test_done(test_done);

    std::cout << "Starting Simulation for Ramulator DDR Wrapper..." << std::endl;
    // Run
    while (!test_done.read()) {
        sc_core::sc_start(10, sc_core::SC_NS);
        // Timeout protection
        if (sc_core::sc_time_stamp() > sc_core::sc_time(50000, sc_core::SC_NS)) {
            std::cout << "Simulation timeout! Deadlock or memory response taking too long." << std::endl;
            break;
        }
    }
    
    std::cout << "@" << sc_core::sc_time_stamp() << " Simulation Completed." << std::endl;
    
    // Explicitly finalize the DDR to dump and print the memory statistics 
    // before the destructors kick in and segfault the program environment.
    ddr.finalize();

    return 0;
}
