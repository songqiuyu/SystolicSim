#pragma once

/**
 * PE.h — SystemC Processing Element (runtime-configured)
 *
 * Data width: 16-bit input/weight (Config::DataType = int16_t)
 *             32-bit partial sum  (Config::AccumType = int32_t)
 * Behaviour is fully determined by port connections; no Config ref needed here
 * because the data types are fixed typedefs.
 */

#include <systemc>
#include "Config.h"

namespace SystolicSim {

SC_MODULE(PE) {
    sc_core::sc_in<bool>               clk;
    sc_core::sc_in<bool>               rst_n;
    sc_core::sc_in<Config::DataType>   weight_in;
    sc_core::sc_in<bool>               weight_load;
    sc_core::sc_in<Config::DataType>   input_in;
    sc_core::sc_in<Config::AccumType>  psum_in;
    sc_core::sc_out<Config::DataType>  input_out;
    sc_core::sc_out<Config::AccumType> psum_out;

    SC_CTOR(PE) : weight_reg_(0), input_reg_(0), psum_reg_(0) {
        SC_METHOD(do_tick);
        sensitive << clk.pos();
    }

    void clear_weight() { weight_reg_ = 0; }

private:
    Config::DataType  weight_reg_;
    Config::DataType  input_reg_;
    Config::AccumType psum_reg_;

    void do_tick() {
        if (!rst_n.read()) {
            weight_reg_ = 0; input_reg_ = 0; psum_reg_ = 0;
            input_out.write(0); psum_out.write(0);
            return;
        }
        if (weight_load.read()) {
            weight_reg_ = weight_in.read();
        } else {
            Config::DataType  in  = input_in.read();
            Config::AccumType ps  = psum_in.read();
            Config::AccumType prod = static_cast<Config::AccumType>(in)
                                   * static_cast<Config::AccumType>(weight_reg_);
            Config::AccumType nps  = ps + prod;
            // Saturate accumulator
            if (nps >  2147483647LL) nps =  2147483647LL;
            if (nps < -2147483648LL) nps = -2147483648LL;
            psum_reg_  = nps;
            input_reg_ = in;
            psum_out.write(psum_reg_);
            input_out.write(input_reg_);
        }
    }
};

} // namespace SystolicSim
