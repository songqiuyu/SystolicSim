#pragma once

#include <systemc>
#include "Config.h"

namespace SystolicSim
{
    SC_MODULE(PE)
    {
    public:
        sc_core::sc_in<bool> clk;
        sc_core::sc_in<bool> rst_n;
        sc_core::sc_in<int16_t> weight_in;
        sc_core::sc_in<bool> weight_load;
        sc_core::sc_in<int16_t> activate_in;
        sc_core::sc_in<int64_t> psum_in;


        sc_core::sc_out<int16_t> activate_out;
        sc_core::sc_out<int64_t> psum_out;

        SC_CTOR(PE) : wieght_reg_(0), input_reg_(0), psum_reg_(0)
        {
            SC_METHOD(do_tick);
            sensitive << clk.pos();
        }

    private:
        int16_t weight_reg_;
        int16_t activate_reg_;
        int64_t psum_reg_;

        void do_tick()
        {
            // 复位阶段
            if (!rst_n.read())
            {
                weight_reg_ = 0; activate_reg_ = 0; psum_reg_ = 0;
                activate_out.write(0); psum_out.write(0);
            }

            // 如果是权重读取阶段
            if (weight_load.read()) {
                weight_reg_ = weight_in.read();
            } else {
                int16_t act = activate_in.read();
                int64_t psum = psum_in.read();

                int64_t prod = static_cast<int64_t>(act) * static_cast<int64_t>(weight_reg_);
                int64_t result = psum + prod;

                if(result > 2147483647LL) result = 2147483647LL;
                if(result < -2147483648LL) result = -2147483648LL;

                psum_reg_ = result;
                activate_reg_ = act;
                psum_out.write(psum_reg_);
                activate_out.write(activate_reg_);
            }

            // 


        }
    };
}