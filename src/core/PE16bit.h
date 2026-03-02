#pragma once

#include <systemc>
#include "Config.h"

namespace SystolicSim
{
    SC_MODULE(PE16bit)
    {
    public:
        sc_core::sc_in<bool> clk;
        sc_core::sc_in<bool> rst_n;
        sc_core::sc_in<int16_t> weight_in;
        sc_core::sc_in<bool> weight_load;
        sc_core::sc_in<int16_t> input_in;
        sc_core::sc_in<int64_t> psum_in;
        sc_core::sc_in<bool> weight_switch;

        sc_core::sc_out<int64_t> psum_out;

        SC_CTOR(PE16bit) : weight_reg_(0), shadow_weight_reg_(0), psum_reg_(0)
        {
            SC_METHOD(do_tick);
            sensitive << clk.pos();
        }

    private:
        int16_t weight_reg_;
        int16_t shadow_weight_reg_;
        int64_t psum_reg_;

        void do_tick()
        {
            // 复位阶段
            if (!rst_n.read())
            {
                weight_reg_ = 0;
                shadow_weight_reg_ = 0;
                psum_reg_ = 0;
                psum_out.write(0);
                return;
            }

            // 权重背景加载（完全并列）
            if (weight_load.read())
            {
                shadow_weight_reg_ = weight_in.read();
            }

            // 权重瞬间切换（与加载不冲突，由外部主控统一在这个周期触发）
            if (weight_switch.read())
            {
                weight_reg_ = shadow_weight_reg_;
            }

            // 常规计算路径（不再有 else if 被阻挡的问题了）
            int16_t act = input_in.read();
            int64_t psum = psum_in.read();

            int64_t prod = static_cast<int64_t>(act) * static_cast<int64_t>(weight_reg_);
            int64_t result = psum + prod;

            if (result > 2147483647LL)
                result = 2147483647LL;
            if (result < -2147483648LL)
                result = -2147483648LL;

            psum_reg_ = result;
            psum_out.write(psum_reg_);

            //
        }
    };
}