#pragma once
#include <systemc>
#include <vector>
#include <string>
#include "PE16bit.h"


namespace SystolicSim {
    SC_MODULE(systolicArray_ws) {
        const int                                       rows_;
        const int                                       cols_;

        sc_core::sc_in<bool>                            clk;
        sc_core::sc_in<bool>                            rst_n;
        sc_core::sc_in<bool>                            weight_load;

        sc_core::sc_vector<sc_core::sc_in<int16_t>>     weight_flat;
        sc_core::sc_vector<sc_core::sc_in<int16_t>>     row_input;
        sc_core::sc_in<bool>                            row_input_valid;

        sc_core::sc_vector<sc_core::sc_out<int64_t>>    col_output;
        sc_core::sc_out<bool>                           tile_done;


        



    }
}