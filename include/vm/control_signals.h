#ifndef CONTROL_SIGNALS_H
#define CONTROL_SIGNALS_H

#include <cstdint>

struct ControlSignals {
    bool reg_write = false;
    bool branch = false;
    bool alu_src = false;
    bool mem_read = false;
    bool mem_write = false;
    bool mem_to_reg = false;
    bool pc_src = false;
    uint8_t alu_op = 0;
};

#endif 