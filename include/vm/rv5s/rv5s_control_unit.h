#ifndef RV5S_CONTROL_UNIT_H
#define RV5S_CONTROL_UNIT_H

#include "../control_unit_base.h"
#include "../alu.h"
#include "../control_signals.h"
#include <cstdint>

class RV5SControlUnit : public ControlUnit {
public:
    RV5SControlUnit() = default;
    ~RV5SControlUnit() override = default;

    void SetControlSignals(uint32_t instruction) override;

    // Matches base class (uint8_t alu_op).
    alu::AluOp GetAluSignal(uint32_t instruction, uint8_t alu_op) override;

    [[nodiscard]] ControlSignals GetSignals() const {
        // Order matches ControlSignals field order above.
        return {
            reg_write_,   // reg_write
            mem_to_reg_,  // mem_to_reg
            branch_,      // branch
            mem_read_,    // mem_read
            mem_write_,   // mem_write
            alu_src_,     // alu_src
            alu_op_,      // alu_op
            jump_,        // jump
            pc_to_alu_    // pc_to_alu
        };
    }

private:
    // Backing bits populated by SetControlSignals(...)
    bool    reg_write_  = false;
    bool    mem_to_reg_ = false;
    bool    branch_     = false;
    bool    mem_read_   = false;
    bool    mem_write_  = false;
    bool    alu_src_    = false;
    uint8_t alu_op_     = 0;
    bool    jump_       = false;
    bool    pc_to_alu_  = false;
};

#endif // RV5S_CONTROL_UNIT_H
