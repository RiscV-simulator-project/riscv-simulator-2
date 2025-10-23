#ifndef RV5S_CONTROL_UNIT_H
#define RV5S_CONTROL_UNIT_H

#include "vm/control_unit_base.h"
#include "vm/alu.h"
#include <cstdint>

/**
 * @brief Holds all control signals for the pipeline stages.
 */
struct ControlSignals {
    bool reg_write = false;
    bool mem_to_reg = false;
    bool branch = false;
    bool mem_read = false;
    bool mem_write = false;
    bool alu_src = false;
    uint8_t alu_op = 0;
    bool jump = false; // For JAL/JALR
    bool pc_to_alu = false;
};

/**
 * @brief Control Unit for the 5-stage pipelined RISC-V VM.
 *
 * Decodes instructions and generates the necessary control signals
 * for the pipeline stages.
 */
class RV5SControlUnit : public ControlUnit {
public:
    RV5SControlUnit() = default;
    ~RV5SControlUnit() override = default;

    void SetControlSignals(uint32_t instruction) override;
    alu::AluOp GetAluSignal(uint32_t instruction, bool ALUOp) override;

    /**
     * @brief Returns a struct containing all current control signals.
     * @return A ControlSignals struct.
     */
    [[nodiscard]] ControlSignals GetSignals() const {
        return {reg_write_, mem_to_reg_, branch_, mem_read_, mem_write_, alu_src_, alu_op_, jump_, pc_to_alu_};
    }
};

#endif // RV5S_CONTROL_UNIT_H