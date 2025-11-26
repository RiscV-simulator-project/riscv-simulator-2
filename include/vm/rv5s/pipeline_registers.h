#ifndef PIPELINE_REGISTERS_H
#define PIPELINE_REGISTERS_H

#include <cstdint>
#include "vm/rv5s/rv5s_control_unit.h" 
#include "vm/alu.h"

// Data passed from Instruction Fetch to Instruction Decode
struct IF_ID_Register {
    uint32_t instruction = 0; // The fetched instruction
    uint64_t pc = 0;          // The PC of this instruction
    bool valid = false;       // Is the data in this register valid?
};

// Data passed from Instruction Decode to Execute
struct ID_EX_Register {
    uint64_t pc = 0;
    uint64_t link_address = 0;     // For JAL/JALR, this holds PC+4
    uint64_t rs1_val = 0;
    uint64_t rs2_val = 0;
    uint64_t rs3_val = 0;          // FIX: Added for FMA operations
    uint64_t imm = 0;
    uint8_t rs1_idx = 0;
    uint8_t rs2_idx = 0;
    uint8_t rs3_idx = 0;           // FIX: Added for FMA operations
    uint8_t rd_idx = 0;
    uint8_t funct3 = 0;            // Used for memory access type and branch conditions
    uint32_t instruction = 0;      // Pass instruction for opcode checks
    alu::AluOp alu_operation = alu::AluOp::kNone; // ALU operation for EX stage

    // Flags for floating-point and rs3 validity
    bool is_fp_instr = false;      // Is this a floating-point instruction?
    bool has_rs3 = false;          // Does this instruction use rs3?
    bool rs1_is_fp = false;
    bool rs2_is_fp = false;
    bool rs3_is_fp = false;

    // --- Branch prediction information ---
    // Indicates whether this instruction has branch prediction info.
    // For non-branch instructions this will remain false.
    bool has_predicted = false;
    // If has_predicted is true, this tells whether the decoder predicted
    // the branch to be taken. Unconditional jumps set this to true as well.
    bool predicted_taken = false;
    // If the decoder predicted a branch or jump, this holds the predicted
    // target address. For JALR, the decoder computes the address using
    // the rs1 register value and the immediate. For conditional branches it
    // equals PC+imm. For non-branch instructions this field is ignored.
    uint64_t predicted_target = 0;

    // Control signals generated in the ID stage
    ControlSignals control;
    bool valid = false;
};

// Data passed from Execute to Memory
struct EX_MEM_Register {
    uint64_t pc = 0;               // Pass PC for debugging/tracing
    uint64_t link_address = 0;     // Pass link address forward
    uint64_t alu_result = 0;
    uint64_t rs2_val = 0;          // Needed for store instructions
    uint8_t rd_idx = 0;
    uint8_t funct3 = 0;            // Used for memory access type
    bool alu_zero = false;         // Result of a comparison was zero
    uint32_t instruction = 0;      // Pass instruction for FP check in WB
    alu::AluOp alu_operation = alu::AluOp::kNone; // FIX: Initialize with default value

    // Flag for floating-point instructions
    bool is_fp_instr = false;      // Is this a floating-point instruction?

    // Control signals passed through
    ControlSignals control;
    bool valid = false;
};

// Data passed from Memory to Write-Back
struct MEM_WB_Register {
    uint64_t pc = 0;               // Pass PC for debugging/tracing
    uint64_t link_address = 0;     // Pass link address to final stage
    uint64_t mem_read_data = 0;    // Data read from memory
    uint64_t alu_result = 0;
    uint8_t rd_idx = 0;
    uint32_t instruction = 0;      // Pass instruction for FP check in WB

    // FIX: Added flag for floating-point instructions
    bool is_fp_instr = false;      // Is this a floating-point instruction?

    // Control signals passed through
    ControlSignals control;
    bool valid = false;
};

#endif // PIPELINE_REGISTERS_H
