#ifndef PIPELINE_REGISTERS_H
#define PIPELINE_REGISTERS_H

#include <cstdint>
#include "vm/rv5s/rv5s_control_unit.h" 

// Data passed from Instruction Fetch to Instruction Decode
struct IF_ID_Register {
    uint32_t instruction = 0; // The fetched instruction
    uint64_t pc = 0;          // The PC of this instruction
    bool valid = false;       // Is the data in this register valid?
};

// Data passed from Instruction Decode to Execute
struct ID_EX_Register {
    uint64_t pc = 0;
    uint64_t link_address = 0; // For JAL/JALR, this holds PC+4
    uint64_t rs1_val = 0;
    uint64_t rs2_val = 0;
    uint64_t imm = 0;
    uint8_t rs1_idx = 0;
    uint8_t rs2_idx = 0;
    uint8_t rd_idx = 0;
    uint8_t funct3 = 0; // Added: Used for memory access type and branch conditions
    
    // Control signals generated in the ID stage
    ControlSignals control;
    bool valid = false;
};

// Data passed from Execute to Memory
struct EX_MEM_Register {
    uint64_t pc = 0; // Pass PC for debugging/tracing
    uint64_t link_address = 0; // Pass link address forward
    uint64_t alu_result = 0;
    uint64_t rs2_val = 0; // Needed for store instructions
    uint8_t rd_idx = 0;
    uint8_t funct3 = 0; // Added: Used for memory access type
    bool alu_zero = false; // Result of a comparison was zero

    // Control signals passed through
    ControlSignals control;
    bool valid = false;
};

// Data passed from Memory to Write-Back
struct MEM_WB_Register {
    uint64_t pc = 0; // Pass PC for debugging/tracing
    uint64_t link_address = 0; // Pass link address to final stage
    uint64_t mem_read_data = 0; // Data read from memory
    uint64_t alu_result = 0;
    uint8_t rd_idx = 0;

    // Control signals passed through
    ControlSignals control;
    bool valid = false;
};

#endif 
