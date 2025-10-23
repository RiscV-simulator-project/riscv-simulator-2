#include "vm/rv5s/rv5s_control_unit.h"
#include "common/instructions.h"
#include "vm/control_unit_base.h"
#include "vm/alu.h"
#include <cstdint>

using namespace instruction_set;

void RV5SControlUnit::SetControlSignals(uint32_t instruction) {
  uint8_t opcode = instruction & 0b1111111;

  // Reset all signals to false
  reg_write_ = false;
  mem_to_reg_ = false;
  branch_ = false;
  mem_read_ = false;
  mem_write_ = false;
  alu_src_ = false;
  alu_op_ = 0;

  if (instruction_set::isFInstruction(instruction) || instruction_set::isDInstruction(instruction)) {
      // Floating point instructions
      switch (opcode) {
          case 0b0000111: // FLW, FLD(load)
              alu_src_ = true;
              mem_to_reg_ = true;
              reg_write_ = true;
              mem_read_ = true;
              break;
          case 0b0100111: // FSW, FSD(store)
              alu_src_ = true;
              mem_write_ = true;
              break;
          case 0b1000011: // FMADD.S/D
          case 0b1000111: // FMSUB.S/D
          case 0b1001011: // FNMSUB.S/D
          case 0b1001111: // FNMADD.S/D
              reg_write_ = true;
              break;
          case 0b1010011: // Float R-Type
              reg_write_ = true;
              break;
      }
  } else {
      // Integer instructions
      switch (opcode) {
          case get_instr_encoding(Instruction::kLui).opcode: // LUI
              reg_write_ = true;
              //alu_op_ is not used, imm is handled separately
              break;
          case get_instr_encoding(Instruction::kAuipc).opcode: // AUIPC
              reg_write_ = true;
              // alu_op_ is not used, PC+imm is handled separately
              break;
          case get_instr_encoding(Instruction::kItype).opcode: //I-type
              alu_src_ = true;
              reg_write_ = true;
              alu_op_ = 0b00;
              break;
          case get_instr_encoding(Instruction::kRtype).opcode: //R-type
              reg_write_ = true;
              alu_op_ = 0b10;
              break;
          case get_instr_encoding(Instruction::kLoadType).opcode: // Load
              mem_to_reg_ = true;
              reg_write_ = true;
              mem_read_ = true;
              alu_src_ = true;
              alu_op_ = 0b00; // ADD for address calculation
              break;
          case get_instr_encoding(Instruction::kStype).opcode: //Store type
              mem_write_ = true;
              alu_src_ = true;
              alu_op_ = 0b00; // ADD for address calculation
              break;
          case get_instr_encoding(Instruction::kBtype).opcode: //Branch
              branch_ = true;
              alu_op_ = 0b01; // SUB for comparison
              break;
          case get_instr_encoding(Instruction::kjal).opcode: //jal
              reg_write_ = true;
              branch_ = true; // To signal a PC change
              break;
          case get_instr_encoding(Instruction::kjalr).opcode: //jalr
              reg_write_ = true;
              branch_ = true; // To signal a PC change
              alu_src_ = true;
              break;
      }
  }
}

alu::AluOp RV5SControlUnit::GetAluSignal(uint32_t instruction, bool ALUOp) {
    uint8_t opcode = instruction & 0b1111111;
    uint8_t funct3 = (instruction >> 12) & 0b111;
    uint8_t funct7 = (instruction >> 25) & 0b1111111;

    if (instruction_set::isFInstruction(instruction) || instruction_set::isDInstruction(instruction)) {

        // This is a simplified example
        switch(opcode) {
            case 0b1010011: // F-R-Type
                if (funct7 == get_instr_encoding(Instruction::kfadd_s).funct7) return alu::AluOp::FADD;
                if (funct7 == get_instr_encoding(Instruction::kfsub_s).funct7) return alu::AluOp::FSUB;
                if (funct7 == get_instr_encoding(Instruction::kfmul_s).funct7) return alu::AluOp::FMUL;
                if (funct7 == get_instr_encoding(Instruction::kfdiv_s).funct7) return alu::AluOp::FDIV;
                if (funct7 == get_instr_encoding(Instruction::kfsqrt_s).funct7) return alu::AluOp::FSQRT;
                if (funct7 == get_instr_encoding(Instruction::kfsgnj_s).funct7) return alu::AluOp::FSGNJ;
                if (funct7 == get_instr_encoding(Instruction::kfmin_s).funct7) return alu::AluOp::FMIN;
                if (funct7 == get_instr_encoding(Instruction::kfmax_s).funct7) return alu::AluOp::FMAX;
                if (funct7 == get_instr_encoding(Instruction::kfcvt_w_s).funct7) return alu::AluOp::FCVT_W_S;
                if (funct7 == get_instr_encoding(Instruction::kfmv_x_w).funct7 && funct3 == 0b000) return alu::AluOp::FMV_X_W;
                if (funct7 == get_instr_encoding(Instruction::kfeq_s).funct7) return alu::AluOp::FEQ;
                if (funct7 == get_instr_encoding(Instruction::kflt_s).funct7) return alu::AluOp::FLT;
                if (funct7 == get_instr_encoding(Instruction::kfle_s).funct7) return alu::AluOp::FLE;
                if (funct7 == get_instr_encoding(Instruction::kfclass_s).funct7) return alu::AluOp::FCLASS;
                if (funct7 == get_instr_encoding(Instruction::kfcvt_s_w).funct7) return alu::AluOp::FCVT_S_W;
                if (funct7 == get_instr_encoding(Instruction::kfmv_w_x).funct7) return alu_op_ == 0b000 ? alu::AluOp::FMV_W_X : alu::AluOp::NOP;
                break;
            case 0b1000011: return alu::AluOp::FMADD;
            case 0b1000111: return alu::AluOp::FMSUB;
            case 0b1001011: return alu::AluOp::FNMSUB;
            case 0b1001111: return alu::AluOp::FNMADD;
            default: return alu::AluOp::ADD; // For address calculation
        }
        return alu::AluOp::kNone;
    }

    if (ALUOp == 0b01) { // Branch
        switch (funct3) {
            case 0b000: return alu::AluOp::SUB; //beq,bne
            case 0b001: return alu::AluOp::SUB; //beq,bne
            case 0b100: return alu::AluOp::SLT; //blt
            case 0b101: return alu::AluOp::SLT; //bge
            case 0b110: return alu::AluOp::SLTU; //bltu -branch if less than unsigned
            case 0b111: return alu::AluOp::SLTU; //bgeu
            default: return alu::AluOp::kNone;
        }
    }

    if (opcode == get_instr_encoding(Instruction::kItype).opcode || opcode == get_instr_encoding(Instruction::kLoadType).opcode || opcode == get_instr_encoding(Instruction::kStype).opcode || opcode == get_instr_encoding(Instruction::kjalr).opcode) { // I-type, Load, Store, JALR
        switch (funct3) {
            case 0b000: return alu::AluOp::ADD;
            case 0b001: return alu::AluOp::SLL;
            case 0b010: return alu::AluOp::SLT;
            case 0b011: return alu::AluOp::SLTU;
            case 0b100: return alu::AluOp::XOR;
            case 0b101: return (funct7 >> 5) ? alu::AluOp::SRA : alu::AluOp::SRL;
            case 0b110: return alu::AluOp::OR;
            case 0b111: return alu::AluOp::AND;
            default: return alu::AluOp::NOP;
        }
    }

    if (opcode == get_instr_encoding(Instruction::kRtype).opcode) { // R-type
        switch (funct3) {
            case 0b000: return (funct7 >> 5) ? alu::AluOp::SUB : alu::AluOp::ADD;
            case 0b001: return alu::AluOp::SLL;
            case 0b010: return alu::AluOp::SLT;
            case 0b011: return alu::AluOp::SLTU;
            case 0b100: return alu::AluOp::XOR;
            case 0b101: return (funct7 >> 5) ? alu::AluOp::SRA : alu::AluOp::SRL;
            case 0b110: return alu::AluOp::OR;
            case 0b111: return alu::AluOp::AND;
            default: return alu::AluOp::NOP;
        }
    }

    return alu::AluOp::NOP;
}
