#include "vm/rv5s/rv5s_control_unit.h" 
#include "common/instructions.h"
#include "vm/control_unit_base.h"
#include "vm/alu.h"
#include <cstdint>

using namespace instruction_set;

void RV5SControlUnit::SetControlSignals(uint32_t instruction) {
  uint8_t opcode = instruction & 0b1111111;

  reg_write_ = false;
  mem_to_reg_ = false;
  branch_ = false;
  mem_read_ = false;
  mem_write_ = false;
  alu_src_ = false;
  jump_ = false;
  pc_to_alu_ = false;
  alu_op_ = 0;

  if (instruction_set::isFInstruction(instruction) || instruction_set::isDInstruction(instruction) ||
      opcode == 0b1000011 || opcode == 0b1000111 ||
      opcode == 0b1001011 || opcode == 0b1001111) {
      switch (opcode) {
          case 0b0000111: // FLW, FLD
              alu_src_ = true; 
              mem_to_reg_ = true;
              reg_write_ = true; 
              mem_read_ = true;
              alu_op_ = 0b00;
              break;
          case 0b0100111: // FSW, FSD
              alu_src_ = true;
              mem_write_ = true;
              alu_op_ = 0b00;
              break;
          case 0b1000011: 
          case 0b1000111: 
          case 0b1001011: 
          case 0b1001111: 
              reg_write_ = true;
              break;
          case 0b1010011: 
              reg_write_ = true;
              break;
      }
  } else {
      switch (opcode) {
          case 0b0110111: // LUI
              reg_write_ = true; 
              alu_op_ = 0b00;
              break;
          case 0b0010111: // AUIPC
              reg_write_ = true; 
              pc_to_alu_ = true;
              alu_src_ = true;
              alu_op_ = 0b00;
              break;
          case get_instr_encoding(Instruction::kItype).opcode: 
              alu_src_ = true;
              reg_write_ = true;
              alu_op_ = 0b00;
              break;
          case get_instr_encoding(Instruction::kRtype).opcode: 
              reg_write_ = true;
              alu_op_ = 0b10;
              break;
          case get_instr_encoding(Instruction::kLoadType).opcode: 
              mem_to_reg_ = true;
              reg_write_ = true;
              mem_read_ = true;
              alu_src_ = true;
              alu_op_ = 0b00;
              break;
          case get_instr_encoding(Instruction::kStype).opcode: 
              mem_write_ = true;
              alu_src_ = true;
              alu_op_ = 0b00;
              break;
          case get_instr_encoding(Instruction::kBtype).opcode: 
              branch_ = true;
              alu_op_ = 0b01;
              break;
          case get_instr_encoding(Instruction::kjal).opcode: 
              reg_write_ = true;
              jump_ = true;
              pc_to_alu_ = true;
              alu_src_ = true;
              alu_op_ = 0b00;
              break;
          case get_instr_encoding(Instruction::kjalr).opcode: 
              reg_write_ = true;
              jump_ = true;
              alu_src_ = true;
              alu_op_ = 0b00;
              break;
      }
  }
}

  alu::AluOp RV5SControlUnit::GetAluSignal(uint32_t instruction, uint8_t alu_op) {
      uint8_t opcode = instruction & 0b1111111;
      uint8_t funct3 = (instruction >> 12) & 0b111;
      uint8_t funct7 = (instruction >> 25) & 0b1111111;

    switch (opcode) {
        case 0b1000011: {
            uint8_t fmt = (instruction >> 25) & 0b11;
            if (fmt == 0b00) return alu::AluOp::kFmadd_s;
            if (fmt == 0b01) return alu::AluOp::FMADD_D;
            break;
        }
        case 0b1000111: {
            uint8_t fmt = (instruction >> 25) & 0b11;
            if (fmt == 0b00) return alu::AluOp::kFmsub_s;
            if (fmt == 0b01) return alu::AluOp::FMSUB_D;
            break;
        }
        case 0b1001011: {
            uint8_t fmt = (instruction >> 25) & 0b11;
            if (fmt == 0b00) return alu::AluOp::kFnmadd_s;
            if (fmt == 0b01) return alu::AluOp::FNMADD_D;
            break;
        }
        case 0b1001111: {
            uint8_t fmt = (instruction >> 25) & 0b11;
            if (fmt == 0b00) return alu::AluOp::kFnmsub_s;
            if (fmt == 0b01) return alu::AluOp::FNMSUB_D;
            break;
        }
        default:
            break;
    }

    if (instruction_set::isFInstruction(instruction) || instruction_set::isDInstruction(instruction)) {
        switch (opcode) {
            case 0b1010011: {
                switch (funct7) {
                    case 0b0000000: return alu::AluOp::FADD_S; 
                    case 0b0000001: return alu::AluOp::FADD_D; 
                    case 0b0000100: return alu::AluOp::FSUB_S; 
                    case 0b0000101: return alu::AluOp::FSUB_D; 
                    case 0b0001000: return alu::AluOp::FMUL_S; 
                    case 0b0001001: return alu::AluOp::FMUL_D; 
                    case 0b0001100: return alu::AluOp::FDIV_S; 
                    case 0b0001101: return alu::AluOp::FDIV_D; 
                    case 0b0101100: return alu::AluOp::FSQRT_S; 
                    case 0b0101101: return alu::AluOp::FSQRT_D; 
                    case 0b1100000: {
                        uint8_t funct5 = (instruction >> 27) & 0b11111;
                        switch (funct5) {
                            case 0b00000: return alu::AluOp::FCVT_W_S;
                            case 0b00001: return alu::AluOp::FCVT_WU_S;
                            case 0b00010: return alu::AluOp::FCVT_L_S;
                            case 0b00011: return alu::AluOp::FCVT_LU_S;
                        }
                        break;
                    }
                    case 0b1100001: {
                        uint8_t funct5 = (instruction >> 27) & 0b11111;
                        switch (funct5) {
                            case 0b00000: return alu::AluOp::FCVT_W_D;
                            case 0b00001: return alu::AluOp::FCVT_WU_D;
                            case 0b00010: return alu::AluOp::FCVT_L_D;
                            case 0b00011: return alu::AluOp::FCVT_LU_D;
                        }
                        break;
                    }
                    case 0b1101000: {
                        uint8_t funct5 = (instruction >> 27) & 0b11111;
                        switch (funct5) {
                            case 0b00000: return alu::AluOp::FCVT_S_W;
                            case 0b00001: return alu::AluOp::FCVT_S_WU;
                            case 0b00010: return alu::AluOp::FCVT_S_L;
                            case 0b00011: return alu::AluOp::FCVT_S_LU;
                        }
                        break;
                    }
                    case 0b1101001: {
                        uint8_t funct5 = (instruction >> 27) & 0b11111;
                        switch (funct5) {
                            case 0b00000: return alu::AluOp::FCVT_D_W;
                            case 0b00001: return alu::AluOp::FCVT_D_WU;
                            case 0b00010: return alu::AluOp::FCVT_D_L;
                            case 0b00011: return alu::AluOp::FCVT_D_LU;
                        }
                        break;
                    }
                    case 0b0010000: {
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FSGNJ_S;
                            case 0b001: return alu::AluOp::FSGNJN_S;
                            case 0b010: return alu::AluOp::FSGNJX_S;
                        }
                        break;
                    }
                    case 0b0010001: {
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FSGNJ_D;
                            case 0b001: return alu::AluOp::FSGNJN_D;
                            case 0b010: return alu::AluOp::FSGNJX_D;
                        }
                        break;
                    }
                    case 0b0010100: {
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FMIN_S;
                            case 0b001: return alu::AluOp::FMAX_S;
                        }
                        break;
                    }
                    case 0b0010101: {
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FMIN_D;
                            case 0b001: return alu::AluOp::FMAX_D;
                        }
                        break;
                    }
                    case 0b1010000: {
                        switch (funct3) {
                            case 0b010: return alu::AluOp::FEQ_S;
                            case 0b001: return alu::AluOp::FLT_S;
                            case 0b000: return alu::AluOp::FLE_S;
                        }
                        break;
                    }
                    case 0b1010001: {
                        uint8_t rs2 = (instruction >> 20) & 0b11111;
                        uint8_t fmt = (instruction >> 25) & 0b11;
                        if (rs2 != 0) {
                            if (fmt == 0b00 && rs2 == 0b00001) return alu::AluOp::FCVT_S_D;
                            if (fmt == 0b01 && rs2 == 0b00000) return alu::AluOp::FCVT_D_S;
                        }
                        switch (funct3) {
                            case 0b010: return alu::AluOp::FEQ_D;
                            case 0b001: return alu::AluOp::FLT_D;
                            case 0b000: return alu::AluOp::FLE_D;
                        }
                        break;
                    }
                    case 0b1111000: return alu::AluOp::FMV_W_X;
                    case 0b1111001: return alu::AluOp::FMV_D_X;
                    case 0b1110000: {
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FMV_X_W;
                            case 0b001: return alu::AluOp::FCLASS_S;
                        }
                        break;
                    }
                    case 0b1110001: {
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FMV_X_D;
                            case 0b001: return alu::AluOp::FCLASS_D;
                        }
                        break;
                    }
                    case 0b0100001: {
                        uint8_t rs2 = (instruction >> 20) & 0b11111;
                        uint8_t fmt = (instruction >> 25) & 0b11;
                        if (fmt == 0b00 && rs2 == 0b00001) return alu::AluOp::FCVT_S_D;
                        if (fmt == 0b01 && rs2 == 0b00000) return alu::AluOp::FCVT_D_S;
                        break;
                    }
                    // ADDED CASE FOR FCVT.S.D (Double -> Single)
                    case 0b0100000: {
                        uint8_t rs2 = (instruction >> 20) & 0b11111;
                        // rs2=1 for Double->Single
                        if (rs2 == 0b00001) return alu::AluOp::FCVT_S_D;
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case 0b0000111: return alu::AluOp::kAdd;
            case 0b0100111: return alu::AluOp::kAdd;
            default: return alu::AluOp::kAdd;
        }
        return alu::AluOp::kNone;
    }

    const bool is_branch_opcode = (opcode == get_instr_encoding(Instruction::kBtype).opcode);

    if (alu_op == 0b01 || is_branch_opcode) {
        switch (funct3) {
            case 0b000: return alu::AluOp::kSub; 
            case 0b001: return alu::AluOp::kSub; 
            case 0b100: return alu::AluOp::kSub; 
            case 0b101: return alu::AluOp::kSub; 
            case 0b110: return alu::AluOp::kSub; 
            case 0b111: return alu::AluOp::kSub; 
            default: return alu::AluOp::kNone;
        }
    }

    if (opcode == get_instr_encoding(Instruction::kLoadType).opcode ||
        opcode == get_instr_encoding(Instruction::kStype).opcode   ||
        opcode == get_instr_encoding(Instruction::kjalr).opcode) {
        return alu::AluOp::kAdd;
    }

    if (opcode == get_instr_encoding(Instruction::kItype).opcode) {
        switch (funct3) {
            case 0b000: return alu::AluOp::kAdd;
            case 0b001: return alu::AluOp::kSll;
            case 0b010: return alu::AluOp::kSlt;
            case 0b011: return alu::AluOp::kSltu;
            case 0b100: return alu::AluOp::kXor;
            case 0b101: return (funct7 >> 5) ? alu::AluOp::kSra : alu::AluOp::kSrl;
            case 0b110: return alu::AluOp::kOr;
            case 0b111: return alu::AluOp::kAnd;
            default:    return alu::AluOp::kNone;
        }
    }

    if (opcode == get_instr_encoding(Instruction::kjal).opcode) {
        return alu::AluOp::kAdd;
    }

    const bool is_rtype = (opcode == get_instr_encoding(Instruction::kRtype).opcode);
    const bool is_rv64_op32 = (opcode == 0b0111011);

    if (is_rtype || is_rv64_op32) {
        if (funct7 == 0b0000001) {
            if (is_rv64_op32) {
                switch (funct3) {
                    case 0b000: return alu::AluOp::kMulw;
                    case 0b100: return alu::AluOp::kDivw;
                    case 0b101: return alu::AluOp::kDivuw;
                    case 0b110: return alu::AluOp::kRemw;
                    case 0b111: return alu::AluOp::kRemuw;
                    default:    return alu::AluOp::kNone;
                }
            } else {
                switch (funct3) {
                    case 0b000: return alu::AluOp::kMul;
                    case 0b001: return alu::AluOp::kMulh;
                    case 0b010: return alu::AluOp::kMulhsu;
                    case 0b011: return alu::AluOp::kMulhu;
                    case 0b100: return alu::AluOp::kDiv;
                    case 0b101: return alu::AluOp::kDivu;
                    case 0b110: return alu::AluOp::kRem;
                    case 0b111: return alu::AluOp::kRemu;
                    default:    return alu::AluOp::kNone;
                }
            }
        }

        switch (funct3) {
            case 0b000: return (funct7 >> 5) ? alu::AluOp::kSub : (is_rv64_op32 ? alu::AluOp::kAddw : alu::AluOp::kAdd);
            case 0b001: return is_rv64_op32 ? alu::AluOp::kSllw : alu::AluOp::kSll;
            case 0b010: return alu::AluOp::kSlt;
            case 0b011: return alu::AluOp::kSltu;
            case 0b100: return alu::AluOp::kXor;
            case 0b101: return is_rv64_op32
                             ? ((funct7 >> 5) ? alu::AluOp::kSraw : alu::AluOp::kSrlw)
                             : ((funct7 >> 5) ? alu::AluOp::kSra  : alu::AluOp::kSrl);
            case 0b110: return alu::AluOp::kOr;
            case 0b111: return alu::AluOp::kAnd;
            default: return alu::AluOp::kNone;
        }
    }

    if (opcode == 0b0110111) return alu::AluOp::kAdd;
    if (opcode == 0b0010111) return alu::AluOp::kAdd;

    return alu::AluOp::kNone;
}
