#include "vm/rv5s/rv5s_control_unit.h" 
#include "common/instructions.h"
#include "vm/control_unit_base.h"
#include "vm/alu.h"
#include <cstdint>

using namespace instruction_set;

void RV5SControlUnit::SetControlSignals(uint32_t instruction) {
  uint8_t opcode = instruction & 0b1111111;

  // clear all control signals
  reg_write_ = false;
  mem_to_reg_ = false;
  branch_ = false;
  mem_read_ = false;
  mem_write_ = false;
  alu_src_ = false;
  jump_ = false;
  pc_to_alu_ = false;
  alu_op_ = 0;

  // handle FP and FMA instructions
  if (instruction_set::isFInstruction(instruction) || instruction_set::isDInstruction(instruction) ||
      opcode == 0b1000011 || opcode == 0b1000111 ||
      opcode == 0b1001011 || opcode == 0b1001111) {
      switch (opcode) {
          case 0b0000111: // FLW, FLD (load)
              alu_src_ = true;      // base + imm
              mem_to_reg_ = true;   // write value from memory
              reg_write_ = true;    // write FP reg
              mem_read_ = true;     // load
              alu_op_ = 0b00;       // ADD for address
              break;
          case 0b0100111: // FSW, FSD (store)
              alu_src_ = true;      // base + imm
              mem_write_ = true;    // store
              alu_op_ = 0b00;       // ADD for address
              break;
          case 0b1000011: // FMADD.S/D
          case 0b1000111: // FMSUB.S/D
          case 0b1001011: // FNMSUB.S/D
          case 0b1001111: // FNMADD.S/D
              reg_write_ = true;    // FMA writes FP dest
              break;
          case 0b1010011: // Float R-Type
              reg_write_ = true;    // FP R-type writes result
              break;
      }
  } else {
      // normal integer instructions
      switch (opcode) {
          case 0b0110111: // LUI
              reg_write_ = true;    // write rd
              alu_op_ = 0b00;       // ALU op not really used (handled specially)
              break;
          case 0b0010111: // AUIPC
              reg_write_ = true;    // write rd
              pc_to_alu_ = true;    // use PC as first operand
              alu_src_ = true;      // use imm as second operand
              alu_op_ = 0b00;       // ADD
              break;
          case get_instr_encoding(Instruction::kItype).opcode: // I-type
              alu_src_ = true;      // rs1 + imm
              reg_write_ = true;    // write rd
              alu_op_ = 0b00;       // ALU op chosen later by GetAluSignal
              break;
          case get_instr_encoding(Instruction::kRtype).opcode: // R-type
              reg_write_ = true;    // write rd
              alu_op_ = 0b10;       // use funct3/funct7
              break;
          case get_instr_encoding(Instruction::kLoadType).opcode: // Load
              mem_to_reg_ = true;   // write from memory
              reg_write_ = true;    // write rd
              mem_read_ = true;     // load
              alu_src_ = true;      // base + imm
              alu_op_ = 0b00;       // ADD for address
              break;
          case get_instr_encoding(Instruction::kStype).opcode: // Store
              mem_write_ = true;    // store
              alu_src_ = true;      // base + imm
              alu_op_ = 0b00;       // ADD for address
              break;
          case get_instr_encoding(Instruction::kBtype).opcode: // Branch
              branch_ = true;       // branch instr
              alu_op_ = 0b01;       // compare (SUB)
              break;
          case get_instr_encoding(Instruction::kjal).opcode: // JAL
              reg_write_ = true;    // write return addr to rd
              jump_ = true;         // change PC
              pc_to_alu_ = true;    // PC as first operand
              alu_src_ = true;      // imm as second operand
              alu_op_ = 0b00;       // ADD PC + imm
              break;
          case get_instr_encoding(Instruction::kjalr).opcode: // JALR
              reg_write_ = true;    // write return addr
              jump_ = true;         // change PC
              alu_src_ = true;      // rs1 + imm
              alu_op_ = 0b00;       // ADD
              break;
      }
  }
}

// choose the exact ALU operation
  alu::AluOp RV5SControlUnit::GetAluSignal(uint32_t instruction, uint8_t alu_op) {
      uint8_t opcode = instruction & 0b1111111;
      uint8_t funct3 = (instruction >> 12) & 0b111;
      uint8_t funct7 = (instruction >> 25) & 0b1111111;

    if (instruction_set::isFInstruction(instruction) || instruction_set::isDInstruction(instruction)) {
        /*
         * Decode floating-point and double-precision instructions.
         * The opcode identifies the general class of FP instruction.
         * For OP-FP (0b1010011) the upper 7 bits (funct7) select the operation,
         * with bit 0 of funct7 indicating single (0) or double (1) precision for many ops.
         * Some instructions also examine funct3 or other fields for sub‑opcodes.
         * For the fused multiply-add variants (FMADD/FM{SUB,NMADD,NMSUB}), the opcode determines
         * the family and bits [26:25] (funct2) select single (0b00) or double (0b01) format.
         */
        switch (opcode) {
            // Fused multiply-add/subtract families
            case 0b1000011: {
                // FMADD.S/D
                uint8_t fmt = (instruction >> 25) & 0b11;
                if (fmt == 0b00) return alu::AluOp::kFmadd_s;        // FMADD.S
                else if (fmt == 0b01) return alu::AluOp::FMADD_D;    // FMADD.D
                break;
            }
            case 0b1000111: {
                // FMSUB.S/D
                uint8_t fmt = (instruction >> 25) & 0b11;
                if (fmt == 0b00) return alu::AluOp::kFmsub_s;        // FMSUB.S
                else if (fmt == 0b01) return alu::AluOp::FMSUB_D;    // FMSUB.D
                break;
            }
            case 0b1001011: {
                // FNMADD.S/D
                uint8_t fmt = (instruction >> 25) & 0b11;
                if (fmt == 0b00) return alu::AluOp::kFnmadd_s;       // FNMADD.S
                else if (fmt == 0b01) return alu::AluOp::FNMADD_D;   // FNMADD.D
                break;
            }
            case 0b1001111: {
                // FNMSUB.S/D
                uint8_t fmt = (instruction >> 25) & 0b11;
                if (fmt == 0b00) return alu::AluOp::kFnmsub_s;       // FNMSUB.S
                else if (fmt == 0b01) return alu::AluOp::FNMSUB_D;   // FNMSUB.D
                break;
            }
            case 0b1010011: {
                // OP-FP: arithmetic, comparisons, conversions
                switch (funct7) {
                    case 0b0000000: return alu::AluOp::FADD_S;   // FADD.S
                    case 0b0000001: return alu::AluOp::FADD_D;   // FADD.D
                    case 0b0000100: return alu::AluOp::FSUB_S;   // FSUB.S
                    case 0b0000101: return alu::AluOp::FSUB_D;   // FSUB.D
                    case 0b0001000: return alu::AluOp::FMUL_S;   // FMUL.S
                    case 0b0001001: return alu::AluOp::FMUL_D;   // FMUL.D
                    case 0b0001100: return alu::AluOp::FDIV_S;   // FDIV.S
                    case 0b0001101: return alu::AluOp::FDIV_D;   // FDIV.D
                    case 0b0101100: return alu::AluOp::FSQRT_S;  // FSQRT.S
                    case 0b0101101: return alu::AluOp::FSQRT_D;  // FSQRT.D
                    case 0b1100000: {
                        // FCVT.(W|WU|L|LU).S
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
                        // FCVT.(W|WU|L|LU).D
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
                        // FCVT.S.(W|WU|L|LU)
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
                        // FCVT.D.(W|WU|L|LU)
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
                        // FSGNJ(N|X).S
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FSGNJ_S;
                            case 0b001: return alu::AluOp::FSGNJN_S;
                            case 0b010: return alu::AluOp::FSGNJX_S;
                        }
                        break;
                    }
                    case 0b0010001: {
                        // FSGNJ(N|X).D
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FSGNJ_D;
                            case 0b001: return alu::AluOp::FSGNJN_D;
                            case 0b010: return alu::AluOp::FSGNJX_D;
                        }
                        break;
                    }
                    case 0b0010100: {
                        // FMIN/FMAX.S
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FMIN_S;
                            case 0b001: return alu::AluOp::FMAX_S;
                        }
                        break;
                    }
                    case 0b0010101: {
                        // FMIN/FMAX.D
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FMIN_D;
                            case 0b001: return alu::AluOp::FMAX_D;
                        }
                        break;
                    }
                    case 0b1010000: {
                        // F(EQ|LT|LE).S
                        switch (funct3) {
                            case 0b010: return alu::AluOp::FEQ_S;
                            case 0b001: return alu::AluOp::FLT_S;
                            case 0b000: return alu::AluOp::FLE_S;
                        }
                        break;
                    }
                    case 0b1010001: {
                        // F(EQ|LT|LE).D
                        switch (funct3) {
                            case 0b010: return alu::AluOp::FEQ_D;
                            case 0b001: return alu::AluOp::FLT_D;
                            case 0b000: return alu::AluOp::FLE_D;
                        }
                        break;
                    }
                    case 0b1111000: {
                        // FMV.W.X (S)
                        return alu::AluOp::FMV_W_X;
                    }
                    case 0b1111001: {
                        // FMV.D.X
                        return alu::AluOp::FMV_D_X;
                    }
                    case 0b1110000: {
                        // FMV.X.W or FCLASS.S
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FMV_X_W;
                            case 0b001: return alu::AluOp::FCLASS_S;
                        }
                        break;
                    }
                    case 0b1110001: {
                        // FMV.X.D or FCLASS.D
                        switch (funct3) {
                            case 0b000: return alu::AluOp::FMV_X_D;
                            case 0b001: return alu::AluOp::FCLASS_D;
                        }
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case 0b0000111: {
                // FLW/FLD: address calculation uses ADD
                return alu::AluOp::kAdd;
            }
            case 0b0100111: {
                // FSW/FSD: address calculation uses ADD
                return alu::AluOp::kAdd;
            }
            default: {
                // For unrecognized FP instructions, default to ADD (e.g., address calc)
                return alu::AluOp::kAdd;
            }
        }
        // If no FP pattern matched, return kNone
        return alu::AluOp::kNone;
    }

    const bool is_branch_opcode = (opcode == get_instr_encoding(Instruction::kBtype).opcode);

    // branch ops: always use SUB-based compare
    if (alu_op == 0b01 || is_branch_opcode) {
        switch (funct3) {
            case 0b000: return alu::AluOp::kSub;  // beq
            case 0b001: return alu::AluOp::kSub;  // bne
            case 0b100: return alu::AluOp::kSub;  // blt
            case 0b101: return alu::AluOp::kSub;  // bge
            case 0b110: return alu::AluOp::kSub;  // bltu
            case 0b111: return alu::AluOp::kSub;  // bgeu
            default: return alu::AluOp::kNone;
        }
    }

    // loads, stores and JALR use ADD for base + offset
    if (opcode == get_instr_encoding(Instruction::kLoadType).opcode ||
        opcode == get_instr_encoding(Instruction::kStype).opcode   ||
        opcode == get_instr_encoding(Instruction::kjalr).opcode) {
        return alu::AluOp::kAdd;
    }

    // I-type arithmetic: choose ALU op from funct3/funct7
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

    // JAL uses ADD for PC + imm
    if (opcode == get_instr_encoding(Instruction::kjal).opcode) {
        return alu::AluOp::kAdd;
    }

    const bool is_rtype = (opcode == get_instr_encoding(Instruction::kRtype).opcode);
    const bool is_rv64_op32 = (opcode == 0b0111011);

    // R-type and RV64-32bit ops
    if (is_rtype || is_rv64_op32) {
        // M-extension (mul/div/rem)
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

        // normal R-type ALU ops
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

    // LUI: ALU just passes imm (or ADD with 0)
    if (opcode == 0b0110111) { // LUI
        return alu::AluOp::kAdd;
    }

    // AUIPC: PC + imm
    if (opcode == 0b0010111) { // AUIPC
        return alu::AluOp::kAdd;
    }

    // default when nothing matches
    return alu::AluOp::kNone;
}
