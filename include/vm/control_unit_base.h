#ifndef CONTROL_UNIT_BASE_H
#define CONTROL_UNIT_BASE_H

#include "registers.h"
#include "alu.h"

class ControlUnit {
 public:
  virtual ~ControlUnit() = default;

  void Reset() {
    alu_src_ = false;//alu will use register value as second oeprand
    mem_to_reg_ = false;//write alu result to register not to memory
    reg_write_ = false;//do not write to register file
    mem_read_ = false;//do not read from memory
    mem_write_ = false;//not a branch instruction
    branch_ = false;//default alu operation
    jump_ = false;
    alu_op_ = 0;
  }

  virtual void SetControlSignals(uint32_t instruction) = 0;
  virtual alu::AluOp GetAluSignal(uint32_t instruction, bool ALUOp) = 0;

  [[nodiscard]] bool GetAluSrc() const;
  [[nodiscard]] bool GetMemToReg() const;
  [[nodiscard]] bool GetRegWrite() const;
  [[nodiscard]] bool GetMemRead() const;
  [[nodiscard]] bool GetMemWrite() const;
  [[nodiscard]] uint8_t GetAluOp() const;
  [[nodiscard]] bool GetBranch() const;
  [[nodiscard]] bool GetJump() const;

 protected:
  bool reg_write_ = false;
  bool branch_ = false;
  bool alu_src_ = false;
  bool mem_read_ = false;
  bool mem_write_ = false;
  bool mem_to_reg_ = false;
  bool jump_ = false;

  uint8_t alu_op_{};
};

#endif 
