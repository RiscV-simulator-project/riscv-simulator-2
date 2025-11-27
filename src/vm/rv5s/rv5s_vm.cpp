#include "vm/rv5s/rv5s_vm.h"
#include "common/instructions.h"
#include "utils.h"
#include "globals.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <tuple>

RiscV5StageVM::RiscV5StageVM() : VmBase() {
    Reset();
}

void RiscV5StageVM::Run() {
    stop_requested_ = false;
    
    auto pipeline_not_empty = [&]() {
        return if_id_reg_.valid ||
               id_ex_reg_.valid ||
               ex_mem_reg_.valid ||
               mem_wb_reg_.valid;
    };
    
    while (!stop_requested_ && 
           (program_counter_ < program_size_ || pipeline_not_empty())) {
        ClockTick();
    }
    
    std::cout << "VM_PROGRAM_END" << std::endl;
    output_status_ = "VM_PROGRAM_END";
    
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);

    std::cout << "VM_STATS cycles=" << cycle_s_
              << " retired=" << instructions_retired_
              << " stalls=" << stall_cycles_
              << " mispredictions=" << branch_mispredictions_
              << std::endl;
}

void RiscV5StageVM::ClockTick() {
    if (debug_mode_) {
        DebugPrintCycle();
    }

    if (system_trap_cycles_ > 0) {
        system_trap_cycles_--;
        cycle_s_++;
        return;
    }
    
    if (hazard_detection_enabled_) {
        DetectAndHandleHazards();
    } else {
        pc_write_ = true;
        if_id_write_ = true;
    }

    WriteBackStage();
    MemoryStage();
    ExecuteStage();
    DecodeStage();
    FetchStage();

    if (if_id_write_) {
        if_id_reg_ = next_if_id_reg_;
    }
    id_ex_reg_ = next_id_ex_reg_;
    ex_mem_reg_ = next_ex_mem_reg_;
    mem_wb_reg_ = next_mem_wb_reg_;
    
    if (pc_write_) {
        if (branch_taken_) {
            program_counter_ = branch_target_;
            if (branch_mispredicted_) {
                branch_mispredictions_++;
            }
            branch_taken_ = false;
            branch_mispredicted_ = false;
            has_predicted_dec_ = false;
            predicted_taken_dec_ = false;
            predicted_target_dec_ = 0;
        } else {
            if (has_predicted_dec_) {
                if (predicted_taken_dec_) {
                    program_counter_ = predicted_target_dec_;
                } else {
                    program_counter_ += 4;
                }
            } else {
                program_counter_ += 4;
            }
            has_predicted_dec_ = false;
            predicted_taken_dec_ = false;
            predicted_target_dec_ = 0;
        }
    }

    cycle_s_++;
}

// STAGE 5: Write-Back
void RiscV5StageVM::WriteBackStage() {
    if (!mem_wb_reg_.valid) return;

    // Some FP instr write to integer regs (GPR)
    bool fp_op_writes_gpr = false;
    if (mem_wb_reg_.is_fp_instr) {
        uint8_t opcode = mem_wb_reg_.instruction & 0b1111111;
        if (opcode == 0b1010011) { // OP-FP
            uint32_t funct5 = (mem_wb_reg_.instruction >> 27) & 0x1F;
            // FEQ/FLT/FLE, FCVT.W.S/FCVT.L.S, FCVT.W.D, FCLASS/FMV.X.W write GPR
            if (funct5 == 0b10100 || funct5 == 0b11000 || 
                funct5 == 0b11001 || funct5 == 0b11100) {
                fp_op_writes_gpr = true;
            }
        }
    }

    bool is_gpr_write = (!mem_wb_reg_.is_fp_instr) || fp_op_writes_gpr;
    bool gpr_zero_dest = (is_gpr_write && mem_wb_reg_.rd_idx == 0);

    if (mem_wb_reg_.control.reg_write && !gpr_zero_dest) {
        uint64_t data_to_write;
        if (mem_wb_reg_.control.jump) {
            data_to_write = mem_wb_reg_.link_address;
        } else if (mem_wb_reg_.control.mem_to_reg) {
            data_to_write = mem_wb_reg_.mem_read_data;
        } else {
            data_to_write = mem_wb_reg_.alu_result;
        }
        
        if (mem_wb_reg_.is_fp_instr && !fp_op_writes_gpr) {
            // Write to FP register file.
            bool fp_is_double = instruction_set::isDInstruction(mem_wb_reg_.instruction);
            uint8_t opcode = mem_wb_reg_.instruction & 0b1111111;
            if (opcode == 0b1000011 || opcode == 0b1000111 ||
                opcode == 0b1001011 || opcode == 0b1001111) {
                // Fused Multiply-Add: check fmt bits
                uint8_t fmt = (mem_wb_reg_.instruction >> 25) & 0b11;
                if (fmt == 0b01) fp_is_double = true;
            }

            // Enforce Zero-Extension for Single Precision results
            if (!fp_is_double) {
                data_to_write &= 0x00000000FFFFFFFFULL;
            }
            
            registers_.WriteFpr(mem_wb_reg_.rd_idx, data_to_write);
        } else {
            registers_.WriteGpr(mem_wb_reg_.rd_idx, data_to_write);
        }
    }
    instructions_retired_++;
}

// STAGE 4: Memory Access
void RiscV5StageVM::MemoryStage() {
    next_mem_wb_reg_ = {};
    if (!ex_mem_reg_.valid) return;

    next_mem_wb_reg_.pc = ex_mem_reg_.pc;
    next_mem_wb_reg_.link_address = ex_mem_reg_.link_address;
    next_mem_wb_reg_.alu_result = ex_mem_reg_.alu_result;
    next_mem_wb_reg_.rd_idx = ex_mem_reg_.rd_idx;
    next_mem_wb_reg_.control = ex_mem_reg_.control;
    next_mem_wb_reg_.instruction = ex_mem_reg_.instruction;
    next_mem_wb_reg_.is_fp_instr = ex_mem_reg_.is_fp_instr;
    next_mem_wb_reg_.valid = true;

    // Handle loads
    if (ex_mem_reg_.control.mem_read) {
        uint64_t addr = ex_mem_reg_.alu_result;
        
        if (addr >= memory_controller_.GetMemorySize()) {
            std::cerr << "ERROR: Memory read address out of bounds: 0x" 
                      << std::hex << addr << std::dec << std::endl;
            next_mem_wb_reg_.mem_read_data = 0;
            next_mem_wb_reg_.valid = false;
            return;
        }
        
        switch (ex_mem_reg_.funct3) {
            case 0b000: // LB
                next_mem_wb_reg_.mem_read_data = static_cast<int64_t>(
                    static_cast<int8_t>(memory_controller_.ReadByte(addr))
                );
                break;
            case 0b001: // LH
                if (addr % 2 != 0) std::cerr << "ERROR: Misaligned halfword read\n";
                next_mem_wb_reg_.mem_read_data = static_cast<int64_t>(
                    static_cast<int16_t>(memory_controller_.ReadHalfWord(addr))
                );
                break;
            case 0b010: // LW and FLW
                {
                    if (addr % 4 != 0) std::cerr << "ERROR: Misaligned word read\n";
                    uint32_t val32 = memory_controller_.ReadWord(addr);
                    
                    if (ex_mem_reg_.is_fp_instr) {
                        // FLW: Zero-Extend to 64-bit FPR
                        next_mem_wb_reg_.mem_read_data = static_cast<uint64_t>(val32);
                    } else {
                        // LW: Sign-Extend to 64-bit GPR
                        next_mem_wb_reg_.mem_read_data = static_cast<int64_t>(static_cast<int32_t>(val32));
                    }
                }
                break;
            case 0b011: // LD
                if (addr % 8 != 0) std::cerr << "ERROR: Misaligned doubleword read\n";
                next_mem_wb_reg_.mem_read_data = memory_controller_.ReadDoubleWord(addr);
                break;
            case 0b100: // LBU
                next_mem_wb_reg_.mem_read_data = memory_controller_.ReadByte(addr);
                break;
            case 0b101: // LHU
                if (addr % 2 != 0) std::cerr << "ERROR: Misaligned halfword read\n";
                next_mem_wb_reg_.mem_read_data = memory_controller_.ReadHalfWord(addr);
                break;
            case 0b110: // LWU
                if (addr % 4 != 0) std::cerr << "ERROR: Misaligned word read\n";
                next_mem_wb_reg_.mem_read_data = memory_controller_.ReadWord(addr);
                break;
            default:
                std::cerr << "ERROR: Invalid load funct3\n";
                next_mem_wb_reg_.mem_read_data = 0;
                break;
        }
    }
    
    // Handle stores
    if (ex_mem_reg_.control.mem_write) {
        uint64_t addr = ex_mem_reg_.alu_result;
        
        if (addr >= memory_controller_.GetMemorySize()) {
            std::cerr << "ERROR: Memory write address out of bounds: 0x" 
                      << std::hex << addr << std::dec << std::endl;
            return;
        }
        
        switch (ex_mem_reg_.funct3) {
            case 0b000: // SB
                memory_controller_.WriteByte(addr, ex_mem_reg_.rs2_val & 0xFF);
                break;
            case 0b001: // SH
                if (addr % 2 != 0) std::cerr << "ERROR: Misaligned halfword write\n";
                memory_controller_.WriteHalfWord(addr, ex_mem_reg_.rs2_val & 0xFFFF);
                break;
            case 0b010: // SW and FSW
                if (addr % 4 != 0) std::cerr << "ERROR: Misaligned word write\n";
                memory_controller_.WriteWord(addr, ex_mem_reg_.rs2_val & 0xFFFFFFFF);
                break;
            case 0b011: // SD
                if (addr % 8 != 0) std::cerr << "ERROR: Misaligned doubleword write\n";
                memory_controller_.WriteDoubleWord(addr, ex_mem_reg_.rs2_val);
                break;
            default:
                std::cerr << "ERROR: Invalid store funct3\n";
                break;
        }
    }
}

// STAGE 3: Execute
void RiscV5StageVM::ExecuteStage() {
    next_ex_mem_reg_ = {};
    if (!id_ex_reg_.valid) return;

    bool is_ecall = (id_ex_reg_.instruction == 0x00000073);
    if (is_ecall) {
        if (!trap_in_progress_ && system_trap_cycles_ == 0) {
            trap_in_progress_ = true;
            system_trap_cycles_ = kSystemTrapLatency_;
        }
    } else if (trap_in_progress_) {
        trap_in_progress_ = false;
    }

    next_ex_mem_reg_.pc = id_ex_reg_.pc;
    next_ex_mem_reg_.link_address = id_ex_reg_.link_address;
    next_ex_mem_reg_.rd_idx = id_ex_reg_.rd_idx;
    next_ex_mem_reg_.funct3 = id_ex_reg_.funct3;
    next_ex_mem_reg_.control = id_ex_reg_.control;
    next_ex_mem_reg_.instruction = id_ex_reg_.instruction;
    next_ex_mem_reg_.alu_operation = id_ex_reg_.alu_operation;
    next_ex_mem_reg_.is_fp_instr = id_ex_reg_.is_fp_instr;
    next_ex_mem_reg_.valid = true;

    uint64_t alu_input1;
    uint64_t alu_input2;
    uint64_t alu_input3 = id_ex_reg_.rs3_val;
    
    if (id_ex_reg_.control.pc_to_alu) {
        alu_input1 = id_ex_reg_.pc;
    } else {
        alu_input1 = id_ex_reg_.rs1_val;
    }
    
    alu_input2 = id_ex_reg_.control.alu_src ? id_ex_reg_.imm : id_ex_reg_.rs2_val;
    uint64_t store_val = id_ex_reg_.rs2_val;

    bool rs1_needed = !id_ex_reg_.control.pc_to_alu;
    bool rs2_alu_needed = !id_ex_reg_.control.alu_src;
    bool rs2_store_needed = id_ex_reg_.control.mem_write;
    bool rs3_needed = id_ex_reg_.has_rs3;

    auto check_if_instruction_writes_fp = [](const auto& stage_reg) -> bool {
        if (!stage_reg.is_fp_instr) return false;
        uint8_t op = stage_reg.instruction & 0x7F;
        if (op == 0b0000111) return true;
        if (op == 0b1000011 || op == 0b1000111 || 
            op == 0b1001011 || op == 0b1001111) return true;
        if (op == 0b1010011) {
            uint32_t f5 = (stage_reg.instruction >> 27) & 0x1F;
            if (f5 == 0b10100 || f5 == 0b11000 || 
                f5 == 0b11001 || f5 == 0b11100) {
                return false; 
            }
        }
        if (op == 0b0100111) return false;
        return true;
    };

    // EX/MEM FORWARDING
    bool ex_mem_writes_fp = check_if_instruction_writes_fp(ex_mem_reg_);
    bool exmem_dest_is_fp = ex_mem_reg_.is_fp_instr && ex_mem_writes_fp;
    bool exmem_dest_is_gpr = (!ex_mem_reg_.is_fp_instr) || (ex_mem_reg_.is_fp_instr && !ex_mem_writes_fp);
    bool exmem_zero_dest = (exmem_dest_is_gpr && ex_mem_reg_.rd_idx == 0);
    
    if (ex_mem_reg_.valid && 
        ex_mem_reg_.control.reg_write && 
        !exmem_zero_dest) {
        
        bool can_forward_from_exmem = !ex_mem_reg_.control.mem_read;
        
        if (can_forward_from_exmem) {
            uint64_t forward_data = ex_mem_reg_.control.jump ? 
                                   ex_mem_reg_.link_address : 
                                   ex_mem_reg_.alu_result;
            
            if (rs1_needed && 
                ex_mem_reg_.rd_idx == id_ex_reg_.rs1_idx &&
                (exmem_dest_is_fp == id_ex_reg_.rs1_is_fp)) {
                alu_input1 = forward_data;
            }
            if (rs2_alu_needed && 
                ex_mem_reg_.rd_idx == id_ex_reg_.rs2_idx &&
                (exmem_dest_is_fp == id_ex_reg_.rs2_is_fp)) {
                alu_input2 = forward_data;
            }
            if (rs2_store_needed && 
                ex_mem_reg_.rd_idx == id_ex_reg_.rs2_idx &&
                (exmem_dest_is_fp == id_ex_reg_.rs2_is_fp)) {
                store_val = forward_data;
            }
            if (rs3_needed && 
                ex_mem_reg_.rd_idx == id_ex_reg_.rs3_idx &&
                (exmem_dest_is_fp == id_ex_reg_.rs3_is_fp)) {
                alu_input3 = forward_data;
            }
        }
    }

    // MEM/WB FORWARDING
    bool mem_wb_writes_fp = check_if_instruction_writes_fp(mem_wb_reg_);
    bool memwb_dest_is_fp = mem_wb_reg_.is_fp_instr && mem_wb_writes_fp;
    bool memwb_dest_is_gpr = (!mem_wb_reg_.is_fp_instr) || (mem_wb_reg_.is_fp_instr && !mem_wb_writes_fp);
    bool memwb_zero_dest = (memwb_dest_is_gpr && mem_wb_reg_.rd_idx == 0);

    if (mem_wb_reg_.valid && 
        mem_wb_reg_.control.reg_write && 
        !memwb_zero_dest) {
        
        uint64_t forward_data;
        if (mem_wb_reg_.control.jump) {
            forward_data = mem_wb_reg_.link_address;
        } else if (mem_wb_reg_.control.mem_to_reg) {
            forward_data = mem_wb_reg_.mem_read_data;
        } else {
            forward_data = mem_wb_reg_.alu_result;
        }
        
        bool exmem_forwarded_to_rs1 = (ex_mem_reg_.valid && 
                                       ex_mem_reg_.control.reg_write && 
                                       !ex_mem_reg_.control.mem_read &&
                                       ex_mem_reg_.rd_idx == id_ex_reg_.rs1_idx &&
                                       (exmem_dest_is_fp == id_ex_reg_.rs1_is_fp));
        
        bool exmem_forwarded_to_rs2 = (ex_mem_reg_.valid && 
                                       ex_mem_reg_.control.reg_write && 
                                       ex_mem_reg_.rd_idx == id_ex_reg_.rs2_idx &&
                                       !ex_mem_reg_.control.mem_read &&
                                       (exmem_dest_is_fp == id_ex_reg_.rs2_is_fp));
        
        bool exmem_forwarded_to_rs3 = (ex_mem_reg_.valid && 
                                       ex_mem_reg_.control.reg_write && 
                                       ex_mem_reg_.rd_idx == id_ex_reg_.rs3_idx &&
                                       !ex_mem_reg_.control.mem_read &&
                                       (exmem_dest_is_fp == id_ex_reg_.rs3_is_fp));
        
        if (rs1_needed && 
            mem_wb_reg_.rd_idx == id_ex_reg_.rs1_idx && 
            (memwb_dest_is_fp == id_ex_reg_.rs1_is_fp) &&
            !exmem_forwarded_to_rs1) {
            alu_input1 = forward_data;
        }
        if (rs2_alu_needed && 
            mem_wb_reg_.rd_idx == id_ex_reg_.rs2_idx && 
            (memwb_dest_is_fp == id_ex_reg_.rs2_is_fp) &&
            !exmem_forwarded_to_rs2) {
            alu_input2 = forward_data;
        }
        if (rs2_store_needed && 
            mem_wb_reg_.rd_idx == id_ex_reg_.rs2_idx && 
            (memwb_dest_is_fp == id_ex_reg_.rs2_is_fp) &&
            !exmem_forwarded_to_rs2) {
            store_val = forward_data;
        }
        if (rs3_needed && 
            mem_wb_reg_.rd_idx == id_ex_reg_.rs3_idx && 
            (memwb_dest_is_fp == id_ex_reg_.rs3_is_fp) &&
            !exmem_forwarded_to_rs3) {
            alu_input3 = forward_data;
        }
    }

    next_ex_mem_reg_.rs2_val = store_val;

    uint8_t opcode = id_ex_reg_.instruction & 0b1111111;
    if (opcode == 0b0110111) {
        next_ex_mem_reg_.alu_result = id_ex_reg_.imm;
    } else {
        const bool is_memory_op = id_ex_reg_.control.mem_read || id_ex_reg_.control.mem_write;
        const bool use_fp_alu = id_ex_reg_.is_fp_instr && !is_memory_op;

        if (use_fp_alu) {
            uint8_t rm = id_ex_reg_.funct3;
            if (rm == 0b111) {
                rm = static_cast<uint8_t>(registers_.ReadCsr(0x002) & 0x7);
            }

            const uint64_t rs3_value = id_ex_reg_.has_rs3 ? alu_input3 : 0;
            uint8_t fcsr_status = 0;

            if (instruction_set::isDInstruction(id_ex_reg_.instruction)) {
                std::tie(next_ex_mem_reg_.alu_result, fcsr_status) = alu::Alu::dfpexecute(
                    id_ex_reg_.alu_operation,
                    alu_input1,
                    alu_input2,
                    rs3_value,
                    rm
                );
            } else {
                std::tie(next_ex_mem_reg_.alu_result, fcsr_status) = alu::Alu::fpexecute(
                    id_ex_reg_.alu_operation,
                    alu_input1,
                    alu_input2,
                    rs3_value,
                    rm
                );
            }
            registers_.WriteCsr(0x003, fcsr_status);
        } else {
            auto execution = alu::Alu::execute(
                id_ex_reg_.alu_operation, 
                alu_input1, 
                alu_input2
            );
            next_ex_mem_reg_.alu_result = execution.first;
        }
    }

    next_ex_mem_reg_.alu_zero = (next_ex_mem_reg_.alu_result == 0);

    if (id_ex_reg_.control.jump) {
        uint64_t actual_target;
        if (static_cast<uint8_t>(id_ex_reg_.instruction & 0b1111111) == 
            instruction_set::get_instr_encoding(instruction_set::Instruction::kjalr).opcode) {
            actual_target = next_ex_mem_reg_.alu_result & ~1ULL;
        } else {
            actual_target = next_ex_mem_reg_.alu_result;
        }

        bool predicted_taken = id_ex_reg_.predicted_taken;
        bool mispredicted = (!predicted_taken) || (id_ex_reg_.predicted_target != actual_target);

        if (mispredicted) {
            branch_taken_ = true;
            branch_target_ = actual_target;
            branch_mispredicted_ = true;
            next_if_id_reg_.valid = false;
            next_id_ex_reg_.valid = false;
        }
    }
    else if (id_ex_reg_.control.branch) {
        bool take_branch = false;
        switch (id_ex_reg_.funct3) {
            case 0b000: take_branch = (next_ex_mem_reg_.alu_result == 0); break;
            case 0b001: take_branch = (next_ex_mem_reg_.alu_result != 0); break;
            case 0b100: take_branch = (static_cast<int64_t>(next_ex_mem_reg_.alu_result) < 0); break;
            case 0b101: take_branch = (static_cast<int64_t>(next_ex_mem_reg_.alu_result) >= 0); break;
            case 0b110: take_branch = (alu_input1 < alu_input2); break;
            case 0b111: take_branch = (alu_input1 >= alu_input2); break;
        }

        bool predicted_taken = id_ex_reg_.predicted_taken;
        uint64_t actual_target = id_ex_reg_.pc + id_ex_reg_.imm;
        bool mispredicted = (predicted_taken != take_branch);

        if (predictor_mode_ == PredictorMode::kDynamic) {
             uint8_t counter;
             auto it = branch_predictor_.find(id_ex_reg_.pc);
             if (it != branch_predictor_.end()) counter = it->second;
             else {
                 counter = 1;
                 branch_predictor_[id_ex_reg_.pc] = counter;
             }
             
             if (take_branch) { if (counter < 3) counter++; } 
             else { if (counter > 0) counter--; }
             branch_predictor_[id_ex_reg_.pc] = counter;
        }

        if (mispredicted) {
            branch_taken_ = true;
            branch_mispredicted_ = true;
            branch_target_ = take_branch ? actual_target : (id_ex_reg_.pc + 4);
            next_if_id_reg_.valid = false;
            next_id_ex_reg_.valid = false;
        } else {
            branch_mispredicted_ = false;
        }
    }
}

// STAGE 2: Instruction Decode
void RiscV5StageVM::DecodeStage() {
    next_id_ex_reg_ = {};
    if (!if_id_reg_.valid) return;
    if (!if_id_write_) {
        next_id_ex_reg_ = {};
        return;
    }
    if (branch_taken_) {
        next_id_ex_reg_ = {};
        return;
    }

    uint32_t instruction = if_id_reg_.instruction;
    uint8_t opcode = instruction & 0b1111111;

    control_unit_.SetControlSignals(instruction);

    next_id_ex_reg_.pc = if_id_reg_.pc;
    next_id_ex_reg_.link_address = if_id_reg_.pc + 4;
    next_id_ex_reg_.control = control_unit_.GetSignals();
    next_id_ex_reg_.valid = true;

    next_id_ex_reg_.rs1_idx = (instruction >> 15) & 0b11111;
    next_id_ex_reg_.rs2_idx = (instruction >> 20) & 0b11111;
    next_id_ex_reg_.rd_idx = (instruction >> 7) & 0b11111;
    next_id_ex_reg_.funct3 = (instruction >> 12) & 0b111;
    next_id_ex_reg_.imm = ImmGenerator(instruction);
    next_id_ex_reg_.instruction = instruction;
    
    bool is_fp = instruction_set::isFInstruction(instruction) ||
                 instruction_set::isDInstruction(instruction);
    bool is_fma = (opcode == 0b1000011 || opcode == 0b1000111 ||
                   opcode == 0b1001011 || opcode == 0b1001111);
    next_id_ex_reg_.is_fp_instr = is_fp || is_fma;
    
    if (opcode == 0b1000011 || opcode == 0b1000111 || 
        opcode == 0b1001011 || opcode == 0b1001111) {
        next_id_ex_reg_.has_rs3 = true;
        next_id_ex_reg_.rs3_idx = (instruction >> 27) & 0b11111;
        next_id_ex_reg_.rs3_is_fp = true;
        next_id_ex_reg_.rs3_val = registers_.ReadFpr(next_id_ex_reg_.rs3_idx);
    } else {
        next_id_ex_reg_.has_rs3 = false;
        next_id_ex_reg_.rs3_idx = 0;
        next_id_ex_reg_.rs3_is_fp = false;
        next_id_ex_reg_.rs3_val = 0;
    }
    
    if (next_id_ex_reg_.is_fp_instr) {
        if (opcode == 0b0000111) { // FLW/FLD
            next_id_ex_reg_.rs1_is_fp = false;
            next_id_ex_reg_.rs2_is_fp = false;
            next_id_ex_reg_.rs1_val = registers_.ReadGpr(next_id_ex_reg_.rs1_idx);
            next_id_ex_reg_.rs2_val = 0;
        } else if (opcode == 0b0100111) { // FSW/FSD
            next_id_ex_reg_.rs1_is_fp = false;
            next_id_ex_reg_.rs2_is_fp = true;
            next_id_ex_reg_.rs1_val = registers_.ReadGpr(next_id_ex_reg_.rs1_idx);
            next_id_ex_reg_.rs2_val = registers_.ReadFpr(next_id_ex_reg_.rs2_idx);
        } else {
            next_id_ex_reg_.rs1_is_fp = true;
            next_id_ex_reg_.rs2_is_fp = true;

            if (opcode == 0b1010011) {
                uint32_t funct5 = (instruction >> 27) & 0x1F;
                if (funct5 == 0b11010 || funct5 == 0b11011 || funct5 == 0b11110) {
                    next_id_ex_reg_.rs1_is_fp = false;
                }
            }

            if (next_id_ex_reg_.rs1_is_fp) next_id_ex_reg_.rs1_val = registers_.ReadFpr(next_id_ex_reg_.rs1_idx);
            else next_id_ex_reg_.rs1_val = registers_.ReadGpr(next_id_ex_reg_.rs1_idx);
            
            next_id_ex_reg_.rs2_val = registers_.ReadFpr(next_id_ex_reg_.rs2_idx);
        }
    } else {
        next_id_ex_reg_.rs1_is_fp = false;
        next_id_ex_reg_.rs2_is_fp = false;
        next_id_ex_reg_.rs1_val = registers_.ReadGpr(next_id_ex_reg_.rs1_idx);
        next_id_ex_reg_.rs2_val = registers_.ReadGpr(next_id_ex_reg_.rs2_idx);
    }

    next_id_ex_reg_.alu_operation = control_unit_.GetAluSignal(instruction, control_unit_.GetAluOp());

    next_id_ex_reg_.has_predicted = false;
    next_id_ex_reg_.predicted_taken = false;
    next_id_ex_reg_.predicted_target = 0;

    bool new_has_predicted = false;
    bool new_predicted_taken = false;
    uint64_t new_predicted_target = 0;

    if (if_id_reg_.valid) {
        if (next_id_ex_reg_.control.jump) {
            new_has_predicted = true;
            new_predicted_taken = true;
            if (opcode == instruction_set::get_instr_encoding(instruction_set::Instruction::kjalr).opcode) {
                new_predicted_target = (next_id_ex_reg_.rs1_val + next_id_ex_reg_.imm) & ~1ULL;
            } else {
                new_predicted_target = next_id_ex_reg_.pc + next_id_ex_reg_.imm;
            }
        } else if (next_id_ex_reg_.control.branch) {
            new_has_predicted = true;
            if (predictor_mode_ == PredictorMode::kDynamic) {
                uint8_t counter;
                auto it = branch_predictor_.find(next_id_ex_reg_.pc);
                if (it != branch_predictor_.end()) counter = it->second;
                else {
                    counter = 1;
                    branch_predictor_[next_id_ex_reg_.pc] = counter;
                }
                new_predicted_taken = (counter >= 2);
            } else {
                new_predicted_taken = false;
            }
            new_predicted_target = next_id_ex_reg_.pc + next_id_ex_reg_.imm;
        }

        if (new_has_predicted) {
            next_id_ex_reg_.has_predicted = true;
            next_id_ex_reg_.predicted_taken = new_predicted_taken;
            next_id_ex_reg_.predicted_target = new_predicted_target;
            
            has_predicted_dec_ = true;
            predicted_taken_dec_ = new_predicted_taken;
            predicted_target_dec_ = new_predicted_target;
        }
    }
}

// STAGE 1: Instruction Fetch
void RiscV5StageVM::FetchStage() {
    next_if_id_reg_ = {};

    if (program_counter_ >= program_size_) {
         next_if_id_reg_.valid = false;
         return;
    }

    if (program_counter_ >= memory_controller_.GetMemorySize()) {
         next_if_id_reg_.valid = false;
         return;
    }

    if (!pc_write_) {
        next_if_id_reg_ = if_id_reg_;
        return;
    }

    if (has_predicted_dec_ && predicted_taken_dec_) {
        next_if_id_reg_.valid = false;
        return;
    }

    next_if_id_reg_.instruction = memory_controller_.ReadWord(program_counter_);
    next_if_id_reg_.pc = program_counter_;
    next_if_id_reg_.valid = true;

    if (branch_taken_) {
        next_if_id_reg_.valid = false;
    }
}

// STAGE 2.5: Hazard Detection
void RiscV5StageVM::DetectAndHandleHazards() {
    pc_write_ = true;
    if_id_write_ = true;

    if (!if_id_reg_.valid) return;
    
    uint32_t next_instr = if_id_reg_.instruction;
    uint8_t next_opcode = next_instr & 0b1111111;
    uint8_t id_rs1 = (next_instr >> 15) & 0b11111;
    uint8_t id_rs2 = (next_instr >> 20) & 0b11111;
    uint8_t id_rs3 = 0;
    bool next_uses_rs3 = false;

    bool next_rs1_is_fp = false;
    bool next_rs2_is_fp = false;
    bool next_rs3_is_fp = false; 

    if (next_opcode == 0b1000011 || next_opcode == 0b1000111 ||
        next_opcode == 0b1001011 || next_opcode == 0b1001111) {
        next_uses_rs3 = true;
        id_rs3 = (next_instr >> 27) & 0b11111;
        next_rs3_is_fp = true;
    }

    bool next_is_f = instruction_set::isFInstruction(next_instr) || instruction_set::isDInstruction(next_instr);
    bool next_is_fma = (next_opcode == 0b1000011 || next_opcode == 0b1000111 ||
                        next_opcode == 0b1001011 || next_opcode == 0b1001111);

    if (next_is_fma) {
        next_rs1_is_fp = true;
        next_rs2_is_fp = true;
    } else if (next_is_f) {
        if (next_opcode == 0b0000111) {
            next_rs1_is_fp = false;
            next_rs2_is_fp = false;
        } else if (next_opcode == 0b0100111) {
            next_rs1_is_fp = false;
            next_rs2_is_fp = true;
        } else {
            next_rs1_is_fp = true;
            next_rs2_is_fp = true;
            if (next_opcode == 0b1010011) {
                 uint32_t funct5 = (next_instr >> 27) & 0x1F;
                 if (funct5 == 0b11010 || funct5 == 0b11011 || funct5 == 0b11110) {
                     next_rs1_is_fp = false;
                 }
            }
        }
    } else {
        next_rs1_is_fp = false;
        next_rs2_is_fp = false;
    }

    bool load_writes_zero = (!id_ex_reg_.is_fp_instr && id_ex_reg_.rd_idx == 0);
    if (id_ex_reg_.valid && id_ex_reg_.control.mem_read && !load_writes_zero) {
        bool hazard_detected = false;
        bool id_ex_writes_fp = id_ex_reg_.is_fp_instr;
        
        if (id_ex_writes_fp) {
            if ((next_rs1_is_fp && id_ex_reg_.rd_idx == id_rs1) ||
                (next_rs2_is_fp && id_ex_reg_.rd_idx == id_rs2) ||
                (next_uses_rs3 && next_rs3_is_fp && id_ex_reg_.rd_idx == id_rs3)) {
                hazard_detected = true;
            }
        } else {
            if ((!next_rs1_is_fp && id_rs1 != 0 && id_ex_reg_.rd_idx == id_rs1) ||
                (!next_rs2_is_fp && id_rs2 != 0 && id_ex_reg_.rd_idx == id_rs2)) {
                hazard_detected = true;
            }
        }

        if (hazard_detected) {
            pc_write_ = false;
            if_id_write_ = false;
            stall_cycles_++;
            next_id_ex_reg_ = {};
            return;
        }
    }

    {
        bool next_is_jalr = (next_opcode == 0b1100111);
        if (next_is_jalr) {
            bool jalr_hazard = false;
            if (id_ex_reg_.valid && id_ex_reg_.control.reg_write &&
                id_ex_reg_.rd_idx == id_rs1 && id_rs1 != 0 && !next_rs1_is_fp) {
                jalr_hazard = true;
            }
            if (ex_mem_reg_.valid && ex_mem_reg_.control.reg_write &&
                ex_mem_reg_.rd_idx == id_rs1 && id_rs1 != 0 && !next_rs1_is_fp) {
                jalr_hazard = true;
            }
            if (jalr_hazard) {
                pc_write_ = false;
                if_id_write_ = false;
                stall_cycles_++;
                next_id_ex_reg_ = {};
                return;
            }
        }
    }
}

void RiscV5StageVM::DebugPrintCycle() const {
    std::cout << "\n========== Cycle " << cycle_s_ << " ==========\n";
    std::cout << "PC: 0x" << std::hex << program_counter_ << std::dec << "\n";
    
    std::cout << "\nIF/ID: ";
    if (if_id_reg_.valid) {
        std::cout << "PC=0x" << std::hex << if_id_reg_.pc 
                  << " Instr=0x" << if_id_reg_.instruction << std::dec;
    } else {
        std::cout << "BUBBLE";
    }
    
    std::cout << "\nID/EX: ";
    if (id_ex_reg_.valid) {
        std::cout << "PC=0x" << std::hex << id_ex_reg_.pc << std::dec
                  << " rd=x" << (int)id_ex_reg_.rd_idx
                  << " rs1=x" << (int)id_ex_reg_.rs1_idx << "(" << id_ex_reg_.rs1_val << ")"
                  << " rs2=x" << (int)id_ex_reg_.rs2_idx << "(" << id_ex_reg_.rs2_val << ")";
        if (id_ex_reg_.has_rs3) {
            std::cout << " rs3=x" << (int)id_ex_reg_.rs3_idx << "(" << id_ex_reg_.rs3_val << ")";
        }
    } else {
        std::cout << "BUBBLE";
    }
    
    std::cout << "\nEX/MEM: ";
    if (ex_mem_reg_.valid) {
        std::cout << "PC=0x" << std::hex << ex_mem_reg_.pc << std::dec
                  << " rd=x" << (int)ex_mem_reg_.rd_idx
                  << " ALU=" << ex_mem_reg_.alu_result
                  << " rs2_val=" << ex_mem_reg_.rs2_val;
    } else {
        std::cout << "BUBBLE";
    }
    
    std::cout << "\nMEM/WB: ";
    if (mem_wb_reg_.valid) {
        std::cout << "PC=0x" << std::hex << mem_wb_reg_.pc << std::dec
                  << " rd=x" << (int)mem_wb_reg_.rd_idx
                  << " ALU=" << mem_wb_reg_.alu_result
                  << " MEM=" << mem_wb_reg_.mem_read_data;
    } else {
        std::cout << "BUBBLE";
    }
    
    std::cout << "\n\nHazard Flags: pc_write=" << pc_write_ 
              << " if_id_write=" << if_id_write_
              << " branch_taken=" << branch_taken_;
    if (branch_taken_) {
        std::cout << " target=0x" << std::hex << branch_target_ << std::dec;
    }
    
    std::cout << "\nCounters: Stalls=" << stall_cycles_ 
              << " Retired=" << instructions_retired_
              << " Mispredictions=" << branch_mispredictions_;
    std::cout << "\n========================================\n";
}

void RiscV5StageVM::DumpPipelineState() const {
    std::cout << GetPipelineStateString();
}

std::string RiscV5StageVM::GetPipelineStateString() const {
    std::ostringstream oss;
    
    oss << "Pipeline State at Cycle " << cycle_s_ << ":\n";
    oss << "  PC: 0x" << std::hex << program_counter_ << std::dec << "\n";
    oss << "  Instructions Retired: " << instructions_retired_ << "\n";
    oss << "  Stall Cycles: " << stall_cycles_ << "\n";
    oss << "  Branch Mispredictions: " << branch_mispredictions_ << "\n";
    oss << "  CPI: " << std::fixed << std::setprecision(2) 
        << (instructions_retired_ > 0 ? (double)cycle_s_ / instructions_retired_ : 0.0) << "\n";
    
    oss << "\n  IF/ID: " << (if_id_reg_.valid ? "Valid" : "Bubble");
    if (if_id_reg_.valid) {
        oss << " [PC=0x" << std::hex << if_id_reg_.pc << ", Instr=0x" 
            << if_id_reg_.instruction << std::dec << "]";
    }
    
    oss << "\n  ID/EX: " << (id_ex_reg_.valid ? "Valid" : "Bubble");
    if (id_ex_reg_.valid) {
        oss << " [PC=0x" << std::hex << id_ex_reg_.pc << std::dec 
            << ", rd=x" << (int)id_ex_reg_.rd_idx 
            << ", rs1=x" << (int)id_ex_reg_.rs1_idx
            << ", rs2=x" << (int)id_ex_reg_.rs2_idx;
        if (id_ex_reg_.has_rs3) {
            oss << ", rs3=x" << (int)id_ex_reg_.rs3_idx;
        }
        oss << "]";
    }
    
    oss << "\n  EX/MEM: " << (ex_mem_reg_.valid ? "Valid" : "Bubble");
    if (ex_mem_reg_.valid) {
        oss << " [PC=0x" << std::hex << ex_mem_reg_.pc << std::dec 
            << ", rd=x" << (int)ex_mem_reg_.rd_idx 
            << ", Result=" << ex_mem_reg_.alu_result << "]";
    }
    
    oss << "\n  MEM/WB: " << (mem_wb_reg_.valid ? "Valid" : "Bubble");
    if (mem_wb_reg_.valid) {
        oss << " [PC=0x" << std::hex << mem_wb_reg_.pc << std::dec 
            << ", rd=x" << (int)mem_wb_reg_.rd_idx 
            << ", ALU=" << mem_wb_reg_.alu_result 
            << ", MEM=" << mem_wb_reg_.mem_read_data << "]";
    }
    
    oss << "\n";
    return oss.str();
}

void RiscV5StageVM::DebugRun() {
    SetDebugMode(true);
    stop_requested_ = false;
    
    std::cout << "\n===== Starting Debug Run =====\n";
    std::cout << "Press Enter to step through each cycle, or 'q' to quit\n\n";
    
    auto pipeline_not_empty = [&]() {
        return if_id_reg_.valid ||
               id_ex_reg_.valid ||
               ex_mem_reg_.valid ||
               mem_wb_reg_.valid;
    };

    while (!stop_requested_ && 
           (program_counter_ < program_size_ || pipeline_not_empty())) {
        ClockTick();
        
        std::string input;
        std::getline(std::cin, input);
        if (input == "q" || input == "quit") {
            break;
        }
    }
    
    SetDebugMode(false);
    
    if (program_counter_ >= program_size_ && !pipeline_not_empty()) {
        std::cout << "\nVM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    
    std::cout << "\n===== Final Statistics =====\n";
    DumpPipelineState();
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RiscV5StageVM::Step() {
    ClockTick();
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RiscV5StageVM::Undo() {
    std::cout << "Undo for 5-stage pipeline is not yet implemented." << std::endl;
}

void RiscV5StageVM::Redo() {
    std::cout << "Redo for 5-stage pipeline is not yet implemented." << std::endl;
}

void RiscV5StageVM::Reset() {
    program_counter_ = 0;
    instructions_retired_ = 0;
    cycle_s_ = 0;
    stall_cycles_ = 0;
    registers_.Reset();
    memory_controller_.Reset();
    control_unit_.Reset();

    if_id_reg_ = {};
    id_ex_reg_ = {};
    ex_mem_reg_ = {};
    mem_wb_reg_ = {};
    next_if_id_reg_ = {};
    next_id_ex_reg_ = {};
    next_ex_mem_reg_ = {};
    next_mem_wb_reg_ = {};

    branch_taken_ = false;
    branch_target_ = 0;
    pc_write_ = true;
    if_id_write_ = true;
    branch_mispredictions_ = 0;

    branch_predictor_.clear();
    has_predicted_dec_ = false;
    predicted_taken_dec_ = false;
    predicted_target_dec_ = 0;
    branch_mispredicted_ = false;
}
