#include "vm/rv5s/rv5s_vm.h"
#include "common/instructions.h"
#include "utils.h"
#include "globals.h"

RiscV5StageVM::RiscV5StageVM() : VmBase() {
    // Constructor to Initialize any specific state for the 5-stage VM
    Reset();
}

RiscV5StageVM::~RiscV5StageVM() = default;

void RiscV5StageVM::Run() {
    stop_requested_ = false;
    while (!stop_requested_ && program_counter_ < program_size_) {
        ClockTick();
    }
    if (program_counter_ >= program_size_) {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RiscV5StageVM::ClockTick() {
    // --- Execute pipeline stages in reverse order ---
    // This prevents an instruction from racing through multiple stages in one cycle.
    // Each stage uses the register state from the *previous* cycle.

    DetectAndHandleHazards();

    WriteBackStage();
    MemoryStage();
    ExecuteStage();
    DecodeStage();
    FetchStage();

    // --- Latch new values for the next cycle ---
    // This simulates the clock edge where all registers are updated simultaneously.
    if (if_id_write_) if_id_reg_ = next_if_id_reg_;
    // Stalls in ID/EX are handled by injecting bubbles
    id_ex_reg_ = next_id_ex_reg_; // Stalls in ID/EX are handled by injecting bubbles
    ex_mem_reg_ = next_ex_mem_reg_;
    mem_wb_reg_ = next_mem_wb_reg_;
    
    // --- PC Update Logic ---
    // Handle PC update based on whether a branch was taken in the EX stage
    if (branch_taken_) {
        program_counter_ = branch_target_;
        // Flush the IF and ID stages by invalidating their registers
        if_id_reg_.valid = false;
        id_ex_reg_.valid = false;
        branch_taken_ = false; // Reset for the next cycle
    } else {
        program_counter_ += 4;
    }

    cycle_s_++;
    // Note: instructions_retired_ should only be incremented in the WB stage.
}


// STAGE 5: Write-Back
void RiscV5StageVM::WriteBackStage() {
    if (!mem_wb_reg_.valid) return;

    if (mem_wb_reg_.control.reg_write && mem_wb_reg_.rd_idx != 0) {
        uint64_t data_to_write;
        if (mem_wb_reg_.control.jump) {
            // For JAL/JALR, we write the link address (PC+4)
            data_to_write = mem_wb_reg_.link_address;
        } else if (mem_wb_reg_.control.mem_to_reg) {
            data_to_write = mem_wb_reg_.mem_read_data;
        } else {
            data_to_write = mem_wb_reg_.alu_result;
        }
        registers_.WriteGpr(mem_wb_reg_.rd_idx, data_to_write);
    }
    instructions_retired_++;
}

// STAGE 4: Memory Access
void RiscV5StageVM::MemoryStage() {
    // Default pass-through
    next_mem_wb_reg_ = {}; // Clear previous data
    if (!ex_mem_reg_.valid) return;

    // Pass-through values from EX/MEM to MEM/WB
    next_mem_wb_reg_.pc = ex_mem_reg_.pc;
    next_mem_wb_reg_.link_address = ex_mem_reg_.link_address;
    next_mem_wb_reg_.alu_result = ex_mem_reg_.alu_result;
    next_mem_wb_reg_.rd_idx = ex_mem_reg_.rd_idx;
    next_mem_wb_reg_.control = ex_mem_reg_.control;
    // Removed redundant line: next_mem_wb_reg_.control = ex_mem_reg_.control;
    next_mem_wb_reg_.valid = true;

    // Perform memory operations
    if (ex_mem_reg_.control.mem_read) {
        switch (ex_mem_reg_.funct3) {
            case 0b000: // LB
                next_mem_wb_reg_.mem_read_data = static_cast<int8_t>(memory_controller_.ReadByte(ex_mem_reg_.alu_result));
                break;
            case 0b001: // LH
                next_mem_wb_reg_.mem_read_data = static_cast<int16_t>(memory_controller_.ReadHalfWord(ex_mem_reg_.alu_result));
                break;
            case 0b010: // LW
                next_mem_wb_reg_.mem_read_data = static_cast<int32_t>(memory_controller_.ReadWord(ex_mem_reg_.alu_result));
                break;
            case 0b011: // LD
                next_mem_wb_reg_.mem_read_data = memory_controller_.ReadDoubleWord(ex_mem_reg_.alu_result);
                break;
            case 0b100: // LBU
                next_mem_wb_reg_.mem_read_data = memory_controller_.ReadByte(ex_mem_reg_.alu_result);
                break;
            case 0b101: // LHU
                next_mem_wb_reg_.mem_read_data = memory_controller_.ReadHalfWord(ex_mem_reg_.alu_result);
                break;
            case 0b110: // LWU
                next_mem_wb_reg_.mem_read_data = memory_controller_.ReadWord(ex_mem_reg_.alu_result);
                break;
        }
    }
    if (ex_mem_reg_.control.mem_write) {
        // Use funct3 to determine the correct store type
        switch (ex_mem_reg_.funct3) {
            case 0b000: // SB
                memory_controller_.WriteByte(ex_mem_reg_.alu_result, ex_mem_reg_.rs2_val & 0xFF);
                break;
            case 0b001: // SH
                memory_controller_.WriteHalfWord(ex_mem_reg_.alu_result, ex_mem_reg_.rs2_val & 0xFFFF);
                break;
            case 0b010: // SW
                memory_controller_.WriteWord(ex_mem_reg_.alu_result, ex_mem_reg_.rs2_val & 0xFFFFFFFF);
                break;
            case 0b011: // SD
                memory_controller_.WriteDoubleWord(ex_mem_reg_.alu_result, ex_mem_reg_.rs2_val);
                break;
        }
    }
}

// STAGE 3: Execute
void RiscV5StageVM::ExecuteStage() {
    // Default pass-through
    next_ex_mem_reg_ = {}; // Clear previous data
    if (!id_ex_reg_.valid) return;

    // Pass-through values from ID/EX to EX/MEM
    next_ex_mem_reg_.pc = id_ex_reg_.pc;
    next_ex_mem_reg_.link_address = id_ex_reg_.link_address;
    next_ex_mem_reg_.rs2_val = id_ex_reg_.rs2_val; // Needed for store instructions
    next_ex_mem_reg_.rd_idx = id_ex_reg_.rd_idx;
    next_ex_mem_reg_.funct3 = id_ex_reg_.funct3; // Pass funct3 forward (now correctly defined in struct)
    next_ex_mem_reg_.control = id_ex_reg_.control;
    next_ex_mem_reg_.valid = true;

    // Determine ALU inputs (will be modified by forwarding unit later)
    uint64_t alu_input1;

    // For AUIPC, JAL, and JALR, the first ALU input is the PC. For others, it's rs1.
    if (id_ex_reg_.control.pc_to_alu) { // Assumes a new 'pc_to_alu' signal from the control unit
        alu_input1 = id_ex_reg_.pc;
    } else {
        alu_input1 = id_ex_reg_.rs1_val;
    }
    uint64_t alu_input2 = id_ex_reg_.control.alu_src ? id_ex_reg_.imm : id_ex_reg_.rs2_val;

    // Perform ALU operation
    bool overflow; // Ignored for now
    std::tie(next_ex_mem_reg_.alu_result, overflow) = alu_.execute(id_ex_reg_.alu_operation, alu_input1, alu_input2);

    // Set zero flag for branches
    next_ex_mem_reg_.alu_zero = (next_ex_mem_reg_.alu_result == 0);

    // Handle unconditional jumps (JAL, JALR)
    if (id_ex_reg_.control.jump) {
        branch_taken_ = true;
        // For JAL, the target is alu_result (PC + imm).
        // For JALR, the target is (rs1_val + imm) & ~1ULL.
        if (id_ex_reg_.funct3 == 0) { // JALR instruction
            branch_target_ = next_ex_mem_reg_.alu_result & ~1ULL;
        } else { // JAL instruction
            branch_target_ = next_ex_mem_reg_.alu_result;
        }
    }
    // Handle conditional branches
    else if (id_ex_reg_.control.branch) {
        bool take_branch = false;
        switch (id_ex_reg_.funct3) {
            case 0b000: // BEQ (Branch if Equal)
                take_branch = (next_ex_mem_reg_.alu_result == 0); // rs1 == rs2 implies rs1 - rs2 == 0
                break;
            case 0b001: // BNE (Branch if Not Equal)
                take_branch = (next_ex_mem_reg_.alu_result != 0); // rs1 != rs2 implies rs1 - rs2 != 0
                break;
            case 0b100: // BLT (Branch if Less Than, signed)
                take_branch = ((int64_t)next_ex_mem_reg_.alu_result < 0); // rs1 < rs2 implies rs1 - rs2 < 0
                break;
            case 0b101: // BGE (Branch if Greater than or Equal, signed)
                take_branch = ((int64_t)next_ex_mem_reg_.alu_result >= 0); // rs1 >= rs2 implies rs1 - rs2 >= 0
                break;
            case 0b110: // BLTU (Branch if Less Than, unsigned)
                take_branch = (id_ex_reg_.rs1_val < id_ex_reg_.rs2_val); // Direct comparison of original values
                break;
            case 0b111: // BGEU (Branch if Greater than or Equal, unsigned)
                take_branch = (id_ex_reg_.rs1_val >= id_ex_reg_.rs2_val); // Direct comparison of original values
                break;
        }
        
        if (take_branch) {
            branch_taken_ = true;
            // The target address is PC + immediate for branches
            branch_target_ = id_ex_reg_.pc + id_ex_reg_.imm;
        }
    }
}

// STAGE 2: Instruction Decode
void RiscV5StageVM::DecodeStage() {
    // Default pass-through
    next_id_ex_reg_ = {}; // Clear previous data
    if (!if_id_reg_.valid) return;

    uint32_t instruction = if_id_reg_.instruction;

    // Generate control signals for this instruction
    control_unit_.SetControlSignals(instruction);

    // Pass values to the next pipeline register
    next_id_ex_reg_.pc = if_id_reg_.pc;
    next_id_ex_reg_.link_address = if_id_reg_.pc + 4; // Pre-calculate link address
    next_id_ex_reg_.control = control_unit_.GetSignals(); // Get a copy of the signals
    next_id_ex_reg_.valid = true;

    // Extract register numbers and immediate
    next_id_ex_reg_.rs1_idx = (instruction >> 15) & 0b11111;
    next_id_ex_reg_.rs2_idx = (instruction >> 20) & 0b11111;
    next_id_ex_reg_.rd_idx = (instruction >> 7) & 0b11111;
    next_id_ex_reg_.funct3 = (instruction >> 12) & 0b111;
    next_id_ex_reg_.imm = ImmGenerator(instruction);

    next_id_ex_reg_.instruction = instruction; // Pass instruction for JALR check
    // Read from register file
    next_id_ex_reg_.rs1_val = registers_.ReadGpr(next_id_ex_reg_.rs1_idx);
    next_id_ex_reg_.rs2_val = registers_.ReadGpr(next_id_ex_reg_.rs2_idx);

    // Pre-compute ALU operation for the EX stage
    next_id_ex_reg_.alu_operation = control_unit_.GetAluSignal(instruction, control_unit_.GetAluOp());
}


// STAGE 1: Instruction Fetch
void RiscV5StageVM::FetchStage() {
    // Default pass-through
    next_if_id_reg_ = {}; // Clear previous data

    // Fetch instruction from memory at the current PC
    next_if_id_reg_.instruction = memory_controller_.ReadWord(program_counter_);
    next_if_id_reg_.pc = program_counter_;
    next_if_id_reg_.valid = true;
}

// STAGE 2.5: Hazard Detection
void RiscV5StageVM::DetectAndHandleHazards() {
    // By default, no stalls or flushes
    pc_write_ = true;
    if_id_write_ = true;
    // We control stalls in ID/EX by injecting a "bubble" (invalid instruction)
    // into the next_id_ex_reg rather than preventing the write.

    // --- Load-Use Hazard Detection (RAW) ---
    // Occurs when an instruction in ID needs a register that a load in EX is reading from memory.
    // We must stall the pipeline for one cycle.
    uint8_t id_rs1 = (if_id_reg_.instruction >> 15) & 0b11111;
    uint8_t id_rs2 = (if_id_reg_.instruction >> 20) & 0b11111;

    if (id_ex_reg_.valid && id_ex_reg_.control.mem_read &&
        (id_ex_reg_.rd_idx != 0) &&
        (id_ex_reg_.rd_idx == id_rs1 || id_ex_reg_.rd_idx == id_rs2))
    {
        // Stall detected!
        pc_write_ = false;    // Don't fetch the next instruction (re-fetch current one)
        if_id_write_ = false; // Don't let the new (and incorrect) instruction into the ID stage

        // Inject a bubble (NOP) into the ID/EX register to prevent the stalled instruction from proceeding
        next_id_ex_reg_ = {}; // An empty struct with valid=false is a bubble
    }

    // --- Data Forwarding (from EX/MEM and MEM/WB stages) ---
    // This logic should be in the ExecuteStage to modify ALU inputs, but is shown here for clarity.
    // For simplicity, we will modify the `next_id_ex_reg_` values before they are used in Execute.
    // This is conceptually equivalent to a forwarding unit just before the ALU.

    // Forward from EX/MEM stage (higher priority)
    if (ex_mem_reg_.valid && ex_mem_reg_.control.reg_write && (ex_mem_reg_.rd_idx != 0)) {
        if (ex_mem_reg_.rd_idx == next_id_ex_reg_.rs1_idx) {
            next_id_ex_reg_.rs1_val = ex_mem_reg_.alu_result;
        }
        if (ex_mem_reg_.rd_idx == next_id_ex_reg_.rs2_idx) {
            next_id_ex_reg_.rs2_val = ex_mem_reg_.alu_result;
        }
    }

    // Forward from MEM/WB stage (lower priority)
    if (mem_wb_reg_.valid && mem_wb_reg_.control.reg_write && (mem_wb_reg_.rd_idx != 0)) {
        // Only forward if not already forwarded from EX/MEM
        if (mem_wb_reg_.rd_idx == next_id_ex_reg_.rs1_idx && !(ex_mem_reg_.valid && ex_mem_reg_.control.reg_write && ex_mem_reg_.rd_idx == next_id_ex_reg_.rs1_idx)) {
            // Determine what data to forward
            next_id_ex_reg_.rs1_val = mem_wb_reg_.control.mem_to_reg ? mem_wb_reg_.mem_read_data : mem_wb_reg_.alu_result;
        }
        if (mem_wb_reg_.rd_idx == next_id_ex_reg_.rs2_idx && !(ex_mem_reg_.valid && ex_mem_reg_.control.reg_write && ex_mem_reg_.rd_idx == next_id_ex_reg_.rs2_idx)) {
            next_id_ex_reg_.rs2_val = mem_wb_reg_.control.mem_to_reg ? mem_wb_reg_.mem_read_data : mem_wb_reg_.alu_result;
        }
    }
}

// --- Placeholder implementations for the rest of the VmBase interface ---

void RiscV5StageVM::DebugRun() {
    // TODO: Implement debug run for pipeline (will involve single clock ticks)
    std::cout << "DebugRun for 5-stage pipeline is not yet implemented." << std::endl;
    Run(); // Fallback to normal run for now
}

void RiscV5StageVM::Step() {
    // TODO: Implement single clock tick step
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
    registers_.Reset();
    memory_controller_.Reset();
    // memory_controller_.Reset(); // Assuming this resets memory content
    control_unit_.Reset();

    // Clear all pipeline registers
    if_id_reg_ = {};
    id_ex_reg_ = {};
    ex_mem_reg_ = {};
    mem_wb_reg_ = {};
    next_if_id_reg_ = {};
    next_id_ex_reg_ = {};
    next_ex_mem_reg_ = {};
    next_mem_wb_reg_ = {};

    // Reset branch state
    branch_taken_ = false;
    branch_target_ = 0;
}