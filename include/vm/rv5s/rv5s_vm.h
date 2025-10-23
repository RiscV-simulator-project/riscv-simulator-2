#ifndef RV5S_VM_H
#define RV5S_VM_H

#include "vm/vm_base.h"
#include "vm/rv5s/pipeline_registers.h"
#include "vm/rv5s/rv5s_control_unit.h"
#include "vm/alu.h"

class RiscV5StageVM : public VmBase {
public:
    RiscV5StageVM();
    ~RiscV5StageVM() override = default;

    // VmBase Interface 
    void Run() override;
    void DebugRun() override;
    void Step() override;
    void Undo() override;
    void Redo() override;
    void Reset() override;

private:
    void ClockTick();

    // Pipeline stage functions
    void FetchStage();
    void DecodeStage();
    void ExecuteStage();
    void MemoryStage();
    void WriteBackStage();

    // Hazard detection and handling
    void DetectAndHandleHazards();

    // Pipeline registers
    IF_ID_Register if_id_reg_{};
    ID_EX_Register id_ex_reg_{};
    EX_MEM_Register ex_mem_reg_{};
    MEM_WB_Register mem_wb_reg_{};

    // Pipeline registers for the *next* clock cycle
    IF_ID_Register next_if_id_reg_{};
    ID_EX_Register next_id_ex_reg_{};
    EX_MEM_Register next_ex_mem_reg_{};
    MEM_WB_Register next_mem_wb_reg_{};

    // Architectural components
    RV5SControlUnit control_unit_{};
    alu::Alu alu_{};

    // State for stalling and flushing
    bool pc_write_ = true;
    bool if_id_write_ = true;

    // Branch prediction/handling state
    bool branch_taken_ = false;
    uint64_t branch_target_ = 0;
};

#endif // RV5S_VM_H