#ifndef RV5S_VM_H
#define RV5S_VM_H

#include "vm/vm_base.h"
#include "vm/rv5s/pipeline_registers.h"
#include "vm/rv5s/rv5s_control_unit.h"
#include "vm/alu.h"
#include <string>
#include <unordered_map>

enum class PredictorMode {
    kStatic,
    kDynamic
};

class RiscV5StageVM : public VmBase {
public:
    RiscV5StageVM();
    ~RiscV5StageVM() = default;

    // Configuration
    void SetHazardDetection(bool enabled) { hazard_detection_enabled_ = enabled; }
    bool GetHazardDetection() const { return hazard_detection_enabled_; }
    
    // Branch Predictor Configuration
    void SetPredictorMode(PredictorMode mode) { predictor_mode_ = mode; }
    PredictorMode GetPredictorMode() const { return predictor_mode_; }

    // Debug mode control
    void SetDebugMode(bool enabled) { debug_mode_ = enabled; }
    bool GetDebugMode() const { return debug_mode_; }

    // VmBase Interface 
    void Run() override;
    void DebugRun() override;
    void Step() override;
    void Undo() override;
    void Redo() override;
    void Reset() override;
    
    // Pipeline state inspection
    void DumpPipelineState() const;
    std::string GetPipelineStateString() const;
    
    // Performance counter getters
    uint64_t GetStallCycles() const { return stall_cycles_; }
    uint64_t GetBranchMispredictions() const { return branch_mispredictions_; }

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
    
    // Debug output helper
    void DebugPrintCycle() const;

    // Pipeline registers (current cycle)
    IF_ID_Register if_id_reg_{};
    ID_EX_Register id_ex_reg_{};
    EX_MEM_Register ex_mem_reg_{};
    MEM_WB_Register mem_wb_reg_{};

    // Pipeline registers for the next clock cycle
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

    // State for branch prediction/handling
    bool branch_taken_ = false;
    uint64_t branch_target_ = 0;

    // Branch prediction: saturating counter predictor keyed by PC.
    // Each entry is a 2-bit counter:
    //   0 = strongly not taken,
    //   1 = weakly not taken,
    //   2 = weakly taken,
    //   3 = strongly taken.
    std::unordered_map<uint64_t, uint8_t> branch_predictor_;
    
    PredictorMode predictor_mode_ = PredictorMode::kStatic;

    // Prediction information from the decode stage for the next fetch.
    // These flags tell the PC update logic at the end of ClockTick
    // whether the instruction in Decode predicted a branch or jump
    // and what the predicted outcome was.  They are cleared each cycle after the PC is updated.
    bool has_predicted_dec_ = false;
    bool predicted_taken_dec_ = false;
    uint64_t predicted_target_dec_ = 0;

    // Flag set in ExecuteStage when a misprediction occurs.  The
    // ClockTick will use this to increment branch_mispredictions_ and
    // then clear it.
    bool branch_mispredicted_ = false;

    // System trap handling
    static constexpr int kSystemTrapLatency_ = 3;
    int system_trap_cycles_ = 0;
    bool trap_in_progress_ = false;

    // Branch misprediction handling
    // Number of additional stall cycles to incur when a branch or jump
    // prediction is incorrect.  A typical 5‑stage pipeline incurs a penalty
    // of three cycles for a mispredicted branch (two bubbles plus the
    // branch itself), so we set this to 2 extra cycles beyond the one
    // inherent flush cycle.
    static constexpr int kBranchMispredPenalty_ = 2;
    // Counter of remaining penalty cycles for the current misprediction.
    int mispred_flush_cycles_ = 0;

    // Feature flags
    bool hazard_detection_enabled_ = true;
    bool debug_mode_ = false;
    
    // Performance counters are inherited from VmBase (stall_cycles_, branch_mispredictions_).
};

#endif // RV5S_VM_H
