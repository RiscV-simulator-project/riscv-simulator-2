#include "main.h"
#include "assembler/assembler.h"
#include "utils.h"
#include "globals.h"
#include "vm/rvss/rvss_vm.h"
#include "vm/rv5s/rv5s_vm.h"
#include "vm_runner.h"
#include "command_handler.h"
#include "config.h"

#include <iostream>
#include <thread>
#include <bitset>
#include <regex>

int main(int argc, char *argv[]) {
  if (argc <= 1) {
    std::cerr << "No arguments provided. Use --help for usage information.\n";
    return 1;
  }

  std::string pipeline = "rvss"; // default single-stage VM
  bool hazards_enabled = true;    // default: enable hazards for rv5s pipeline
  bool debug_mode = false;        // default: disable debug mode
  PredictorMode predictor_mode = PredictorMode::kStatic; // default: static
  bool run_requested = false;
  std::string run_file;
  
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
        std::cout << "Usage: " << argv[0] << " [options]\n"
                  << "Options:\n"
                  << "  --help, -h             Show this help message\n"
                  << "  --assemble <file>      Assemble the specified file\n"
                  << "  --run <file>           Run the specified file\n"
                  << "  --verbose-errors       Enable verbose error printing\n"
                  << "  --start-vm             Start the VM with the default program\n"
                  << "  --start-vm --vm-as-backend  Start the VM with the default program in backend mode\n"
                  << "  --pipeline <rvss|rv5s> Select VM pipeline (default rvss)\n"
                  << "  --hazards=<on|off>     Enable/disable hazard detection (rv5s only, default on)\n"
                  << "  --debug=<on|off>       Enable/disable debug mode (rv5s only, default off)\n"
                  << "  --predictor=<static|dynamic> Select branch predictor mode (rv5s only, default static)\n";
        return 0;
    } 
    else if (arg == "--assemble") {
        if (++i >= argc) {
            std::cerr << "Error: No file specified for assembly.\n";
            return 1;
        }
        try {
            AssembledProgram program = assemble(argv[i]);
            std::cout << "Assembled program: " << program.filename << '\n';
            return 0;
        } catch (const std::runtime_error& e) {
            std::cerr << e.what() << '\n';
            return 1;
        }
    } 
    else if (arg == "--run") {
        if (run_requested) {
            std::cerr << "Error: --run specified multiple times.\n";
            return 1;
        }
        if (++i >= argc) {
            std::cerr << "Error: No file specified to run.\n";
            return 1;
        }
        run_file = argv[i];
        run_requested = true;

    } else if (arg == "--verbose-errors") {
        globals::verbose_errors_print = true;
        std::cout << "Verbose error printing enabled.\n";

    } else if (arg == "--vm-as-backend") {
        globals::vm_as_backend = true;
        std::cout << "VM backend mode enabled.\n";
        
    } else if (arg == "--start-vm") {
        break;
        
    } else if (arg.rfind("--hazards=", 0) == 0) {
        std::string v = arg.substr(std::string("--hazards=").size());
        if (v == "on" || v == "all") hazards_enabled = true;
        else if (v == "off") hazards_enabled = false;
        else {
            std::cerr << "Unknown value for --hazards: " << v << " (use on|off|all)\n";
            return 1;
        }
        
    } else if (arg.rfind("--debug=", 0) == 0) {
        std::string v = arg.substr(std::string("--debug=").size());
        if (v == "on" || v == "all") debug_mode = true;
        else if (v == "off") debug_mode = false;
        else {
            std::cerr << "Unknown value for --debug: " << v << " (use on|off|all)\n";
            return 1;
        }
        
    } else if (arg.rfind("--predictor=", 0) == 0) {
        std::string v = arg.substr(std::string("--predictor=").size());
        if (v == "static") predictor_mode = PredictorMode::kStatic;
        else if (v == "dynamic") predictor_mode = PredictorMode::kDynamic;
        else {
            std::cerr << "Unknown value for --predictor: " << v << " (use static|dynamic)\n";
            return 1;
        }
    
    } else if (arg.rfind("--pipeline=", 0) == 0) {
        pipeline = arg.substr(std::string("--pipeline=").size());
        if (pipeline != "rvss" && pipeline != "rv5s") {
            std::cerr << "Unknown pipeline: " << pipeline << " (use rvss or rv5s)\n";
            return 1;
        }
        
    } else if (arg == "--pipeline") {
        if (++i >= argc) {
            std::cerr << "Error: No pipeline specified (rvss|rv5s).\n";
            return 1;
        }
        pipeline = argv[i];
        if (pipeline != "rvss" && pipeline != "rv5s") {
            std::cerr << "Unknown pipeline: " << pipeline << " (use rvss or rv5s)\n";
            return 1;
        }

    } else {
        std::cerr << "Unknown option: " << arg << '\n';
        return 1;
    }
  }

  if (run_requested) {
      try {
          AssembledProgram program = assemble(run_file.c_str());
          std::unique_ptr<VmBase> vm;
          if (pipeline == "rv5s") {
            auto p = std::make_unique<RiscV5StageVM>();
            p->SetHazardDetection(hazards_enabled);
            p->SetDebugMode(debug_mode);
            p->SetPredictorMode(predictor_mode);
            vm = std::move(p);
          } else {
            vm = std::make_unique<RVSSVM>();
          }
          vm->LoadProgram(program);
          vm->Run();
          std::cout << "Program running: " << program.filename << '\n';
          return 0;
      } catch (const std::runtime_error& e) {
          std::cerr << e.what() << '\n';
          return 1;
      }
  }
  
  setupVmStateDirectory();

  AssembledProgram program;
  std::unique_ptr<VmBase> vm;
  
  if (pipeline == "rv5s") {
    auto p = std::make_unique<RiscV5StageVM>();
    p->SetHazardDetection(hazards_enabled);
    p->SetDebugMode(debug_mode);
    p->SetPredictorMode(predictor_mode);
    vm = std::move(p);
  } else {
    vm = std::make_unique<RVSSVM>();
  }

  std::cout << "VM_STARTED" << std::endl;

  std::thread vm_thread;
  bool vm_running = false;

  auto launch_vm_thread = [&](auto fn) {
    if (vm_thread.joinable()) {
      vm->stop_requested_ = true;   
      vm_thread.join();
    }
    vm_running = true;
    vm_thread = std::thread([&]() {
      fn();               
      vm_running = false;
    });
  };

  std::string command_buffer;
  while (true) {
    std::getline(std::cin, command_buffer);
    command_handler::Command command = command_handler::ParseCommand(command_buffer);

    if (command.type==command_handler::CommandType::MODIFY_CONFIG) {
      if (command.args.size() != 3) {
        std::cout << "VM_MODIFY_CONFIG_ERROR" << std::endl;
        continue;
      }
      try {
        vm_config::config.modifyConfig(command.args[0], command.args[1], command.args[2]);
        std::cout << "VM_MODIFY_CONFIG_SUCCESS" << std::endl;
      } catch (const std::exception &e) {
        std::cout << "VM_MODIFY_CONFIG_ERROR" << std::endl;
        std::cerr << e.what() << '\n';
        continue;
      }
      continue;
    }

    if (command.type==command_handler::CommandType::LOAD) {
      try {
        program = assemble(command.args[0]);
        std::cout << "VM_PARSE_SUCCESS" << std::endl;
        vm->output_status_ = "VM_PARSE_SUCCESS";
        vm->DumpState(globals::vm_state_dump_file_path);
      } catch (const std::runtime_error &e) {
        std::cout << "VM_PARSE_ERROR" << std::endl;
        vm->output_status_ = "VM_PARSE_ERROR";
        vm->DumpState(globals::vm_state_dump_file_path);
        std::cerr << e.what() << '\n';
        continue;
      }
      vm->LoadProgram(program);
      std::cout << "Program loaded: " << command.args[0] << std::endl;
      
    } else if (command.type==command_handler::CommandType::RUN) {
      launch_vm_thread([&]() { vm->Run(); });
      
    } else if (command.type==command_handler::CommandType::DEBUG_RUN) {
      launch_vm_thread([&]() { vm->DebugRun(); });
      
    } else if (command.type==command_handler::CommandType::STOP) {
      vm->stop_requested_ = true;
      std::cout << "VM_STOPPED" << std::endl;
      vm->output_status_ = "VM_STOPPED";
      vm->DumpState(globals::vm_state_dump_file_path);
      
    } else if (command.type==command_handler::CommandType::STEP) {
      if (vm_running) continue;
      launch_vm_thread([&]() { vm->Step(); });

    } else if (command.type==command_handler::CommandType::UNDO) {
      if (vm_running) continue;
      vm->Undo();
      
    } else if (command.type==command_handler::CommandType::REDO) {
      if (vm_running) continue;
      vm->Redo();
      
    } else if (command.type==command_handler::CommandType::RESET) {
      vm->Reset();
      
    } else if (command.type==command_handler::CommandType::EXIT) {
      vm->stop_requested_ = true;
      if (vm_thread.joinable()) vm_thread.join();
      vm->output_status_ = "VM_EXITED";
      vm->DumpState(globals::vm_state_dump_file_path);
      break;
      
    } else if (command.type==command_handler::CommandType::ADD_BREAKPOINT) {
      vm->AddBreakpoint(std::stoul(command.args[0], nullptr, 10));
      
    } else if (command.type==command_handler::CommandType::REMOVE_BREAKPOINT) {
      vm->RemoveBreakpoint(std::stoul(command.args[0], nullptr, 10));
      
    } else if (command.type==command_handler::CommandType::MODIFY_REGISTER) {
      try {
        if (command.args.size() != 2) {
          std::cout << "VM_MODIFY_REGISTER_ERROR" << std::endl;
          continue;
        }
        std::string reg_name = command.args[0];
        uint64_t value = std::stoull(command.args[1], nullptr, 16);
        vm->ModifyRegister(reg_name, value);
        DumpRegisters(globals::registers_dump_file_path, vm->registers_);
        std::cout << "VM_MODIFY_REGISTER_SUCCESS" << std::endl;
      } catch (const std::out_of_range &e) {
        std::cout << "VM_MODIFY_REGISTER_ERROR" << std::endl;
        continue;
      } catch (const std::exception& e) {
        std::cout << "VM_MODIFY_REGISTER_ERROR" << std::endl;
        continue;
      }
      
    } else if (command.type==command_handler::CommandType::GET_REGISTER) {
      std::string reg_str = command.args[0];
      if (reg_str[0] == 'x') {
        std::cout << "VM_REGISTER_VAL_START";
        std::cout << "0x"
                  << std::hex
                  << vm->registers_.ReadGpr(std::stoi(reg_str.substr(1))) 
                  << std::dec;
        std::cout << "VM_REGISTER_VAL_END"<< std::endl;
      } 
    }
    
    else if (command.type==command_handler::CommandType::MODIFY_MEMORY) {
      if (command.args.size() != 3) {
        std::cout << "VM_MODIFY_MEMORY_ERROR" << std::endl;
        continue;
      }
      try {
        uint64_t address = std::stoull(command.args[0], nullptr, 16);
        std::string type = command.args[1];
        uint64_t value = std::stoull(command.args[2], nullptr, 16);

        if (type == "byte") {
          vm->memory_controller_.WriteByte(address, static_cast<uint8_t>(value));
        } else if (type == "half") {
          vm->memory_controller_.WriteHalfWord(address, static_cast<uint16_t>(value));
        } else if (type == "word") {
          vm->memory_controller_.WriteWord(address, static_cast<uint32_t>(value));
        } else if (type == "double") {
          vm->memory_controller_.WriteDoubleWord(address, value);
        } else {
          std::cout << "VM_MODIFY_MEMORY_ERROR" << std::endl;
          continue;
        }
        std::cout << "VM_MODIFY_MEMORY_SUCCESS" << std::endl;
      } catch (const std::out_of_range &e) {
        std::cout << "VM_MODIFY_MEMORY_ERROR" << std::endl;
        continue;
      } catch (const std::exception& e) {
        std::cout << "VM_MODIFY_MEMORY_ERROR" << std::endl;
        continue;
      }
    }
    
    else if (command.type==command_handler::CommandType::DUMP_MEMORY) {
      try {
        vm->memory_controller_.DumpMemory(command.args);
      } catch (const std::out_of_range &e) {
        std::cout << "VM_MEMORY_DUMP_ERROR" << std::endl;
        continue;
      } catch (const std::exception& e) {
        std::cout << "VM_MEMORY_DUMP_ERROR" << std::endl;
        continue;
      }
      
    } else if (command.type==command_handler::CommandType::PRINT_MEMORY) {
      for (size_t i = 0; i < command.args.size(); i+=2) {
        uint64_t address = std::stoull(command.args[i], nullptr, 16);
        uint64_t rows = std::stoull(command.args[i+1]);
        vm->memory_controller_.PrintMemory(address, rows);
      }
      std::cout << std::endl;
      
    } else if (command.type==command_handler::CommandType::GET_MEMORY_POINT) {
      if (command.args.size() != 1) {
        std::cout << "VM_GET_MEMORY_POINT_ERROR" << std::endl;
        continue;
      }
      vm->memory_controller_.GetMemoryPoint(command.args[0]);
    } 

    else if (command.type==command_handler::CommandType::VM_STDIN) {
      vm->PushInput(command.args[0]);
    }
    
    else if (command.type==command_handler::CommandType::DUMP_CACHE) {
      std::cout << "Cache dumped." << std::endl;
      
    } else {
      std::cout << "Invalid command.";
      std::cout << command_buffer << std::endl;
    }
  }

  return 0;
}
