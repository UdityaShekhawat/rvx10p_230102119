# RVX10-P: Five-Stage Pipelined RISC-V Core

## Overview

RVX10-P is a five-stage pipelined implementation of the RISC-V RV32I instruction set, extended with ten custom ALU operations under the RVX10 extension.
This project was developed as part of the course Digital Logic and Computer Architecture (Dr. Satyajit Das, IIT Guwahati).

The processor executes all base RV32I and RVX10 instructions correctly, handles data and control hazards through forwarding, stalling, and flushing, and includes built-in performance counters to measure CPI (Cycles Per Instruction).

## Pipeline Structure

The design follows the classic five-stage pipeline model:
|  Stage  | Name                               | Description                                        |
| :-----: | :--------------------------------- | :------------------------------------------------- |
|  **IF** | Instruction Fetch                  | Fetches instruction from instruction memory (IMEM) |
|  **ID** | Instruction Decode / Register Read | Decodes opcode and reads registers                 |
|  **EX** | Execute                            | Performs ALU operations and branch decisions       |
| **MEM** | Memory Access                      | Reads or writes data to memory (DMEM)              |
|  **WB** | Write Back                         | Writes results back to the register file           |

# Key Components
🔹 Datapath

Implements the five pipeline stages and valid-bit propagation.
Includes forwarding multiplexers, hazard-based stall and flush logic, and ALU operations for both standard RV32I and custom RVX10 instructions.

🔹 Controller

Generates all control signals for ALU, memory, and register writeback.
Implements maindec (opcode decoder) and aludec (function decoder).

🔹 Forwarding Unit

Detects data dependencies between EX/MEM/WB stages and redirects results to the ALU inputs as needed:

<img width="724" height="478" alt="image" src="https://github.com/user-attachments/assets/7448b04e-6698-43f1-bf27-fbf017754272" />


🔹 Hazard Detection Unit

Implements load-use and branch hazard detection:

<img width="790" height="363" alt="image" src="https://github.com/user-attachments/assets/c37ef675-8e36-4d20-b595-f467321b2c09" />

🔹 Performance Counters

Tracks total cycles and retired instructions to compute CPI:

<img width="808" height="505" alt="image" src="https://github.com/user-attachments/assets/313a6b37-2be9-44b2-a08c-49cf342373ff" />

## Supported Instruction Sets

### Base RV32I

add,sub, and, or, slt, addi, lw, sw, beq, jal

### RVX10 Custom ALU Instructions

<img width="952" height="376" alt="image" src="https://github.com/user-attachments/assets/acd590b8-e0ac-4e08-8ca3-7d4a39f5f4bc" />

## Pipelined processor with full hazard handling Block Diagram

<img width="1084" height="690" alt="image" src="https://github.com/user-attachments/assets/da449398-c181-483e-b0a5-491d1364909c" />

## How to run
```
# Compile (enable SystemVerilog-2012 support)
iverilog -g2012 -o cpu_tb riscvpipeline.sv.sv

# Run
vvp cpu_tb
```


**Expected console output**
```
Simulation succeeded
```


## Waveforms (Optional, with GTKWave)

The testbench is set up to dump `wave.vcd`. To open it:

```bash
# after running the simulation:
gtkwave wave.vcd
```


##  Acknowledgment
Dr. Satyajit Das
Assistant Professor
Department of Computer Science and Engineering
Indian Institute of Technology, Guwahati








