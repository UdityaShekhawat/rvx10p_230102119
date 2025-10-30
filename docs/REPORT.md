# RVX10-P: Five-Stage Pipelined RISC-V Core

**Design Report**  
*Digital Logic and Computer Architecture*  
 instructor :*Dr. Satyajit Das, IIT Guwahati*
 Author :Uditya Shekhawat

---

## 1. Introduction

### 1.1 Project Overview
RVX10-P is a five-stage pipelined implementation of a RISC-V processor core that supports the base RV32I instruction set extended with ten custom ALU instructions (RVX10 extension). This design transforms the previous single-cycle RVX10 implementation into a pipelined architecture to achieve higher instruction throughput and improved performance.

### 1.2 Design Goals
- **ISA Compatibility**: Full support for RV32I base instructions plus 10 custom RVX10 operations
- **Pipeline Efficiency**: Overlapped execution of multiple instructions to reduce CPI
- **Hazard Management**: Correct handling of data and control hazards through forwarding, stalling, and flushing
- **Architectural Correctness**: Identical functional results as the single-cycle implementation

### 1.3 RVX10 Custom Instructions
The processor implements the following custom ALU operations under opcode `custom-0` (0x0B):

| Instruction | funct7 | funct3 | Operation | Description |
|------------|--------|--------|-----------|-------------|
| ANDN | 0000000 | 000 | rd = rs1 & ~rs2 | AND-NOT |
| ORN | 0000000 | 001 | rd = rs1 \| ~rs2 | OR-NOT |
| XNOR | 0000000 | 010 | rd = ~(rs1 ^ rs2) | XOR-NOT |
| MIN | 0000001 | 000 | rd = min(rs1, rs2) | Signed minimum |
| MAX | 0000001 | 001 | rd = max(rs1, rs2) | Signed maximum |
| MINU | 0000001 | 010 | rd = minu(rs1, rs2) | Unsigned minimum |
| MAXU | 0000001 | 011 | rd = maxu(rs1, rs2) | Unsigned maximum |
| ROL | 0000010 | 000 | rd = rs1 ROL rs2[4:0] | Rotate left |
| ROR | 0000010 | 001 | rd = rs1 ROR rs2[4:0] | Rotate right |
| ABS | 0000011 | 000 | rd = abs(rs1) | Absolute value |

---

## 2. Pipeline Architecture

### 2.1 Five-Stage Pipeline Structure

The processor follows the classic RISC five-stage pipeline design:

```
┌────────┐    ┌────────┐    ┌────────┐    ┌────────┐    ┌────────┐
│   IF   │ -> │   ID   │ -> │   EX   │ -> │  MEM   │ -> │   WB   │
└────────┘    └────────┘    └────────┘    └────────┘    └────────┘
    |             |             |             |             |
 IF/ID reg    ID/EX reg   EX/MEM reg   MEM/WB reg      Result
```

**Stage Functions:**

1. **IF (Instruction Fetch)**
   - Fetch instruction from instruction memory
   - Calculate PC+4
   - Update program counter

2. **ID (Instruction Decode)**
   - Decode instruction fields (opcode, funct3, funct7, registers)
   - Read source registers from register file
   - Generate immediate values
   - Generate control signals

3. **EX (Execute)**
   - Perform ALU operations
   - Evaluate branch conditions
   - Calculate branch/jump target addresses
   - Select operands with forwarding

4. **MEM (Memory Access)**
   - Access data memory for loads/stores
   - Pass through ALU results

5. **WB (Write Back)**
   - Select result (ALU, memory, or PC+4)
   - Write back to register file

### 2.2 Pipeline Registers

Four pipeline registers separate the five stages, preserving intermediate values:

**IF/ID Register (96 bits)**
- InstrD [31:0] - Fetched instruction
- PCD [31:0] - Program counter value
- PCPlus4D [31:0] - PC + 4
- ValidD [1] - Valid bit for instruction

**ID/EX Register (175 bits)**
- RD1E, RD2E [31:0 each] - Register operands
- PCE [31:0] - PC value
- Rs1E, Rs2E, RdE [5 bits each] - Register addresses
- ImmExtE [31:0] - Sign-extended immediate
- PCPlus4E [31:0] - PC + 4
- Control signals (RegWriteE, ResultSrcE, MemWriteE, ALUSrcE, ALUControlE, etc.)
- ValidE [1] - Valid bit

**EX/MEM Register (101 bits)**
- ALUResultM [31:0] - ALU output
- WriteDataM [31:0] - Store data
- RdM [5] - Destination register
- PCPlus4M [31:0] - PC + 4
- Control signals (RegWriteM, ResultSrcM, MemWriteM)
- ValidM [1] - Valid bit

**MEM/WB Register (101 bits)**
- ALUResultW [31:0] - ALU result
- ReadDataW [31:0] - Memory read data
- RdW [5] - Destination register
- PCPlus4W [31:0] - PC + 4
- Control signals (RegWriteW, ResultSrcW)
- ValidW [1] - Valid bit

---

## 3. Pipeline Stage Implementation

### 3.1 Instruction Fetch (IF) Stage

```systemverilog
// Program Counter with enable for stalling
flopenr #(32) pcreg (
  .clk(clk),
  .reset(reset),
  .en (~StallF),        // Stall when load-use hazard
  .d  (PCNextF),
  .q  (PCF)
);

// PC source multiplexer (normal or branch target)
mux2 #(32) pcmux (
  .d0(PCPlus4F),
  .d1(PCTargetE),       // Branch target from EX stage
  .s (PCSrcE),          // Branch taken signal
  .y (PCNextF)
);

// PC incrementer
adder pcadd (
  .a(PCF),
  .b(32'd4),
  .s(PCPlus4F)
);
```

**Key Features:**
- PC can be stalled (held) during load-use hazards
- Branch target is calculated in EX stage and fed back
- Instruction memory access is combinational

### 3.2 Instruction Decode (ID) Stage

```systemverilog
// IF/ID pipeline register with enable and clear
flopenrc #(96) regD (
  .clk(clk),
  .reset(reset),
  .clear(FlushD),       // Flush on branch taken
  .en (~StallD),        // Stall on load-use hazard
  .d  ({InstrF, PCF, PCPlus4F}),
  .q  ({InstrD, PCD, PCPlus4D})
);

// Instruction field extraction
assign opD = InstrD[6:0];
assign funct3D = InstrD[14:12];
assign funct7D = InstrD[31:25];
assign funct7b5D = InstrD[30];
assign Rs1D = InstrD[19:15];
assign Rs2D = InstrD[24:20];
assign RdD = InstrD[11:7];

// Register file (read in ID, write in WB)
regfile rf (
  .clk(clk),
  .a1 (Rs1D),
  .a2 (Rs2D),
  .rd1(RD1D),           // Register read port 1
  .rd2(RD2D),           // Register read port 2
  .a3 (RdW),
  .we3(RegWriteW),
  .wd3(ResultW)         // Write from WB stage
);

// Immediate extension
extend ext (
  .instr (InstrD[31:7]),
  .immsrc(ImmSrcD),
  .immext(ImmExtD)
);
```

**Control Signal Generation:**

```systemverilog
// Main decoder
maindec md(
  .op(opD),
  .ResultSrc(ResultSrcD),
  .MemWrite(MemWriteD),
  .Branch(BranchD),
  .ALUSrc(ALUSrcD),
  .RegWrite(RegWriteD),
  .Jump(JumpD),
  .ImmSrc(ImmSrcD),
  .ALUOp(ALUOpD)
);

// ALU decoder
aludec ad(
  .opb5(opD[5]),
  .funct3(funct3D),
  .funct7(funct7D),
  .funct7b5(funct7b5D),
  .ALUOp(ALUOpD),
  .ALUControl(ALUControlD)
);
```

**Immediate Extension Logic:**
- I-type: `{{20{instr[31]}}, instr[31:20]}`
- S-type: `{{20{instr[31]}}, instr[31:25], instr[11:7]}`
- B-type: `{{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}`
- J-type: `{{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}`

### 3.3 Execute (EX) Stage

```systemverilog
// ID/EX pipeline register with clear
floprc #(175) regE (
  .clk(clk),
  .reset(reset),
  .clear(FlushE),       // Flush on branch or load-use
  .d  ({RD1D, RD2D, PCD, Rs1D, Rs2D, RdD, ImmExtE, PCPlus4D}),
  .q  ({RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E})
);

// Forwarding multiplexers
// Forward A: selects rs1 source
mux3 #(32) faemux (
  .d0(RD1E),            // From ID/EX register
  .d1(ResultW),         // Forward from WB stage
  .d2(ALUResultM),      // Forward from MEM stage
  .s (ForwardAE),
  .y (SrcAEforward)
);

// Forward B: selects rs2 source
mux3 #(32) fbemux (
  .d0(RD2E),
  .d1(ResultW),
  .d2(ALUResultM),
  .s (ForwardBE),
  .y (WriteDataE)
);

// ALU source B multiplexer (register or immediate)
mux2 #(32) srcbmux (
  .d0(WriteDataE),      // Register value (forwarded)
  .d1(ImmExtE),         // Immediate value
  .s (ALUSrcE),
  .y (SrcBE)
);

// ALU execution
alu alu (
  .a         (SrcAE),
  .b         (SrcBE),
  .alucontrol(ALUControlE),
  .result    (ALUResultE),
  .zero      (ZeroE)
);

// Branch target calculation
adder branchadd (
  .a(ImmExtE),
  .b(PCE),
  .s(PCTargetE)
);

// Branch decision
assign PCSrcE = (BranchE & ZeroE) | JumpE;
```

**ALU Operations:**

The ALU supports all RV32I and RVX10 operations:

```systemverilog
always_comb begin
  case (alucontrol)
    4'b0000: result = sum;                    // ADD
    4'b0001: result = sum;                    // SUB
    4'b0010: result = a & b;                  // AND
    4'b0011: result = a | b;                  // OR
    4'b0101: result = {31'b0, sum[31] ^ v};   // SLT
    4'b0110: result = (shamt == 0) ? a :      // ROL
                      (a << shamt) | (a >> (32 - shamt));
    4'b0111: result = (shamt == 0) ? a :      // ROR
                      (a >> shamt) | (a << (32 - shamt));
    4'b1000: result = a & ~b;                 // ANDN
    4'b1001: result = a | ~b;                 // ORN
    4'b1010: result = ~(a ^ b);               // XNOR
    4'b1011: result = (as < bs) ? a : b;      // MIN
    4'b1100: result = (as > bs) ? a : b;      // MAX
    4'b1101: result = (a < b) ? a : b;        // MINU
    4'b1110: result = (a > b) ? a : b;        // MAXU
    4'b1111: result = (as[31]) ? (~a + 1'b1) : a; // ABS
    default: result = 32'b0;
  endcase
end
```

### 3.4 Memory (MEM) Stage

```systemverilog
// EX/MEM pipeline register
flopr #(101) regM (
  .clk(clk),
  .reset(reset),
  .d  ({ALUResultE, WriteDataE, RdE, PCPlus4E}),
  .q  ({ALUResultM, WriteDataM, RdM, PCPlus4M})
);

// Data memory (external to datapath)
dmem dmem (
  .clk(clk),
  .we(MemWriteM),
  .a (DataAdrM),        // ALUResultM
  .wd(WriteDataM),
  .rd(ReadDataM)
);
```

**Memory Operations:**
- Load instructions: Read data from memory (word-aligned)
- Store instructions: Write data to memory (word-aligned)
- Other instructions: ALU result passes through unchanged

### 3.5 Write Back (WB) Stage

```systemverilog
// MEM/WB pipeline register
flopr #(101) regW (
  .clk(clk),
  .reset(reset),
  .d  ({ALUResultM, ReadDataM, RdM, PCPlus4M}),
  .q  ({ALUResultW, ReadDataW, RdW, PCPlus4W})
);

// Result multiplexer
mux3 #(32) resultmux (
  .d0(ALUResultW),      // ALU result
  .d1(ReadDataW),       // Memory data (load)
  .d2(PCPlus4W),        // PC+4 (jal)
  .s (ResultSrcW),
  .y (ResultW)
);

// Write back to register file
regfile rf (
  // ... read ports in ID stage
  .a3 (RdW),
  .we3(RegWriteW),
  .wd3(ResultW)
);
```

**Result Selection:**
- `ResultSrcW = 00`: ALU result (R-type, I-type ALU)
- `ResultSrcW = 01`: Memory data (load instructions)
- `ResultSrcW = 10`: PC+4 (jal instruction)

---

## 4. Hazard Handling

### 4.1 Data Hazards and Forwarding

**Problem:** An instruction needs a result that is still being computed by a previous instruction.

**Example:**
```assembly
add x1, x2, x3    # x1 written in WB stage
sub x4, x1, x5    # x1 needed in EX stage
```

**Solution: Forwarding Unit**

```systemverilog
module forwarding_unit (
  input  logic [4:0] Rs1E,      // Source register 1 in EX
  input  logic [4:0] Rs2E,      // Source register 2 in EX
  input  logic [4:0] RdM,       // Destination register in MEM
  input  logic [4:0] RdW,       // Destination register in WB
  input  logic       RegWriteM, // Write enable in MEM
  input  logic       RegWriteW, // Write enable in WB
  output logic [1:0] ForwardAE, // Forward select for rs1
  output logic [1:0] ForwardBE  // Forward select for rs2
);

  // Forwarding logic for rs1
  always_comb begin
    if ((Rs1E == RdM) && RegWriteM && (Rs1E != 5'b0))
      ForwardAE = 2'b10;    // Forward from MEM stage
    else if ((Rs1E == RdW) && RegWriteW && (Rs1E != 5'b0))
      ForwardAE = 2'b01;    // Forward from WB stage
    else
      ForwardAE = 2'b00;    // No forwarding
  end

  // Forwarding logic for rs2 (identical structure)
  always_comb begin
    if ((Rs2E == RdM) && RegWriteM && (Rs2E != 5'b0))
      ForwardBE = 2'b10;
    else if ((Rs2E == RdW) && RegWriteW && (Rs2E != 5'b0))
      ForwardBE = 2'b01;
    else
      ForwardBE = 2'b00;
  end

endmodule
```

**Forwarding Priority:**
1. **MEM stage** (most recent): Forward if EX needs data being written in MEM
2. **WB stage**: Forward if EX needs data being written in WB
3. **No forwarding**: Use value from ID/EX register

**Special Case:** x0 register is hardwired to zero, so never forward to/from x0.

### 4.2 Load-Use Hazard

**Problem:** A load instruction is followed immediately by an instruction that uses the loaded data. The data isn't available until MEM stage, but forwarding from MEM happens too late for the EX stage.

**Example:**
```assembly
lw  x1, 0(x2)     # x1 available in MEM stage
add x3, x1, x4    # x1 needed in EX stage (too early!)
```

**Solution: Stall one cycle**

The hazard unit detects this condition and:
- Stalls IF and ID stages (hold current values)
- Flushes ID/EX register (insert bubble/NOP)
- After one cycle, forwarding from MEM to EX works normally

### 4.3 Control Hazards

**Problem:** Branch/jump decisions are made in EX stage, but the next instruction has already been fetched.

**Example:**
```assembly
beq x1, x2, target    # Decision in EX stage
add x3, x4, x5        # Already fetched (wrong if branch taken)
```

**Solution: Flush on branch taken**

When a branch is taken (`PCSrcE = 1`):
- Flush IF/ID register (convert fetched instruction to NOP)
- Flush ID/EX register (convert decoded instruction to NOP)
- Update PC to branch target

### 4.4 Hazard Detection Unit

```systemverilog
module hazard_unit (
  input  logic [4:0] Rs1D,      // Source registers in ID
  input  logic [4:0] Rs2D,
  input  logic [4:0] RdE,       // Destination in EX
  input  logic       PCSrcE,    // Branch taken
  input  logic       ResultSrcEb0, // Load instruction in EX
  output logic       StallF,    // Stall fetch
  output logic       StallD,    // Stall decode
  output logic       FlushD,    // Flush IF/ID
  output logic       FlushE     // Flush ID/EX
);

  logic lwStallD;

  // Load-use hazard detection
  assign lwStallD = ResultSrcEb0 && 
                    ((Rs1D == RdE) || (Rs2D == RdE));

  // Stall signals
  assign StallF = lwStallD;
  assign StallD = lwStallD;

  // Flush signals
  assign FlushD = PCSrcE;           // Flush on branch
  assign FlushE = lwStallD || PCSrcE; // Flush on stall or branch

endmodule
```

**Hazard Detection Logic:**

1. **Load-Use Stall:**
   - Condition: `ResultSrcEb0` (load in EX) AND (`Rs1D == RdE` OR `Rs2D == RdE`)
   - Action: `StallF = StallD = 1`, `FlushE = 1`

2. **Control Hazard:**
   - Condition: `PCSrcE = 1` (branch taken)
   - Action: `FlushD = FlushE = 1`

### 4.5 Valid Bit Pipeline

To accurately track instructions through the pipeline and measure performance:

```systemverilog
// Valid bit pipeline registers
flopenrc #(1) validregD (
  .clk(clk),
  .reset(reset),
  .clear(FlushD),       // Clear on flush
  .en(~StallD),         // Hold on stall
  .d(ValidF),           // ValidF = 1 (fetch always valid)
  .q(ValidD)
);

floprc #(1) validregE (
  .clk(clk),
  .reset(reset),
  .clear(FlushE),
  .d(ValidD),
  .q(ValidE)
);

flopr #(1) validregM (
  .clk(clk),
  .reset(reset),
  .d(ValidE),
  .q(ValidM)
);

flopr #(1) validregW (
  .clk(clk),
  .reset(reset),
  .d(ValidM),
  .q(ValidW)
);
```

**Purpose:**
- Track valid instructions through pipeline
- Distinguish between real instructions and bubbles (NOPs from flushes)
- Accurate instruction retirement counting for CPI calculation

---

## 5. Performance Monitoring

### 5.1 Performance Counters

```systemverilog
// Performance counters
always_ff @(posedge clk or posedge reset) begin
  if (reset) begin
    cycle_count <= 32'b0;
    instr_retired <= 32'b0;
  end else begin
    // Increment cycle counter every clock cycle
    cycle_count <= cycle_count + 1;
    
    // Increment retired instruction counter when valid
    // instruction reaches WB stage
    if (ValidW) begin
      instr_retired <= instr_retired + 1;
    end
  end
end
```
<img width="974" height="465" alt="image" src="https://github.com/user-attachments/assets/cb0c1adc-5f98-4bc1-99fc-4e421f77fc3a" />

<img width="398" height="190" alt="image" src="https://github.com/user-attachments/assets/a22742ae-b7bf-4525-80d6-d01da913a760" />




## 6. Control Signal Summary

| Signal | Stage | Description |
|--------|-------|-------------|
| **RegWriteD/E/M/W** | All | Enable register write |
| **ResultSrcD/E/M/W** | All | Select write-back source (00=ALU, 01=Mem, 10=PC+4) |
| **MemWriteD/E/M** | ID-MEM | Enable data memory write |
| **ALUSrcD/E** | ID-EX | Select ALU operand B (0=reg, 1=imm) |
| **ALUControlD/E** | ID-EX | ALU operation select (4 bits) |
| **ImmSrcD** | ID | Immediate format select |
| **BranchD/E** | ID-EX | Branch instruction indicator |
| **JumpD/E** | ID-EX | Jump instruction indicator |
| **PCSrcE** | EX | PC source select (0=PC+4, 1=target) |
| **ForwardAE/BE** | EX | Forwarding select for ALU inputs |
| **StallF/D** | IF-ID | Stall pipeline stages |
| **FlushD/E** | ID-EX | Flush pipeline stages |

---
## Verification and waveform

Single test program finishes with store of 25 to memory address 100.
<img width="1589" height="733" alt="25 at 100 (1)" src="https://github.com/user-attachments/assets/42d57a82-2f3e-4c10-8ff7-fb102e3eee40" />

x0 register remains constant at zero
<img width="1894" height="495" alt="R0 is 0" src="https://github.com/user-attachments/assets/f416c073-6260-496d-86bb-d87de2afd066" />

 Forwarding verified for back-to-back ALU ops.
 <img width="1904" height="810" alt="Screenshot 2025-10-26 212422" src="https://github.com/user-attachments/assets/b443d5b3-f486-48e4-b2b8-22caa18851b8" />

<img width="1893" height="278" alt="Screenshot 2025-10-26 212517" src="https://github.com/user-attachments/assets/1e49ecb8-b1da-492c-a298-e3eecad8cb0c" />

<img width="943" height="351" alt="Screenshot 2025-10-26 212807" src="https://github.com/user-attachments/assets/4dcb0579-633d-4c0e-b48f-f0c01bf333e3" />

<img width="1877" height="290" alt="Screenshot 2025-10-26 212817" src="https://github.com/user-attachments/assets/47567af7-fb8d-46a6-83c2-6dcb08d499b7" />

• One-cycle stall correctly inserted for load-use.
<img width="983" height="219" alt="Screenshot 2025-10-26 215217" src="https://github.com/user-attachments/assets/cda02967-98a2-40e0-9a63-444ebb45c5ff" />

<img width="1904" height="477" alt="Screenshot 2025-10-26 215149" src="https://github.com/user-attachments/assets/c628dc9c-fdf4-4248-be53-0cd23b1e0023" />

Pipeline flush works for taken branches
<img width="937" height="148" alt="Screenshot 2025-10-26 220603" src="https://github.com/user-attachments/assets/0cc719d9-2922-444d-985c-5b9e125c37ca" />

<img width="1820" height="471" alt="Screenshot 2025-10-26 220615" src="https://github.com/user-attachments/assets/5e8b2fcc-8edf-4cd1-b3ac-bd120bcc9a39" />

<img width="1891" height="428" alt="Screenshot 2025-10-26 220625" src="https://github.com/user-attachments/assets/89d3df8d-2802-4708-a8c3-6fbabbb06f70" />

<img width="1907" height="828" alt="Screenshot 2025-10-26 220652" src="https://github.com/user-attachments/assets/92f5c9dc-e55e-4cf2-b2c5-21efeb14bda2" />

<img width="1883" height="862" alt="Screenshot 2025-10-26 220659" src="https://github.com/user-attachments/assets/2c2e47e1-33be-4032-8ca6-509127c1ea55" />

pipeline concurrency
<img width="1569" height="735" alt="Screenshot 2025-10-26 224253" src="https://github.com/user-attachments/assets/f7b6a523-ba79-4734-8bd0-9c7f7294e0a7" />

<img width="1626" height="781" alt="Screenshot 2025-10-26 224301" src="https://github.com/user-attachments/assets/1e059e93-08f5-4b72-82b7-fa61b6c2b764" />

<img width="1591" height="821" alt="Screenshot 2025-10-26 224309" src="https://github.com/user-attachments/assets/b8cfd29d-d0cf-4a7d-82af-39fa0edeb0da" />

<img width="1629" height="819" alt="Screenshot 2025-10-26 224317" src="https://github.com/user-attachments/assets/fbdd7cec-e443-4651-9538-5936001c5cba" />

<img width="1620" height="779" alt="Screenshot 2025-10-26 224323" src="https://github.com/user-attachments/assets/ea74dc3e-c6aa-4115-ab6f-3dcf89d20bc9" />

 ## Simulation Output
 <img width="1522" height="208" alt="25 at 100 (2)" src="https://github.com/user-attachments/assets/e427b7bc-c736-4104-a6b9-af94df0df359" />
<img width="398" height="190" alt="Screenshot 2025-10-30 220528" src="https://github.com/user-attachments/assets/26c73e70-82ad-44d6-a0ca-049d59d3f765" />




















