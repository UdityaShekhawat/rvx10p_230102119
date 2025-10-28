`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////


// Forwarding Unit - Handles data forwarding from MEM and WB stages to EX

module forwarding_unit (
  input  logic [4:0] Rs1E,
  input  logic [4:0] Rs2E,
  input  logic [4:0] RdM,
  input  logic [4:0] RdW,
  input  logic       RegWriteM,
  input  logic       RegWriteW,
  output logic [1:0] ForwardAE,
  output logic [1:0] ForwardBE
);

  // Forwarding logic for SrcAE (first source operand)
  always_comb begin
    if ((Rs1E == RdM) && RegWriteM && (Rs1E != 5'b0))
      ForwardAE = 2'b10; // Forward from Memory stage (ALUResultM)
    else if ((Rs1E == RdW) && RegWriteW && (Rs1E != 5'b0))
      ForwardAE = 2'b01; // Forward from Writeback stage (ResultW)
    else
      ForwardAE = 2'b00; // No forwarding - use value from ID/EX register
  end

  // Forwarding logic for SrcBE (second source operand)
  always_comb begin
    if ((Rs2E == RdM) && RegWriteM && (Rs2E != 5'b0))
      ForwardBE = 2'b10; // Forward from Memory stage (ALUResultM)
    else if ((Rs2E == RdW) && RegWriteW && (Rs2E != 5'b0))
      ForwardBE = 2'b01; // Forward from Writeback stage (ResultW)
    else
      ForwardBE = 2'b00; // No forwarding - use value from ID/EX register
  end

endmodule


// Hazard Detection Unit - Detects load-use hazards and control hazards

module hazard_unit (
  input  logic [4:0] Rs1D,
  input  logic [4:0] Rs2D,
  input  logic [4:0] RdE,
  input  logic       PCSrcE,
  input  logic       ResultSrcEb0,
  output logic       StallF,
  output logic       StallD,
  output logic       FlushD,
  output logic       FlushE
);

  logic lwStallD;

  // Load-use hazard detection
  assign lwStallD = ResultSrcEb0 && ((Rs1D == RdE) || (Rs2D == RdE));

  // Stall signals - hold IF and ID stages when load-use hazard detected
  assign StallF = lwStallD;
  assign StallD = lwStallD;

  // Flush ID/EX when branch/jump taken (control hazard)
  assign FlushD = PCSrcE;
  
  // Flush EX stage for both load-use stall and control hazard
  assign FlushE = lwStallD || PCSrcE;

endmodule


// Modified Top-Level RISCV Module with Valid Bit Pipeline

module riscv (
  input  logic        clk,
  input  logic        reset,
  output logic [31:0] PCF,
  input  logic [31:0] InstrF,
  output logic        MemWriteM,
  output logic [31:0] ALUResultM,
  output logic [31:0] WriteDataM,
  input  logic [31:0] ReadDataM,
  // Performance monitoring
  output logic [31:0] cycle_count,
  output logic [31:0] instr_retired
);

  logic [6:0] opD;
  logic [2:0] funct3D;
  logic [6:0] funct7D;
  logic       funct7b5D;
  logic [1:0] ImmSrcD;
  logic       ZeroE;
  logic       PCSrcE;
  logic [3:0] ALUControlE;
  logic       ALUSrcE;
  logic       ResultSrcEb0;
  logic       RegWriteM;
  logic [1:0] ResultSrcW;
  logic       RegWriteW;
  logic [1:0] ForwardAE, ForwardBE;
  logic       StallF, StallD, FlushD, FlushE;
  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
  
  // Valid bit pipeline
  logic       ValidD, ValidE, ValidM, ValidW;

  controller c (
    .clk(clk),
    .reset(reset),
    .opD(opD),
    .funct3D(funct3D),
    .funct7D(funct7D),
    .funct7b5D(funct7b5D),
    .ImmSrcD(ImmSrcD),
    .FlushE(FlushE),
    .ZeroE(ZeroE),
    .PCSrcE(PCSrcE),
    .ALUControlE(ALUControlE),
    .ALUSrcE(ALUSrcE),
    .ResultSrcEb0(ResultSrcEb0),
    .MemWriteM(MemWriteM),
    .RegWriteM(RegWriteM),
    .RegWriteW(RegWriteW),
    .ResultSrcW(ResultSrcW)
  );

  datapath dp (
    .clk(clk),
    .reset(reset),
    .StallF(StallF),
    .PCF(PCF),
    .InstrF(InstrF),
    .opD(opD),
    .funct3D(funct3D),
    .funct7D(funct7D),
    .funct7b5D(funct7b5D),
    .StallD(StallD),
    .FlushD(FlushD),
    .ImmSrcD(ImmSrcD),
    .FlushE(FlushE),
    .ForwardAE(ForwardAE),
    .ForwardBE(ForwardBE),
    .PCSrcE(PCSrcE),
    .ALUControlE(ALUControlE),
    .ALUSrcE(ALUSrcE),
    .ZeroE(ZeroE),
    .MemWriteM(MemWriteM),
    .WriteDataM(WriteDataM),
    .ALUResultM(ALUResultM),
    .ReadDataM(ReadDataM),
    .RegWriteW(RegWriteW),
    .ResultSrcW(ResultSrcW),
    .Rs1D(Rs1D),
    .Rs2D(Rs2D),
    .Rs1E(Rs1E),
    .Rs2E(Rs2E),
    .RdE(RdE),
    .RdM(RdM),
    .RdW(RdW),
    .ValidD(ValidD),
    .ValidE(ValidE),
    .ValidM(ValidM),
    .ValidW(ValidW)
  );

  forwarding_unit fu (
    .Rs1E(Rs1E),
    .Rs2E(Rs2E),
    .RdM(RdM),
    .RdW(RdW),
    .RegWriteM(RegWriteM),
    .RegWriteW(RegWriteW),
    .ForwardAE(ForwardAE),
    .ForwardBE(ForwardBE)
  );

  hazard_unit hu (
    .Rs1D(Rs1D),
    .Rs2D(Rs2D),
    .RdE(RdE),
    .PCSrcE(PCSrcE),
    .ResultSrcEb0(ResultSrcEb0),
    .StallF(StallF),
    .StallD(StallD),
    .FlushD(FlushD),
    .FlushE(FlushE)
  );

  
  // Performance Counters - Fixed CPI calculation
 
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      cycle_count <= 32'b0;
      instr_retired <= 32'b0;
    end else begin
      // Increment cycle counter every clock cycle
      cycle_count <= cycle_count + 1;
      
      // Increment retired instruction counter when a valid instruction
      // reaches WB stage (regardless of whether it writes to a register)
      // This correctly counts all instructions including stores and branches
      if (ValidW) begin
        instr_retired <= instr_retired + 1;
      end
    end
  end

endmodule


// Top Module

module top (
  input  logic        clk,
  input  logic        reset,
  output logic [31:0] WriteDataM,
  output logic [31:0] DataAdrM,
  output logic        MemWriteM,
  output logic [31:0] cycle_count,
  output logic [31:0] instr_retired
);

  logic [31:0] PCF, InstrF, ReadDataM;

  riscv riscv (
    .clk(clk),
    .reset(reset),
    .PCF(PCF),
    .InstrF(InstrF),
    .MemWriteM(MemWriteM),
    .ALUResultM(DataAdrM),
    .WriteDataM(WriteDataM),
    .ReadDataM(ReadDataM),
    .cycle_count(cycle_count),
    .instr_retired(instr_retired)
  );

  imem imem (
    .a (PCF),
    .rd(InstrF)
  );

  dmem dmem (
    .clk(clk),
    .we(MemWriteM),
    .a (DataAdrM),
    .wd(WriteDataM),
    .rd(ReadDataM)
  );

endmodule


// Controller Module
module controller(
  input  logic       clk,
  input  logic       reset,
  input  logic [6:0] opD,
  input  logic [2:0] funct3D,
  input  logic [6:0] funct7D,
  input  logic       funct7b5D,
  output logic [1:0] ImmSrcD,
  input  logic       FlushE,
  input  logic       ZeroE,
  output logic       PCSrcE,
  output logic [3:0] ALUControlE,
  output logic       ALUSrcE,
  output logic       ResultSrcEb0,
  output logic       MemWriteM,
  output logic       RegWriteM,
  output logic       RegWriteW,
  output logic [1:0] ResultSrcW
);

  logic       RegWriteD, RegWriteE;
  logic [1:0] ResultSrcD, ResultSrcE, ResultSrcM;
  logic       MemWriteD, MemWriteE;
  logic       JumpD, JumpE;
  logic       BranchD, BranchE;
  logic [1:0] ALUOpD;
  logic [3:0] ALUControlD;
  logic       ALUSrcD;

  // Decode stage
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

  aludec ad(
    .opb5(opD[5]),
    .funct3(funct3D),
    .funct7(funct7D),
    .funct7b5(funct7b5D),
    .ALUOp(ALUOpD),
    .ALUControl(ALUControlD)
  );

  // Execute stage pipeline register
  floprc #(11) controlregE(
    .clk(clk),
    .reset(reset),
    .clear(FlushE),
    .d({RegWriteD, ResultSrcD, MemWriteD, JumpD, BranchD, ALUControlD, ALUSrcD}),
    .q({RegWriteE, ResultSrcE, MemWriteE, JumpE, BranchE, ALUControlE, ALUSrcE})
  );

  assign PCSrcE = (BranchE & ZeroE) | JumpE;
  assign ResultSrcEb0 = ResultSrcE[0];

  // Memory stage pipeline register
  flopr #(4) controlregM(
    .clk(clk),
    .reset(reset),
    .d({RegWriteE, ResultSrcE, MemWriteE}),
    .q({RegWriteM, ResultSrcM, MemWriteM})
  );

  // Writeback stage pipeline register
  flopr #(3) controlregW(
    .clk(clk),
    .reset(reset),
    .d({RegWriteM, ResultSrcM}),
    .q({RegWriteW, ResultSrcW})
  );

endmodule


// Main Decoder

module maindec(
  input  logic [6:0] op,
  output logic [1:0] ResultSrc,
  output logic       MemWrite,
  output logic       Branch,
  output logic       ALUSrc,
  output logic       RegWrite,
  output logic       Jump,
  output logic [1:0] ImmSrc,
  output logic [1:0] ALUOp
);

  logic [10:0] controls;
  assign {RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb begin
    case(op)
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_00_0_0_00_0_10_0; // R-type
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      7'b0001011: controls = 11'b1_00_0_0_00_0_11_0; // RVX10 (custom-0)
      default:    controls = 11'b0_00_0_0_00_0_00_0; // default value execute during nope also
    endcase
  end

endmodule


// ALU Decoder

module aludec(
  input  logic       opb5,
  input  logic [2:0] funct3,
  input  logic [6:0] funct7,
  input  logic       funct7b5,
  input  logic [1:0] ALUOp,
  output logic [3:0] ALUControl
);

  logic RtypeSub;
  assign RtypeSub = funct7b5 & opb5;

  always_comb begin
    case(ALUOp)
      2'b00: ALUControl = 4'b0000; // addition
      2'b01: ALUControl = 4'b0001; // subtraction
      2'b11: begin // RVX10 custom instructions
        case(funct7)
          7'b0000000: begin
            case(funct3)
              3'b000: ALUControl = 4'b1000; // ANDN
              3'b001: ALUControl = 4'b1001; // ORN
              3'b010: ALUControl = 4'b1010; // XNOR
              default: ALUControl = 4'b0000;
            endcase
          end
          7'b0000001: begin
            case(funct3)
              3'b000: ALUControl = 4'b1011; // MIN
              3'b001: ALUControl = 4'b1100; // MAX
              3'b010: ALUControl = 4'b1101; // MINU
              3'b011: ALUControl = 4'b1110; // MAXU
              default: ALUControl = 4'b0000;
            endcase
          end
          7'b0000010: begin
            case(funct3)
              3'b000: ALUControl = 4'b0110; // ROL
              3'b001: ALUControl = 4'b0111; // ROR
              default: ALUControl = 4'b0000;
            endcase
          end
          7'b0000011: begin
            case(funct3)
              3'b000: ALUControl = 4'b1111; // ABS
              default: ALUControl = 4'b0000;
            endcase
          end
          default: ALUControl = 4'b0000;
        endcase
      end
      2'b10: begin // Standard R-type and I-type ALU
        case(funct3)
          3'b000: ALUControl = RtypeSub ? 4'b0001 : 4'b0000; // sub or add
          3'b010: ALUControl = 4'b0101; // slt
          3'b110: ALUControl = 4'b0011; // or
          3'b111: ALUControl = 4'b0010; // and
          default: ALUControl = 4'b0000;
        endcase
      end
      default: ALUControl = 4'b0000;
    endcase
  end

endmodule


// Datapath Module with Valid Bit Pipeline

module datapath (
  input  logic        clk,
  input  logic        reset,
  input  logic        StallF,
  output logic [31:0] PCF,
  input  logic [31:0] InstrF,
  output logic [ 6:0] opD,
  output logic [ 2:0] funct3D,
  output logic [ 6:0] funct7D,
  output logic        funct7b5D,
  input  logic        StallD,
  input  logic        FlushD,
  input  logic [ 1:0] ImmSrcD,
  input  logic        FlushE,
  input  logic [ 1:0] ForwardAE,
  input  logic [ 1:0] ForwardBE,
  input  logic        PCSrcE,
  input  logic [ 3:0] ALUControlE,
  input  logic        ALUSrcE,
  output logic        ZeroE,
  input  logic        MemWriteM,
  output logic [31:0] WriteDataM,
  output logic [31:0] ALUResultM,
  input  logic [31:0] ReadDataM,
  input  logic        RegWriteW,
  input  logic [ 1:0] ResultSrcW,
  output logic [ 4:0] Rs1D,
  output logic [ 4:0] Rs2D,
  output logic [ 4:0] Rs1E,
  output logic [ 4:0] Rs2E,
  output logic [ 4:0] RdE,
  output logic [ 4:0] RdM,
  output logic [ 4:0] RdW,
  output logic        ValidD,
  output logic        ValidE,
  output logic        ValidM,
  output logic        ValidW
);

  // Fetch stage signals
  logic [31:0] PCNextF, PCPlus4F;
  logic        ValidF;
  
  // Decode stage signals
  logic [31:0] InstrD;
  logic [31:0] PCD, PCPlus4D;
  logic [31:0] RD1D, RD2D;
  logic [31:0] ImmExtD;
  logic [ 4:0] RdD;
  
  // Execute stage signals
  logic [31:0] RD1E, RD2E;
  logic [31:0] PCE, ImmExtE;
  logic [31:0] SrcAE, SrcBE;
  logic [31:0] SrcAEforward;
  logic [31:0] ALUResultE;
  logic [31:0] WriteDataE;
  logic [31:0] PCPlus4E;
  logic [31:0] PCTargetE;
  
  // Memory stage signals
  logic [31:0] PCPlus4M;
  
  // Writeback stage signals
  logic [31:0] ALUResultW;
  logic [31:0] ReadDataW;
  logic [31:0] PCPlus4W;
  logic [31:0] ResultW;

  // ========== Valid Bit Pipeline ==========
  // ValidF is always 1 after reset (fetch is always valid)
  assign ValidF = 1'b1;
  
  // ValidD: clear on flush, hold on stall, otherwise propagate ValidF
  flopenrc #(1) validregD (
    .clk(clk),
    .reset(reset),
    .clear(FlushD),
    .en(~StallD),
    .d(ValidF),
    .q(ValidD)
  );
  
  // ValidE: clear on flush, otherwise propagate ValidD
  floprc #(1) validregE (
    .clk(clk),
    .reset(reset),
    .clear(FlushE),
    .d(ValidD),
    .q(ValidE)
  );
  
  // ValidM: propagate ValidE
  flopr #(1) validregM (
    .clk(clk),
    .reset(reset),
    .d(ValidE),
    .q(ValidM)
  );
  
  // ValidW: propagate ValidM
  flopr #(1) validregW (
    .clk(clk),
    .reset(reset),
    .d(ValidM),
    .q(ValidW)
  );

  // ========== Fetch Stage ==========
  mux2 #(32) pcmux (
    .d0(PCPlus4F),
    .d1(PCTargetE),
    .s (PCSrcE),
    .y (PCNextF)
  );

  flopenr #(32) pcreg (
    .clk(clk),
    .reset(reset),
    .en (~StallF),
    .d  (PCNextF),
    .q  (PCF)
  );

  adder pcadd (
    .a(PCF),
    .b(32'd4),
    .s(PCPlus4F)
  );

  // ========== Decode Stage ==========
  flopenrc #(96) regD (
    .clk(clk),
    .reset(reset),
    .clear(FlushD),
    .en (~StallD),
    .d  ({InstrF, PCF, PCPlus4F}),
    .q  ({InstrD, PCD, PCPlus4D})
  );

  assign opD = InstrD[6:0];
  assign funct3D = InstrD[14:12];
  assign funct7D = InstrD[31:25];
  assign funct7b5D = InstrD[30];
  assign Rs1D = InstrD[19:15];
  assign Rs2D = InstrD[24:20];
  assign RdD = InstrD[11:7];

  regfile rf (
    .clk(clk),
    .a1 (Rs1D),
    .a2 (Rs2D),
    .rd1(RD1D),
    .rd2(RD2D),
    .a3 (RdW),
    .we3(RegWriteW),
    .wd3(ResultW)
  );

  extend ext (
    .instr (InstrD[31:7]),
    .immsrc(ImmSrcD),
    .immext(ImmExtD)
  );

  // ========== Execute Stage ==========
  floprc #(175) regE (
    .clk(clk),
    .reset(reset),
    .clear(FlushE),
    .d  ({RD1D, RD2D, PCD, Rs1D, Rs2D, RdD, ImmExtD, PCPlus4D}),
    .q  ({RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E})
  );

  // Forward A mux
  mux3 #(32) faemux (
    .d0(RD1E),
    .d1(ResultW),
    .d2(ALUResultM),
    .s (ForwardAE),
    .y (SrcAEforward)
  );

  assign SrcAE = SrcAEforward;

  // Forward B mux
  mux3 #(32) fbemux (
    .d0(RD2E),
    .d1(ResultW),
    .d2(ALUResultM),
    .s (ForwardBE),
    .y (WriteDataE)
  );

  // SrcB mux: select between forwarded register or immediate
  mux2 #(32) srcbmux (
    .d0(WriteDataE),
    .d1(ImmExtE),
    .s (ALUSrcE),
    .y (SrcBE)
  );

  alu alu (
    .a         (SrcAE),
    .b         (SrcBE),
    .alucontrol(ALUControlE),
    .result    (ALUResultE),
    .zero      (ZeroE)
  );

  adder branchadd (
    .a(ImmExtE),
    .b(PCE),
    .s(PCTargetE)
  );

  // Memory Stage 
  flopr #(101) regM (
    .clk(clk),
    .reset(reset),
    .d  ({ALUResultE, WriteDataE, RdE, PCPlus4E}),
    .q  ({ALUResultM, WriteDataM, RdM, PCPlus4M})
  );

  // Writeback Stage 
  flopr #(101) regW (
    .clk(clk),
    .reset(reset),
    .d  ({ALUResultM, ReadDataM, RdM, PCPlus4M}),
    .q  ({ALUResultW, ReadDataW, RdW, PCPlus4W})
  );

  mux3 #(32) resultmux (
    .d0(ALUResultW),
    .d1(ReadDataW),
    .d2(PCPlus4W),
    .s (ResultSrcW),
    .y (ResultW)
  );

endmodule


module alu(
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic [ 3:0] alucontrol,
  output logic [31:0] result,
  output logic        zero
);

  logic [31:0] condinvb, sum;
  logic        v, isAddSub;
  logic [ 4:0] shamt;
  logic signed [31:0] as, bs;

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[3] & ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[3] & ~alucontrol[2] & alucontrol[1] & alucontrol[0];
  assign shamt = b[4:0];
  assign as = $signed(a);
  assign bs = $signed(b);

  always_comb begin
    case (alucontrol)
      4'b0000: result = sum;                                    // ADD
      4'b0001: result = sum;                                    // SUB
      4'b0010: result = a & b;                                  // AND
      4'b0011: result = a | b;                                  // OR
      4'b0100: result = a ^ b;                                  // XOR
      4'b0101: result = {31'b0, sum[31] ^ v};                   // SLT
      4'b0110: result = (shamt == 0) ? a : (a << shamt) | (a >> (32 - shamt)); // ROL
      4'b0111: result = (shamt == 0) ? a : (a >> shamt) | (a << (32 - shamt)); // ROR
      4'b1000: result = a & ~b;                                 // ANDN
      4'b1001: result = a | ~b;                                 // ORN
      4'b1010: result = ~(a ^ b);                               // XNOR
      4'b1011: result = (as < bs) ? a : b;                      // MIN
      4'b1100: result = (as > bs) ? a : b;                      // MAX
      4'b1101: result = (a < b) ? a : b;                        // MINU
      4'b1110: result = (a > b) ? a : b;                        // MAXU
      4'b1111: result = (as[31]) ? (~a + 1'b1) : a;            // ABS
      default: result = 32'b0;
    endcase
  end

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;

endmodule

module regfile (
  input  logic          clk,
  input  logic [ 4 : 0] a1,
  input  logic [ 4 : 0] a2,
  output logic [31 : 0] rd1,
  output logic [31 : 0] rd2,
  input  logic [ 4 : 0] a3,
  input  logic          we3,
  input  logic [31 : 0] wd3
);

  logic [31 : 0] rf[32];
    // Initialize all registers to zero
  initial begin
    for (int i = 0; i < 32; i = i + 1) begin
      rf[i] = 32'b0;
    end
  end

  always_ff @(negedge clk) begin   
    if (we3 && (a3 != 5'b0)) rf[a3] <= wd3;
  end

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;

endmodule

module extend (
  input  logic [31:7] instr,
  input  logic [ 1:0] immsrc,
  output logic [31:0] immext
);

  always_comb begin
    case (immsrc)
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};                          // I-type
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};            // S-type
      2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type
      default: immext = 32'bx;
    endcase
  end

endmodule

module adder (
  input  [31:0] a,
  input  [31:0] b,
  output [31:0] s
);
  assign s = a + b;
endmodule

module mux2 #(parameter WIDTH = 32) (
  input  logic [WIDTH-1:0] d0,
  input  logic [WIDTH-1:0] d1,
  input  logic             s,
  output logic [WIDTH-1:0] y
);
  assign y = s ? d1 : d0;
endmodule

module mux3 #(parameter WIDTH = 32) (
  input  logic [WIDTH-1:0] d0,
  input  logic [WIDTH-1:0] d1,
  input  logic [WIDTH-1:0] d2,
  input  logic [      1:0] s,
  output logic [WIDTH-1:0] y
);
  assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule

// Pipeline registers with enable
module flopenr #(parameter WIDTH = 32) (
  input  logic             clk,
  input  logic             reset,
  input  logic             en,
  input  logic [WIDTH-1:0] d,
  output logic [WIDTH-1:0] q
);
  always_ff @(posedge clk or posedge reset) begin
    if (reset)      q <= 0;
    else if (en)  q <= d;
  end
endmodule

// Pipeline registers with enable and clear
module flopenrc #(parameter WIDTH = 32) (
  input  logic             clk,
  input  logic             reset,
  input  logic             clear,
  input  logic             en,
  input  logic [WIDTH-1:0] d,
  output logic [WIDTH-1:0] q
);
  always_ff @(posedge clk or posedge reset) begin
    if (reset)      q <= 0;
    else if (clear) q <= 0;
    else if (en)  q <= d;
  end
endmodule

// Basic pipeline register
module flopr #(parameter WIDTH = 32) (
  input  logic             clk,
  input  logic             reset,
  input  logic [WIDTH-1:0] d,
  output logic [WIDTH-1:0] q
);
  always_ff @(posedge clk or posedge reset) begin
    if (reset) q <= 0;
    else     q <= d;
  end
endmodule

// Pipeline register with clear
module floprc #(parameter WIDTH = 32) (
  input  logic             clk,
  input  logic             reset,
  input  logic             clear,
  input  logic [WIDTH-1:0] d,
  output logic [WIDTH-1:0] q
);
  always_ff @(posedge clk or posedge reset) begin
    if (reset)       q <= 0;
    else if (clear)  q <= 0;
    else             q <= d;
  end
endmodule


module imem (
  input  logic [31:0] a,
  output logic [31:0] rd
);
  logic [31:0] RAM[64];
  
initial begin
    // Initialize to zero first (safety)
    for (int i = 0; i < 64; i = i + 1) begin
      RAM[i] = 32'b0;
    end
    // Then load program (this will overwrite zeros)
    $readmemh("rvx10.txt", RAM);
  end

  assign rd = RAM[a[31:2]];  // word aligned
endmodule

module dmem (
  input  logic        clk,
  input  logic        we,
  input  logic [31:0] a,
  input  logic [31:0] wd,
  output logic [31:0] rd
);
  logic [31:0] RAM[64];
  // Initialize all memory locations to zero
  initial begin
    for (int i = 0; i < 64; i = i + 1) begin
      RAM[i] = 32'b0;
    end
  end

  assign rd = RAM[a[31:2]];  // word aligned

  always_ff @(posedge clk) begin
    if (we) RAM[a[31:2]] <= wd;
  end
endmodule



