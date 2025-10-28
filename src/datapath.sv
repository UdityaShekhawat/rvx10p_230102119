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
