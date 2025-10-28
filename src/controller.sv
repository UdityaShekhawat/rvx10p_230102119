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
