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
