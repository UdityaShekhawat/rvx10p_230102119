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
