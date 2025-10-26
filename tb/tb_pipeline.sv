`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Uditya Shekhawat
// 
// Create Date: 25.10.2025 18:06:13
// Design Name: 
// Module Name: TESTBENCH
// Project Name: Testbench
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
// =============================================================================
// Testbench with CPI Display
// =============================================================================

module tb_pipeline();
  logic clk;
  logic reset;
  logic [31:0] WriteData, DataAdr;
  logic MemWrite;
  logic [31:0] cycle_count, instr_retired;
  real cpi;

  top dut(clk, reset, WriteData, DataAdr, MemWrite, cycle_count, instr_retired);
  
  initial begin
    reset <= 1; #22; reset <= 0;
  end

  initial begin
    $dumpfile("wave.vcd");
    $dumpvars(0,tb_pipeline);
  end
  
  always begin
    clk <= 1; #5; clk <= 0; #5;
  end

  always @(negedge clk) begin
    if(MemWrite) begin
      $display("MemWrite asserted: Addr=%0d Data=%0d", DataAdr, WriteData);
      if(DataAdr === 100 && WriteData === 25) begin
        // Calculate and display CPI
        if (instr_retired > 0) begin
          cpi = $itor(cycle_count) / $itor(instr_retired);
          $display("========================================");
          $display("Simulation succeeded!");
          $display("========================================");
          $display("Performance Metrics:");
          $display("  Total Cycles:         %0d", cycle_count);
          $display("  Instructions Retired: %0d", instr_retired);
          $display("  CPI (Cycles Per Instr): %.3f", cpi);
          $display("========================================");
        end else begin
          $display("Simulation succeeded (no instructions retired yet)");
        end
        $stop;
      end else if (DataAdr !== 96) begin
        $display("Simulation failed");
        $stop;
      end
    end
  end
  
  // Optional: Display periodic updates
  always @(posedge clk) begin
    if (!reset && (cycle_count % 10 == 0) && (cycle_count > 0)) begin
      if (instr_retired > 0) begin
        cpi = $itor(cycle_count) / $itor(instr_retired);
        $display("Cycle %0d: Instructions=%0d, CPI=%.3f", 
                 cycle_count, instr_retired, cpi);
      end
    end
  end
  
endmodule
