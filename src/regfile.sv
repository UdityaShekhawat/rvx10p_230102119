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
