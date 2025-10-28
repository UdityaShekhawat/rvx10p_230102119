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
