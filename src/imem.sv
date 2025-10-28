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
