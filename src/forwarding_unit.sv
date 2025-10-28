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
