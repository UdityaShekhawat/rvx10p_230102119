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
