module alu(
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic [ 3:0] alucontrol,
  output logic [31:0] result,
  output logic        zero
);

  logic [31:0] condinvb, sum;
  logic        v, isAddSub;
  logic [ 4:0] shamt;
  logic signed [31:0] as, bs;

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[3] & ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[3] & ~alucontrol[2] & alucontrol[1] & alucontrol[0];
  assign shamt = b[4:0];
  assign as = $signed(a);
  assign bs = $signed(b);

  always_comb begin
    case (alucontrol)
      4'b0000: result = sum;                                    // ADD
      4'b0001: result = sum;                                    // SUB
      4'b0010: result = a & b;                                  // AND
      4'b0011: result = a | b;                                  // OR
      4'b0100: result = a ^ b;                                  // XOR
      4'b0101: result = {31'b0, sum[31] ^ v};                   // SLT
      4'b0110: result = (shamt == 0) ? a : (a << shamt) | (a >> (32 - shamt)); // ROL
      4'b0111: result = (shamt == 0) ? a : (a >> shamt) | (a << (32 - shamt)); // ROR
      4'b1000: result = a & ~b;                                 // ANDN
      4'b1001: result = a | ~b;                                 // ORN
      4'b1010: result = ~(a ^ b);                               // XNOR
      4'b1011: result = (as < bs) ? a : b;                      // MIN
      4'b1100: result = (as > bs) ? a : b;                      // MAX
      4'b1101: result = (a < b) ? a : b;                        // MINU
      4'b1110: result = (a > b) ? a : b;                        // MAXU
      4'b1111: result = (as[31]) ? (~a + 1'b1) : a;            // ABS
      default: result = 32'b0;
    endcase
  end

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;

endmodule
