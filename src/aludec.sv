module aludec(
  input  logic       opb5,
  input  logic [2:0] funct3,
  input  logic [6:0] funct7,
  input  logic       funct7b5,
  input  logic [1:0] ALUOp,
  output logic [3:0] ALUControl
);

  logic RtypeSub;
  assign RtypeSub = funct7b5 & opb5;

  always_comb begin
    case(ALUOp)
      2'b00: ALUControl = 4'b0000; // addition
      2'b01: ALUControl = 4'b0001; // subtraction
      2'b11: begin // RVX10 custom instructions
        case(funct7)
          7'b0000000: begin
            case(funct3)
              3'b000: ALUControl = 4'b1000; // ANDN
              3'b001: ALUControl = 4'b1001; // ORN
              3'b010: ALUControl = 4'b1010; // XNOR
              default: ALUControl = 4'b0000;
            endcase
          end
          7'b0000001: begin
            case(funct3)
              3'b000: ALUControl = 4'b1011; // MIN
              3'b001: ALUControl = 4'b1100; // MAX
              3'b010: ALUControl = 4'b1101; // MINU
              3'b011: ALUControl = 4'b1110; // MAXU
              default: ALUControl = 4'b0000;
            endcase
          end
          7'b0000010: begin
            case(funct3)
              3'b000: ALUControl = 4'b0110; // ROL
              3'b001: ALUControl = 4'b0111; // ROR
              default: ALUControl = 4'b0000;
            endcase
          end
          7'b0000011: begin
            case(funct3)
              3'b000: ALUControl = 4'b1111; // ABS
              default: ALUControl = 4'b0000;
            endcase
          end
          default: ALUControl = 4'b0000;
        endcase
      end
      2'b10: begin // Standard R-type and I-type ALU
        case(funct3)
          3'b000: ALUControl = RtypeSub ? 4'b0001 : 4'b0000; // sub or add
          3'b010: ALUControl = 4'b0101; // slt
          3'b110: ALUControl = 4'b0011; // or
          3'b111: ALUControl = 4'b0010; // and
          default: ALUControl = 4'b0000;
        endcase
      end
      default: ALUControl = 4'b0000;
    endcase
  end

endmodule
