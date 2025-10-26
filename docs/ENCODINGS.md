



<img width="1068" height="493" alt="image" src="https://github.com/user-attachments/assets/bb80d2f4-d154-4d92-9980-6522940e0c21" />








<img width="1067" height="481" alt="image" src="https://github.com/user-attachments/assets/22cc0bc1-b8f8-4c2f-b002-a4e071eff5ef" />



31 32-bit registers x1-x31, x0 hardwired to 0
## R-Type instructions
    add, sub, and, or, slt, RVX10 ops  
    INSTR rd, rs1, rs2
    Instr[31:25] = funct7 (funct7b5 & opb5 = 1 for sub, 0 for others)
    Instr[24:20] = rs2
    Instr[19:15] = rs1
    Instr[14:12] = funct3
    Instr[11:7]  = rd
    Instr[6:0]   = opcode
## I-Type Instructions
    lw, I-type ALU (addi, andi, ori, slti)
    lw:         INSTR rd, imm(rs1)
    I-type ALU: INSTR rd, rs1, imm (12-bit signed)
    Instr[31:20] = imm[11:0]
    Instr[24:20] = rs2
    Instr[19:15] = rs1
    Instr[14:12] = funct3
    Instr[11:7]  = rd
    Instr[6:0]   = opcode
## S-Type Instruction
    sw rs2, imm(rs1) (store rs2 into address specified by rs1 + immm)
    Instr[31:25] = imm[11:5] (offset[11:5])
    Instr[24:20] = rs2 (src)
    Instr[19:15] = rs1 (base)
    Instr[14:12] = funct3
    Instr[11:7]  = imm[4:0]  (offset[4:0])
    Instr[6:0]   = opcode
## B-Type Instruction
     beq rs1, rs2, imm (PCTarget = PC + (signed imm x 2))
    Instr[31:25] = imm[12], imm[10:5]
    Instr[24:20] = rs2
    Instr[19:15] = rs1
    Instr[14:12] = funct3
    Instr[11:7]  = imm[4:1], imm[11]
    Instr[6:0]   = opcode
## J-Type Instruction
    jal rd, imm  (signed imm is multiplied by 2 and added to PC, rd = PC+4)
    Instr[31:12] = imm[20], imm[10:1], imm[11], imm[19:12]
    Instr[11:7]  = rd
    Instr[6:0]   = opcode






## ALU Control Mapping

## ALUControl[3:0]  Operation

   0000             ADD
   
   0001             SUB
    
   0010             AND
   
   0011             OR
   
   0100             XOR
   
   0101             SLT
   
   0110             ROL
   
   0111             ROR
   
   1000             ANDN
   
   1001             ORN
   
   1010             XNOR
   
   1011             MIN
   
   1100             MAX
   
   1101             MINU
   
   1110             MAXU
   
   1111             ABS
