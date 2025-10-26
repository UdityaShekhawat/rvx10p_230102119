


# Test the RISC-V (RVX10_pipeline) processor.  

 add,sub, and, or, slt, addi, lw, sw, beq, jal
 
 andn, orn,xnor, abs, ror, rol, min, minu, max, maxu 


## If successful, it should write the value 25 to address 100

#       RISC-V Assembly         Description               Address   Machine Code

        addi x2, x0, 5          # x2 = 5                  0         00500113
																					
        addi x3, x0, 12         # x3 = 12                 4         00C00193
		
        addi x7, x3, -9         # x7 = (12 - 9) = 3       8         FF718393
		
        or   x4, x7, x2         # x4 = (3 OR 5) = 7       C         0023E233
		
        and  x5, x3, x4         # x5 = (12 AND 7) = 4     10        0041F2B3
		
        add  x5, x5, x4         # x5 = (4 + 7) = 11       14        004282B3
		
        beq  x5, x7, end        # shouldn't be taken      18        02728863
		
        slt  x4, x3, x4         # x4 = (12 < 7) = 0       1C        0041A233
		
        beq  x4, x0, around     # should be taken         20        00020463
		
        addi x5, x0, 0          # shouldn't happen        24        00000293
		
        slt  x4, x7, x2         # x4 = (3 < 5)  = 1       28        0023A233

        add  x7, x4, x5         # x7 = (1 + 11) = 12      2C        005203B3
		
        sub  x7, x7, x2         # x7 = (12 - 5) = 7       30        402383B3
		
        sw   x7, 84(x3)         # [96] = 7                34        0471AA23 
		
        lw   x2, 96(x0)         # x2 = [96] = 7           38        06002103 
		
        add  x9, x2, x5         # x9 = (7 + 11) = 18      3C        005104B3
		
        jal  x3, end            # jump to end, x3 = 0x44  40        008001EF
		
        addi x2, x0, 1          # shouldn't happen        44        00100113
		
        add  x2, x2, x9         # x2 = (7 + 18)  = 25     48        00910133
		
        andn x7, x4, x5         # x7 = (1 and ~11)=0      4C        0052038B
		
        orn  x7, x4, x5         # x7=(1 or ~11)= -11      50        0052138B
		
        abs  x7, x7, x0         # x7 = abs|-11| =11       54        0603838B 
		
        addi x1 ,x0,-128        #x1 =-128                 58        F8000093 
		
        abs  x1, x0, x1         #x1=128                   5C        0600808B
		
        xnor x4, x1, x7         #x4=0xFFFFF74             60        0070A20B
		
        addi x5, x0,2           # x5=2                    64        00200293
		
        ror  x4, x1,x5          #x4=32                    68        0450920B
		
        rol  x4,x4,x5           #x4=128                   6C        0452020B
		
        rol  x4,x4,x0           #x4 =128 no change        70        0402020B
		
        MIN x1, x4, x5          #x1=min(128,2)=2          74        0252008B
		
        MAX x1, x4, x7          #x1=max(128,11)=128       78        0272108B 
		
        MINU x1, x5, x7         #x1 = 2                   7C        0272A08B 
		
        MAXU x1, x4, x7         #x1 = 128                 80        0272308B 
		
        addi x4, x0, -10        #x4=-10                   84        FF600213
		
        addi x5, x0, 20         #x5=20                    88        01400293
		
        addi x7, x0, -5         #x7=-5                    8C        FFB00393
		
        MIN x1, x4, x5          # MIN(-10, 20) = -10      90        0252008B
		
        MINU x1, x4, x5         # MINU(-10, 20) = 20      94        0252A08B
		
        MIN x1, x4, x7          #MIN(-10, -5) = -10       98        0272008B
		
        MINU x1, x4, x7         #MINU(-10, 20) = 20       9C        0272A08B
		
        MAX x1, x4, x5          # MAX(-10, 20) = 20      100        0252108B
		
        MAXU x1, x4, x5         #MAXU(-10, 20) = -10     104        0252308B
		
        MAX x1, x4, x7          # MAX(-10, -5) = -5      108        0272108B 
		
        MAXU x1, x4, x7         # MAXU(-10, -5) = -5     10C        0272308B
		
        addi x0, x0, 1          # x0=0,should not change 200        00100013
		
        sw   x2, 0x20(x3)       # mem[100] = 25          204        0221A023
		
        beq  x2, x2, done       # infinite loop          208        00210063


author: uditya_shekhawat_230102119
		
		
