`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg, branch;
  wire [0:0]  pcsrc, zero;
  wire        alusrc, regdst, regwrite; 
  wire        jump, jal, jr;
  wire [3:0]  alucontrol;
  wire [1:0]  aluop;
  wire        notequal;
  wire        memread;
  wire        memwriteID;
  wire [31:0] memaddrEX;

  wire [4:0]  writereg;
  wire [31:0] pcplus4, pcnextbr, pcbranch, pcnextbrj, pcnextfinal;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
  wire        shift;

  wire [4:0] regaddr;
  wire [31:0] regdata;
  wire [31:0] memwritedataID;
  wire [31:0] pcplus4MEM;

  wire [31:0] srcaID;
  wire [31:0] srcbID;

  wire stall;
  wire flush;
  wire pcsrcEX;
  wire [31:0] pcnextbrjjr;

  // ###### Lee GeonWoo : Start #######
  // Add 4 FF to implement pipeline

  wire [31:0] instrIF;
  mux2 #(32) flushmuxIF(
    .d0  (instr),
    .d1  (32'b0),
    .s   (flush),
    .y   (instrIF));


  // PC(32) + Instr(32) = 64
  wire [31:0] PCID, instrID, instrtmp;
  flopenr #(64) IFID(
    .clk   (clk),
    .reset (reset),
    .en    (~stall),
    .d     ({pcplus4, instrIF}),
    .q     ({PCID, instrtmp})
    );

  // Mux for stalling control bits.
  wire [13:0] controlID;
  muxr2 #(14) controlmux(
    .d0    ({regwrite, memtoreg, jal, branch, notequal, memread, memwriteID, jump, regdst, aluop, alusrc, shiftl16, signext}),
    .d1    (14'b0),
    .s     (stall | flush),
    .reset (reset),
    .y     (controlID)
    );

  mux2 #(32) flushmuxID(
    .d0  (instrtmp),
    .d1  (32'b0),
    .s   (flush),
    .y   (instrID));



  // control(13) + PC(32) + rd1(32) + rd(32) + signimm(32) + instr(32) + rs(5) + rt(5) + rd(5) = 188
  wire [7:0] controlEX;
  wire [0:0] regdstEX, alusrcEX, shiftl16EX;
  wire [1:0] aluopEX;
  wire [31:0] PCEX, rd1EX, rd2EX, signimmEX;
  wire [4:0] rsEX, rtEX, rdEX;
  wire [31:0] instrEX;
  flopenr #(188) IDEX(
    .clk   (clk),
    .reset (reset),
    .en    (1'b1),
    .d     ({controlID[13:1], PCID, srca, memwritedataID, signimm[31:0],
             instrID, instrtmp[25:21], instrtmp[20:16], instrtmp[15:11]}),
    .q     ({controlEX, regdstEX, aluopEX, alusrcEX, shiftl16EX, PCEX, rd1EX, rd2EX,
             signimmEX, instrEX, rsEX, rtEX, rdEX})
    );

  // Flushing Logic.
  // wire [7:0] controlEX;
  // controlMEM, branchMEM, notequalMEM, memreadMEM, memwrite, jumpMEM
  // regwriteWB, memtoregWB, jalWB
  assign pcsrcEX = (controlEX[4] & (controlEX[3] ^ zero));
  assign flush = pcsrcEX | controlEX[0] | controlEX[5] | jr;


  // control(9) + PC(32) + PC(32) + instr(32) + zero(1) + aluout(32) + wd(32) + wr(5) + rd1(32) = 207
  wire [2:0] controlMEM;
  wire [0:0] branchMEM, notequalMEM, memreadMEM, jrMEM, jumpMEM, zeroMEM; //memwriteMEM
  wire [31:0] PCMEM, aluoutMEM, instrMEM, wdMEM;
  wire [4:0] wrMEM;
  wire [31:0] rd1MEM;

  flopenr #(207) EXMEM(
    .clk   (clk),
    .reset (reset),
    .en    (1'b1),
    .d     ({controlEX, jr, PCEX, pcbranch, instrEX, zero, memaddrEX, rd2EX, writereg, rd1EX}),
    .q     ({controlMEM, branchMEM, notequalMEM, memreadMEM, memwrite, jumpMEM, jrMEM, 
             pcplus4MEM, PCMEM, instrMEM, zeroMEM, memaddr, memwritedata, wrMEM, rd1MEM})
    );

  //assign pcsrcMEM = branchMEM & (zeroMEM ^ notequalMEM);


  // control(3) + wd1(32) + wd2(32) + wr(5) + PC(32) = 104
  wire [0:0] regwriteWB, memtoregWB, jalWB;
  wire [31:0] wd1WB, wd2WB, pcplus4WB;
  wire [4:0] wrWB;

  flopenr #(104) MEMWB(
    .clk   (clk),
    .reset (reset),
    .en    (1'b1),
    .d     ({controlMEM, memreaddata, memaddr, wrMEM, pcplus4MEM}),
    .q     ({regwriteWB, memtoregWB, jalWB, wd2WB, wd1WB, wrWB, pcplus4WB})
    );



  // Forwarding unit and Data hazard detection unit
  wire [5:0] muxcontrol;
  forward fw(
    .rsID         (instrID[25:21]),
    .rtID         (instrID[20:16]),
    .rsEX         (rsEX),
    .rtEX         (rtEX),
    .wrMEM        (wrMEM),
    .wrWB         (regaddr),
    .rwMEM        (controlMEM[2]),
    .rwWB         (regwriteWB),
    .reset        (reset),
    .muxcontrol   (muxcontrol)
    );


  hazard_detection hd (
    .rsID        (instrID[25:21]),
    .rtID        (instrID[20:16]),
    .rtEX        (rtEX),
    .rtMEM       (wrMEM),
    .memreadEX   (controlEX[2]),
    .memreadMEM  (memreadMEM),
    .reset       (reset),
    .stall       (stall)
    );


  // ###### Lee GeonWoo : End #######


  maindec md(
    .op       (instrID[31:26]),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwriteID),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .jal      (jal),
    .aluop    (aluop),
    .notequal (notequal),
    .memread  (memread));

  aludec ad( 
    .funct      (signimmEX[5:0]),
    .aluop      (aluopEX), 
    .alucontrol (alucontrol),
    .jr         (jr));



  adder pcadd4(
    .a (pc),
    .b (32'h00000004),
    .y (pcplus4));

  sl2 immsh(
    .a (signimmEX),
    .y (signimmsh));

// ###### Lee GeonWoo : Start #######
// Change flow of circuit about PC(MEM -> EX)

  adder pcaddbr(
    .a (PCEX),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (PCEX),
    .d1  (pcbranch),
    .s   (pcsrcEX),
    .y   (pcnextbr));

  mux2 #(32) pcjumpmux(
    .d0   (pcnextbr),
    .d1   ({PCEX[31:28], instrEX[25:0], 2'b00}),
    .s    (controlEX[0]),
    .y    (pcnextbrj));

  mux2 #(32) pcjrmux(
    .d0   (pcnextbrj),
    .d1   (rd1EX),
    .s    (jr),
    .y    (pcnextbrjjr));

  mux2 #(32) pcfinalmux(
    .d0   (pcplus4),
    .d1   (pcnextbrjjr),
    .s    (flush),
    .y    (pcnextfinal)
    );
// ###### Lee GeonWoo : End #######

  // next PC logic
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .en    (~stall),
    .d     (pcnextfinal),
    .q     (pc));



  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwriteWB),
    .ra1     (instrID[25:21]),
    .ra2     (instrID[20:16]),
    .wa      (regaddr),
    .wd      (regdata),
    .rd1     (srcaID),
    .rd2     (srcbID));

  mux2 #(32) IDforwardmux1(
    .d0      (srcaID),
    .d1      (regdata),
    .s       (muxcontrol[4]),
    .y       (srca)
    );

  mux2 #(32) IDforwardmux2(
    .d0      (srcbID),
    .d1      (regdata),
    .s       (muxcontrol[5]),
    .y       (memwritedataID)
    );

  mux2 #(5) wrmux(
    .d0  (rtEX),
    .d1  (rdEX),
    .s   (regdstEX),
    .y   (writereg));

  mux2 #(5) wrjalmux(
    .d0  (wrWB),
    .d1  (5'b11111),
    .s   (jalWB),
    .y   (regaddr));

  mux2 #(32) resmux(
    .d0 (wd1WB),
    .d1 (wd2WB),
    .s  (memtoregWB),
    .y  (result));

  mux2 #(32) resjalmux(
    .d0 (result),
    .d1 (pcplus4WB),
    .s  (jalWB),
    .y  (regdata));

  sign_zero_ext sze(
    .a       (instrID[15:0]),
    .signext (controlID[0]),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimmEX[31:0]),
    .shiftl16  (shiftl16EX),
    .y         (shiftedimm[31:0]));

  // ALU logic

  wire [31:0] alu_a, alu_b;
  
  mux2 #(32) srcbmux(
    .d0 (srcb),
    .d1 (shiftedimm[31:0]),
    .s  (alusrcEX),
    .y  (alu_b));


  mux4 #(32) src1(
    .d0   (rd1EX),
    .d1   (regdata),
    .d2   (memaddr),
    .d3   (memaddr),
    .s    (muxcontrol[1:0]),
    .y    (alu_a)
    );

  mux4 #(32) src2(
    .d0   (rd2EX),
    .d1   (regdata),
    .d2   (memaddr),
    .d3   (memaddr),
    .s    (muxcontrol[3:2]),
    .y    (srcb)
    );

  alu alu(
    .a       (alu_a),
    .b       (alu_b),
    .alucont (alucontrol),
    .result  (memaddrEX),
    .zero    (zero));




endmodule



module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump, jal,
               output       notequal,
               output       memread,
               output [1:0] aluop);

  reg [13:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, jal, aluop, notequal, memread} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 14'b00110000001100; // Rtype
      6'b100011: controls <= #`mydelay 14'b10101001000001; // LW
      6'b101011: controls <= #`mydelay 14'b10001010000000; // SW
      6'b000100: controls <= #`mydelay 14'b10000100000100; // BEQ
      6'b000101: controls <= #`mydelay 14'b10000100000110; // BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 14'b10101000000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 14'b00101000001000; // ORI
      6'b001111: controls <= #`mydelay 14'b01101000000001; // LUI
      6'b000010: controls <= #`mydelay 14'b00000000100000; // J
      6'b000011: controls <= #`mydelay 14'b00100000110000; // JAL
      default:   controls <= #`mydelay 14'bxxxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg [3:0] alucontrol,
              output reg [0:0] jr         );

  always @(*)
  begin
  jr <= #`mydelay 1'b0;
    case(aluop)
      2'b00: alucontrol <= #`mydelay 4'b0100;  // add
      2'b01: alucontrol <= #`mydelay 4'b1100;  // sub
      2'b10: alucontrol <= #`mydelay 4'b0010;  // or
      default: case(funct)          // RTYPE
          6'b100000,                                 
          6'b100001: alucontrol <= #`mydelay 4'b0100; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 4'b1100; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
          6'b100101: alucontrol <= #`mydelay 4'b0010; // OR
          6'b101010: alucontrol <= #`mydelay 4'b1110; // SLT
          // Start! - Add SLTU
          6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU
          // End!
          6'b001000: begin alucontrol <= #`mydelay 4'b1001; jr <= #`mydelay 1'b1; end
          default:   alucontrol <= #`mydelay 4'bxxxx; // ???
        endcase
    endcase
  end
endmodule



// ###### Lee GeonWoo : Start #######
// "DDUDDA(뚜따)"ed the cover(datapath, controller) 

// In mips
  /*
  // Instantiate Controller
  controller c(
    .op         (instr[31:26]), 
    .funct      (instr[5:0]), 
    .zero       (zero),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .memwrite   (memwrite),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .jal        (jal),
    .jr         (jr),
    .alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .jal        (jal),
    .jr         (jr),
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata));
    */

  //Controller

  /*
  module controller(input  [5:0] op, funct,
                    input        zero,
                    output       signext,
                    output       shiftl16,
                    output       memtoreg, memwrite,
                    output       pcsrc, alusrc,
                    output       regdst, regwrite,
                    output       jump, jal, jr,
                    output [3:0] alucontrol);

  wire [1:0] aluop;
  wire       branch;
  // Start! - Add notequal wire
  wire       notequal;
  // End !

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .jal      (jal),
    .aluop    (aluop),
    .notequal (notequal));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol),
    .jr         (jr));

  assign pcsrc = branch & (zero ^ notequal);

  endmodule
  */

  // Datapath

  /*
  module datapath(input         clk, reset,
                  input         signext,
                  input         shiftl16,
                  input         memtoreg, pcsrc,
                  input         alusrc, regdst,
                  input         regwrite, jump, jal, jr,
                  input  [3:0]  alucontrol,
                  output        zero,
                  output [31:0] pc,
                  input  [31:0] instr,
                  output [31:0] aluout, writedata,
                  input  [31:0] readdata);

  wire [4:0]  writereg;
  wire [31:0] pcplus4, pcnextbr, pcbranch, pcnextbrj, pcnextfinal; 
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
  wire        shift;

  wire [4:0] regaddr;
  wire [31:0] regdata;

  adder pcadd4(
    .a (pc),
    .b (32'h00000004),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm),
    .y (signimmsh));
         
  adder pcaddbr(
    .a (pcplus4),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcjumpmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4[31:28], instr[25:0], 2'b00}),
    .s    (jump),
    .y    (pcnextbrj));

  mux2 #(32) pcjrmux(
    .d0   (pcnextbrj),
    .d1   (srca),
    .s    (jr),
    .y    (pcnextfinal));

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (pcnextfinal),
    .q     (pc));


  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (regaddr),
    .wd      (regdata),
    .rd1     (srca),
    .rd2     (writedata));

  mux2 #(5) wrmux(
    .d0  (instr[20:16]),
    .d1  (instr[15:11]),
    .s   (regdst),
    .y   (writereg));

  mux2 #(5) wrjalmux(
    .d0  (writereg),
    .d1  (5'b11111),
    .s   (jal),
    .y   (regaddr));

  mux2 #(32) resmux(
    .d0 (aluout),
    .d1 (readdata),
    .s  (memtoreg),
    .y  (result));

  mux2 #(32) resjalmux(
    .d0 (result),
    .d1 (pcplus4),
    .s  (jal),
    .y  (regdata));

  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata),
    .d1 (shiftedimm[31:0]),
    .s  (alusrc),
    .y  (srcb));

  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout),
    .zero    (zero));
    
  endmodule
  */

// ###### Lee GeonWoo : End #######