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
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite; 
  wire        jump, jal, jr;
  wire [3:0]  alucontrol;

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

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump, jal, jr, // Start! jal, jr added - End!
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


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump, jal,
               output       notequal,
               output [1:0] aluop);

  reg [12:0] controls; // Start! - Add notequal and jal - End!

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, jal, aluop, notequal} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 13'b0011000000110; // Rtype
      6'b100011: controls <= #`mydelay 13'b1010100100000; // LW
      6'b101011: controls <= #`mydelay 13'b1000101000000; // SW
      6'b000100: controls <= #`mydelay 13'b1000010000010; // BEQ
      6'b000101: controls <= #`mydelay 13'b1000010000011; // BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 13'b1010100000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 13'b0010100000100; // ORI
      6'b001111: controls <= #`mydelay 13'b0110100000000; // LUI
      6'b000010: controls <= #`mydelay 13'b0000000010000; // J
      6'b000011: controls <= #`mydelay 13'b0010000011000; // JAL
      default:   controls <= #`mydelay 13'bxxxxxxxxxxxxx; // ???
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
  wire [31:0] pcplus4, pcnextbr, pcbranch, pcnextbrj, pcnextfinal; // Start! - For JR - End!
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
  wire        shift;

  // Start! - Add wire for JAL
  wire [4:0] regaddr;
  wire [31:0] regdata;
  // End!

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
