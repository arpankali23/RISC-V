//RISC_V 5-stages Pipeline Implementation
//Program counter
module PC_counter(input [31:0] clk, reset,
		  input [31:0] PC_next,
		  output reg [31:0] PC);

always@(posedge clk)
    begin
	if(reset == 1'b0)
	  PC <= {32{1'b0}};
	else
	  PC <= PC_next;

    end

endmodule

//PC_ADDER
module PC_Adder(input [31:0] a,
	 	input [31:0] b,
		output [31:0]c);

assign c = a + b;

endmodule

//Instruction Memory
module Inst_Mem(reset,A,RD);

input reset;
input [31:0] A;
output [32:0] RD;

//Creation of a memory
reg [31:0] Mem [1023:0];

assign RD = (reset == 1'b0) ? 32'h0000_0000 : Mem[A[31:2]];

endmodule

//MUX2x1
module MUX(a,b,sel,c);

input [31:0] a, b;
input sel;
output [31:0] c;

assign c = (sel == 1'b0) ? a : b;
endmodule


//Implementation of fetch
/*`include "MUX.v"
`include "PC.v"
`include "PC_Adder.v"
`include "Instruction_Mem.v"*/

module fetch_cycle(clk, reset, PCSrcE, PCTargetE, InstrD, PCD, PCPlus4D);

//Input Output port declaration
input clk, reset;
input PCSrcE;
input [31:0] PCTargetE;
output [31:0] InstrD, PCD, PCPlus4D;

//Internal Wires
wire [31:0] PCPlus4F;
wire [31:0] PC_F, PCF, InstrF;

//Register declaration
reg [31:0] PCPlus4F_reg;
reg [31:0] PCF_reg, InstrF_reg;

//Declaration of Instances
//MUX module
MUX mux(.a(PCPlus4F),
	.b(PCTargetE),
	.sel(PCSrcE),
	.c(PC_F));

//Program Counter Module
PC_counter pc(.clk(clk),
	      .reset(reset),
	      .PC_next(PC_F),
	      .PC(PCF));

//Adder Module
PC_Adder pc_adder(.a(PCF),
		  .b(32'h0000_0004),
		  .c(PCPlus4F));

//Instruction Memory Module
Inst_Mem inst_mem(.reset(reset),
		  .A(PCF),
		  .RD(InstrF));

// Fetch Cycle Register Logic
always @(posedge clk or negedge reset) 
     begin
	if(!reset)
	begin
	  InstrF_reg <= 32'h0000_0000;
	  PCF_reg <= 32'h0000_0000;
	  PCPlus4F_reg <= 32'h0000_0000;
	end
	else
	begin
	  InstrF_reg <= InstrF;
	  PCF_reg <= PCF;
	  PCPlus4F_reg <= PCPlus4F;
	end
      end

//Registers Output port logic
assign  InstrD = (!reset) ? 32'h0000_0000 : InstrF_reg;
assign  PCD = (!reset) ? 32'h0000_0000 : PCF_reg;
assign  PCPlus4D = (!reset) ? 32'h0000_0000 : PCPlus4F_reg;


endmodule
//ALU Decoder
module ALU_Decoder(ALUop,funct3,funct7,OP,ALUControl);

    input [1:0]ALUop;
    input [2:0]funct3;
    input [6:0]funct7,OP;
    output [2:0]ALUControl;

 assign ALUControl = (ALUop == 2'b00) ? 3'b000 :  //add (lw,sw)
                        (ALUop == 2'b01) ? 3'b001 :  //sub (beq)
                        ((ALUop == 2'b10) & (funct3 == 3'b000) & ({OP[5],funct7[5]} == 2'b11)) ? 3'b001 : //sub (sub)
                        ((ALUop == 2'b10) & (funct3 == 3'b000) & ({OP[5],funct7[5]} != 2'b11)) ? 3'b000 : //add (add)
                        ((ALUop == 2'b10) & (funct3 == 3'b010)) ? 3'b101 :  //set less then (slt)
                        ((ALUop == 2'b10) & (funct3 == 3'b110)) ? 3'b011 :  //or (or)
                        ((ALUop == 2'b10) & (funct3 == 3'b111)) ? 3'b010 :  //and (and) 
                                                                  3'b000 ;
endmodule


//Main Decoder
module main_decoder(OP, RegWrite, ImmSrc, ALUSrc, MemWrite, ResultSrc, Branch, ALUop, PCSrc);

//input zero;
input [6:0] OP;
output RegWrite, ALUSrc, MemWrite, ResultSrc, Branch, PCSrc;
output [1:0] ImmSrc, ALUop;

assign RegWrite = (OP==7'b0000011 || OP==7'b0110011) ? 1'b1 : 1'b0;
assign ALUSrc = (OP==7'b0000011 || OP==7'b0100011 ) ? 1'b1 : 1'b0;
assign ImmSrc = (OP==7'b0100011) ? 2'b01 : (OP==7'b1100011) ? 2'b10 : 2'b00;
assign MemWrite = (OP==7'b0100011) ? 1'b1 : 1'b0;
assign ResultSrc = (OP==7'b0000011) ? 1'b1 : 1'b0;
assign Branch = (OP==7'b1100011) ? 1'b1 : 1'b0;
assign ALUop  = (OP==7'b0110011) ? 2'b10 : (OP==7'b1100011) ? 2'b01 : 2'b00;
//assign PCSrc = zero & Branch;


endmodule

//Control Unit
module Control_Unit_Top(OP,RegWrite,ImmSrc,ALUSrc,MemWrite,ResultSrc,Branch,funct3,funct7,ALUControl);

input [6:0]OP,funct7;
input [2:0]funct3;
output RegWrite,ALUSrc,MemWrite,ResultSrc,Branch;
output [1:0]ImmSrc;
output [2:0]ALUControl;

wire [1:0]ALUop;

main_decoder Main_Decoder(
                .OP(OP),
                .RegWrite(RegWrite),
                .ImmSrc(ImmSrc),
                .MemWrite(MemWrite),
                .ResultSrc(ResultSrc),
                .Branch(Branch),
                .ALUSrc(ALUSrc),
                .ALUop(ALUop));

ALU_Decoder ALU_Decoder(.ALUop(ALUop),
                        .funct3(funct3),
                        .funct7(funct7),
                        .OP(OP),
                        .ALUControl(ALUControl));


endmodule

//Register file
module Reg_file(clk, reset, A1, A2, A3, WD3, WE3, RD1, RD2);

input clk, reset, WE3;
input [4:0] A1, A2, A3;
input [31:0] WD3;
output [31:0] RD1, RD2;

//creating a memory
reg [31:0] Reg [31:0];

//Read operation
assign RD1 = (!reset) ? 32'h0000_0000 : Reg[A1];
assign RD2 = (!reset) ? 32'h0000_0000 : Reg[A2];

always@(posedge clk)
     begin
	if(WE3 && (A3 != 5'h00))
	  Reg[A3] <= WD3;
     end


endmodule


//Sign Extentaion
module Sign_Extend(In,ImmSrc,Imm_Ext);
    input [31:0] In;
    input [1:0] ImmSrc;
    output [31:0] Imm_Ext;

    assign Imm_Ext =  (ImmSrc == 2'b00) ? {{20{In[31]}},In[31:20]} : 
                     (ImmSrc == 2'b01) ? {{20{In[31]}},In[31:25],In[11:7]} : 32'h00000000; 

endmodule



//DECODER CYCLE

module Decoder_Cycle (clk,reset,InstrD,PCD,PCPlus4D,RegWriteW,RDW,ResultW,RegWriteE,ResultSrcE,MemWriteE,BranchE,ALUSrcE,
			ALUControlE,RD1E,RD2E,ImmExtE,PCE,PCPlus4E,RDE);

//Declaration of I/O
input clk,reset,RegWriteW;
input [31:0] InstrD,PCD,PCPlus4D,ResultW;
input [4:0] RDW;

output RegWriteE,ResultSrcE,MemWriteE,BranchE,ALUSrcE;
output [2:0] ALUControlE;
output [31:0] RD1E,RD2E,ImmExtE;
output [31:0] PCE,PCPlus4E;
output [4:0] RDE;


//Declaration of Internal wires
wire RegWriteD,ALUSrcD,MemWriteD,ResultSrcD,BranchD;
wire [1:0] ImmSrcD;
wire [2:0] ALUControlD;
wire [31:0] RD1D,RD2D,ImmExtD;

// Declaration of Internal Register
reg RegWriteD_reg, ALUSrcD_reg, MemWriteD_reg, ResultSrcD_reg, BranchD_reg;
reg [2:0] ALUControlD_reg;
reg [31:0] RD1D_reg, RD2D_reg, ImmExtD_reg;
reg [4:0] RDD_reg, RS1D_reg, RS2D_reg;
reg [31:0] PCD_reg, PCPlus4D_reg;

//Instanciate the modules
Control_Unit_Top CUT(.OP(InstrD[6:0]),
			.RegWrite(RegWriteD),
			.ImmSrc(ImmSrcD),
			.ALUSrc(ALUSrcD),
			.MemWrite(MemWriteD),
			.ResultSrc(ResultSrcD),
			.Branch(BranchD),
			.funct3(InstrD[14:12]),
			.funct7(InstrD[31:25]),
			.ALUControl(ALUControlD));

Reg_file REGF(.clk(clk),
		.reset(reset),
		.A1(InstrD[19:15]),
		.A2(InstrD[24:20]),
		.A3(RDW),
		.WD3(ResultW),
		.WE3(RegWriteW),
		.RD1(RD1D),
		.RD2(RD2D));

Sign_Extend SIGNEX(.In(InstrD[31:0]),
		   .ImmSrc(ImmSrcD),
		   .Imm_Ext(ImmExtD));

always@(posedge clk or negedge reset)
    begin
	if(!reset)
	  begin
	    RegWriteD_reg <= 1'b0;
            ALUSrcD_reg <= 1'b0;
            MemWriteD_reg <= 1'b0;
            ResultSrcD_reg <= 1'b0;
            BranchD_reg <= 1'b0;
            ALUControlD_reg <= 3'b000;
            RD1D_reg <= 32'h00000000; 
            RD2D_reg <= 32'h00000000; 
            ImmExtD_reg <= 32'h00000000;
            RDD_reg <= 5'h00;
            PCD_reg <= 32'h00000000; 
            PCPlus4D_reg <= 32'h00000000;
            //RS1D_reg <= 5'h00;
            //RS2D_reg <= 5'h00;
	  end
    
	else
	  begin
            RegWriteD_reg <= RegWriteD;
            ALUSrcD_reg <= ALUSrcD;
            MemWriteD_reg <= MemWriteD;
            ResultSrcD_reg <= ResultSrcD;
            BranchD_reg <= BranchD;
            ALUControlD_reg <= ALUControlD;
            RD1D_reg <= RD1D; 
            RD2D_reg <= RD2D; 
            ImmExtD_reg <= ImmExtD;
            RDD_reg <= InstrD[11:7];
            PCD_reg <= PCD; 
            PCPlus4D_reg <= PCPlus4D;
            //RS1D_reg <= InstrD[19:15];
            //RS2D_reg <= InstrD[24:20];
         end
    end

// Output asssign statements
assign RegWriteE = RegWriteD_reg;
assign ALUSrcE = ALUSrcD_reg;
assign MemWriteE = MemWriteD_reg;
assign ResultSrcE = ResultSrcD_reg;
assign BranchE = BranchD_reg;
assign ALUControlE = ALUControlD_reg;
assign RD1E = RD1D_reg;
assign RD2E = RD2D_reg;
assign ImmExtE = ImmExtD_reg;
assign RDE = RDD_reg;
assign PCE = PCD_reg;
assign PCPlus4E = PCPlus4D_reg;
//assign RS1E = RS1D_reg;
//assign RS2E = RS2D_reg;

endmodule

//ALU
module ALU(A,B,Result,ALUControl,OverFlow,Carry,Zero,Negative);

    input [31:0]A,B;
    input [2:0]ALUControl;
    output Carry,OverFlow,Zero,Negative;
    output [31:0]Result;

    wire Cout;
    wire [31:0]Sum;

    assign Sum = (ALUControl[0] == 1'b0) ? A + B :
                                          (A + ((~B)+1)) ;
    assign {Cout,Result} = (ALUControl == 3'b000) ? Sum :
                           (ALUControl == 3'b001) ? Sum :
                           (ALUControl == 3'b010) ? A & B :
                           (ALUControl == 3'b011) ? A | B :
                           (ALUControl == 3'b101) ? {{32{1'b0}},(Sum[31])} :
                           {33{1'b0}};
    assign OverFlow = ((Sum[31] ^ A[31]) & 
                      (~(ALUControl[0] ^ B[31] ^ A[31])) &
                      (~ALUControl[1]));
    assign Carry = ((~ALUControl[1]) & Cout);
    assign Zero = &(~Result);
    assign Negative = Result[31];

endmodule


//EXECUTION CYCLE

module Execute_cycle(clk,reset,RegWriteE,ResultSrcE,MemWriteE,BranchE,ALUSrcE,ALUControlE,RD1E,RD2E,ImmExtE,PCE,PCPlus4E,RDE,
       PCTargetE,PCSrcE,RegWriteM, ResultSrcM, MemWriteM,ALUResultM,WriteDataM,PCPlus4M, RDM);

//Declarartion of input and output
input clk, reset;
input RegWriteE,ResultSrcE,MemWriteE,BranchE,ALUSrcE;
input [2:0] ALUControlE;
input [31:0] RD1E,RD2E,ImmExtE;
input [31:0] PCE,PCPlus4E;
input [4:0] RDE;

output PCSrcE;
output [31:0] PCTargetE;
output RegWriteM, ResultSrcM, MemWriteM;
output [4:0] RDM;
output [31:0] ALUResultM, WriteDataM, PCPlus4M;
 

//Declaration of internal wires
wire [31:0] SrcBE;
wire [31:0] ResultE;
wire ZeroE;


//Internal registers
reg RegWriteE_reg;
reg ResultSrcE_reg, MemWriteE_reg;
reg [4:0] RDE_reg;
reg [31:0] RD2E_reg, PCPlus4E_reg, ResultE_reg;

//Module Instances
ALU alu1(.A(RD1E),
	.B(SrcBE),
	.Result(ResultE),
	.ALUControl(ALUControlE),
	.OverFlow(),
	.Carry(),
	.Zero(ZeroE),
	.Negative());



/*ALU alu1(.A(RD1E),
        .B(SrcBE),
	.ALUControl(ALUControlE),
	.Result(ResultE),
	.Zero_flag(ZeroE),
	.Neg_flag(),
	.Carry_flag(),
	.Ovf_flag());*/

PC_Adder branch_add(.a(PCE),
	 .b(ImmExtE),
	 .c(PCTargetE));

MUX alu_mux(.a(RD2E),
	 .b(ImmExtE),
	 .sel(ALUSrcE),
	 .c(SrcBE));


//Register logic
always@(posedge clk or negedge reset)
     begin
	if(!reset)
	  begin
	    RegWriteE_reg <= 1'b0;
	    ResultSrcE_reg <= 1'b0;
	    MemWriteE_reg <= 1'b0;
	    RD2E_reg <= 32'h0000_0000;
	    ResultE_reg <= 32'h0000_0000;
	    RDE_reg <= 5'h00;
	    PCPlus4E_reg <= 32'h0000_0000;
	  end
	else
	  begin
	    RegWriteE_reg <= RegWriteE;
	    ResultSrcE_reg <= ResultSrcE;
	    MemWriteE_reg <= MemWriteE;
	    RD2E_reg <= RD2E;
	    ResultE_reg <= ResultE;
	    RDE_reg <= RDE;
	    PCPlus4E_reg <= PCPlus4E;
	  end
     end

//Output Declaration
assign PCSrcE = ZeroE & BranchE;

assign RegWriteM = RegWriteE_reg;
assign ResultSrcM = ResultSrcE_reg;
assign MemWriteM = MemWriteE_reg;
assign RDM = RDE_reg;
assign PCPlus4M = PCPlus4E_reg;
assign ALUResultM = ResultE_reg;
assign WriteDataM = RD2E_reg;


endmodule


//datamemory
module data_memory(clk,reset, A, WD, RD, WE);

input clk,reset;
input [31:0] A, WD;
input WE;
output [31:0] RD;

//MEMORY CREATION
reg [31:0] Data_Mem [1023:0];

//READ OP
assign RD = (WE==1'b0) ? Data_Mem[A] : 32'h0000_0000;

//WRITE OP
always@(posedge clk)
   begin
      if(WE)
	Data_Mem[A] <= WD;
   end


endmodule

//MEMORY CYCLE

module Memory_cycle(clk,reset,RegWriteM, ResultSrcM, MemWriteM,ALUResultM, WriteDataM, PCPlus4M,RDM,
		    RegWriteW, ResultSrcW,ALUResultW, ReadDataW,RDW,PCPlus4W);
                    
//Declaration of input and output ports
input clk,reset;
input RegWriteM, ResultSrcM, MemWriteM;
input [31:0] ALUResultM, WriteDataM, PCPlus4M;
input [4:0] RDM;

output RegWriteW, ResultSrcW;
output [31:0] ALUResultW, ReadDataW;
output [4:0] RDW;
output [31:0] PCPlus4W;


//Declaration of intrenal wires
wire [31:0] ReadDataM;


//Declaration of internal register
reg RegWriteM_reg,ResultSrcM_reg;
reg [4:0] RDM_reg;
reg [31:0] PCPlus4M_reg, ALUResultM_reg, ReadDataM_reg;

//Module Instances
data_memory DM(.clk(clk),
	       .reset(reset),
	       .A(ALUResultM),
	       .WD(WriteDataM),
	       .RD(ReadDataM),
	       .WE(MemWriteM));

//Register Logic
always@(posedge clk)
     begin
	if(~reset)
	begin
	RegWriteM_reg <= 1'b0;
	ResultSrcM_reg <= 1'b0;
	RDM_reg <= 5'h00;
	PCPlus4M_reg <= 32'h0000_0000;
	ALUResultM_reg <= 32'h0000_0000;
	ReadDataM_reg <= 32'h0000_0000;
	end

	else
	begin
	RegWriteM_reg <= RegWriteM;
	ResultSrcM_reg <= ResultSrcM;
	RDM_reg <= RDM;
	PCPlus4M_reg <= PCPlus4M;
	ALUResultM_reg <= ALUResultM;
	ReadDataM_reg <= ReadDataM;
	end
     end

//Output Assignments
assign RegWriteW = RegWriteM_reg;
assign ResultSrcW = ResultSrcM_reg;
assign RDW = RDM_reg;
assign PCPlus4W = PCPlus4M_reg;
assign ALUResultW = ALUResultM_reg;
assign ReadDataW = ReadDataM_reg;

endmodule

//WRITE BACK CYCLE
module writeback_cycle(clk, reset, ResultSrcW, ALUResultW, ReadDataW, PCPlus4W, ResultW);

input clk, reset;
input ResultSrcW;
input [31:0] ALUResultW, ReadDataW, PCPlus4W;
output [31:0] ResultW;

MUX re_mux1(
    .a(ALUResultW),
    .b(ReadDataW),
    .sel(ResultSrcW),
    .c(ResultW)
);

endmodule


//RISC_PIPELINE_TOP
module pipeline_top(clk, reset);

//Declaration of I/Os
input clk, reset;

//Declaration of internal wires
wire PCSrcE, RegWriteW, RegWriteE, ALUSrcE, ResultSrcE, MemWriteE, BranchE, RegWriteM;
wire ResultSrcM, MemWriteM, ResultSrcW;
wire [2:0] ALUControlE;
wire [4:0] RDW, RDE, RDM;
wire [31:0] PCTargetE, InstrD, PCD, PCPlus4D, ResultW, RD1E, RD2E, ImmExtE, PCE, PCPlus4E;
wire [31:0] PCPlus4M, WriteDataM, ALUResultM, PCPlus4W, ALUResultW, ReadDataW;


//Module Instances
fetch_cycle FEC(.clk(clk),
		.reset(reset),
		.PCSrcE(PCSrcE),
		.PCTargetE(PCTargetE),
		.InstrD(InstrD),
		.PCD(PCD),
		.PCPlus4D(PCPlus4D));

Decoder_Cycle DEC(.clk(clk),
		.reset(reset),
		.InstrD(InstrD),
		.PCD(PCD),
		.PCPlus4D(PCPlus4D),
		.RegWriteW(RegWriteW),
		.RDW(RDW),
		.ResultW(ResultW),
		.RegWriteE(RegWriteE),
		.ResultSrcE(ResultSrcE),
		.MemWriteE(MemWriteE),
		.BranchE(BranchE),
		.ALUSrcE(ALUSrcE),
		.ALUControlE(ALUControlE),
		.RD1E(RD1E),
		.RD2E(RD2E),
		.ImmExtE(ImmExtE),
		.PCE(PCE),
		.PCPlus4E(PCPlus4E),
		.RDE(RDE));

Execute_cycle EXEC(.clk(clk),
		.reset(reset),
		.RegWriteE(RegWriteE),
		.ResultSrcE(ResultSrcE),
		.MemWriteE(MemWriteE),
		.BranchE(BranchE),
		.ALUSrcE(ALUSrcE),
		.ALUControlE(ALUControlE),
		.RD1E(RD1E),
		.RD2E(RD2E),
		.ImmExtE(ImmExtE),
		.PCE(PCE),
		.PCPlus4E(PCPlus4E),
		.RDE(RDE),
		.PCTargetE(PCTargetE),
		.PCSrcE(PCSrcE),
		.RegWriteM(RegWriteM),
		.ResultSrcM(ResultSrcM),
		.MemWriteM(MemWriteM),
		.ALUResultM(ALUResultM),
		.WriteDataM(WriteDataM),
		.PCPlus4M(PCPlus4M),
		.RDM(RDM));

Memory_cycle MEMC(.clk(clk),
		.reset(reset),
		.RegWriteM(RegWriteM),
		.ResultSrcM(ResultSrcM),
		.MemWriteM(MemWriteM),
		.ALUResultM(ALUResultM),
		.WriteDataM(WriteDataM),
		.PCPlus4M(PCPlus4M),
		.RDM(RDM),
		.RegWriteW(RegWriteW),
		.ResultSrcW(ResultSrcW),
		.ALUResultW(ALUResultW),
		.ReadDataW(ReadDataW),
		.RDW(RDW),
		.PCPlus4W(PCPlus4W));

writeback_cycle WBC(.clk(clk),
		.reset(reset),
		.ResultSrcW(ResultSrcW),
		.ALUResultW(ALUResultW),
		.ReadDataW(ReadDataW),
		.PCPlus4W(PCPlus4W),
		.ResultW(ResultW));





endmodule
