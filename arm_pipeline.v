module adder (
	a,
	b,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] a;
	input wire [WIDTH - 1:0] b;
	output wire [WIDTH - 1:0] y;
	assign y = a + b;
endmodule

module alu (
    input wire [31:0] SrcA,
    input wire [31:0] SrcB,
    input wire [31:0] SrcC, 
    input wire [3:0] ALUControl,
    input wire CarryIn,  
    output reg [31:0] ALUResult,
    output  [3:0] ALUFlags
);
    wire [32:0] sum;  

    always @(*) begin
        case (ALUControl)
            4'b0000: ALUResult = SrcA + SrcB;       // ADD
            4'b0001: ALUResult = SrcA - SrcB;       // SUB
            4'b0010: ALUResult = SrcB;              // MOV
            4'b0011: ALUResult = SrcA & SrcB;       // AND
            4'b0100: ALUResult = SrcA | SrcB;       // ORR
            4'b0101: ALUResult = SrcC - (SrcA * SrcB); // MLS
            4'b0110: ALUResult = SrcA * SrcB;       // MUL
            4'b0111: ALUResult = SrcC + (SrcA * SrcB); // MLA
            4'b1000: ALUResult = SrcA - SrcB - ~CarryIn; // SBC
            4'b1001: ALUResult = SrcB - SrcA;           // RSB
            4'b1010: ALUResult = SrcA + SrcB + CarryIn; //ADC
            default: ALUResult = 32'b0;
        endcase
    end

    assign sum = SrcA + (ALUControl[1] ? ~SrcB + 1 : SrcB); 

    assign neg = ALUResult[31];
    assign zero = (ALUResult == 32'b0);
    assign carry = (ALUControl[1] == 1'b0) & sum[32];
    assign overflow = (ALUControl[1] == 1'b0) & ~(SrcA[31] ^ SrcB[31] ^ ALUControl[0]) & (SrcA[31] ^ sum[31]);
    assign ALUFlags = {neg, zero, carry, overflow};
endmodule


module arm (
	clk,
	reset,
	PC,
	Instr,
	MemWrite,
	ALUResult,
	WriteData,
	ReadData
);
	input wire clk;
	input wire reset;
	output wire [31:0] PC;
	input wire [31:0] Instr;
	output wire MemWrite;
	output wire [31:0] ALUResult;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
	wire [3:0] ALUFlags;
	wire RegWrite;
	wire ALUSrc;
	wire MemtoReg;
	wire PCSrc;
	wire [1:0] RegSrc;
	wire [1:0] ImmSrc;
	wire [1:0] ALUControl;
	controller c(
		.clk(clk),
		.reset(reset),
		.Instr(Instr[31:12]),
		.ALUFlags(ALUFlags),
		.RegSrc(RegSrc),
		.RegWrite(RegWrite),
		.ImmSrc(ImmSrc),
		.ALUSrc(ALUSrc),
		.ALUControl(ALUControl),
		.MemWrite(MemWrite),
		.MemtoReg(MemtoReg),
		.PCSrc(PCSrc)
	);
	datapath dp(
		.clk(clk),
		.reset(reset),
		.RegSrc(RegSrc),
		.RegWrite(RegWrite),
		.ImmSrc(ImmSrc),
		.ALUSrc(ALUSrc),
		.ALUControl(ALUControl),
		.MemtoReg(MemtoReg),
		.PCSrc(PCSrc),
		.ALUFlags(ALUFlags),
		.PC(PC),
		.Instr(Instr),
		.ALUResult(ALUResult),
		.WriteData(WriteData),
		.ReadData(ReadData)
	);
endmodule
module condcheck (
	Cond,
	Flags,
	CondEx
);
	input wire [3:0] Cond;
	input wire [3:0] Flags;
	output reg CondEx;
	wire neg;
	wire zero;
	wire carry;
	wire overflow;
	wire ge;
	assign {neg, zero, carry, overflow} = Flags;
	assign ge = neg == overflow;
	always @(*)
		case (Cond)
			4'b0000: CondEx = zero;
			4'b0001: CondEx = ~zero;
			4'b0010: CondEx = carry;
			4'b0011: CondEx = ~carry;
			4'b0100: CondEx = neg;
			4'b0101: CondEx = ~neg;
			4'b0110: CondEx = overflow;
			4'b0111: CondEx = ~overflow;
			4'b1000: CondEx = carry & ~zero;
			4'b1001: CondEx = ~(carry & ~zero);
			4'b1010: CondEx = ge;
			4'b1011: CondEx = ~ge;
			4'b1100: CondEx = ~zero & ge;
			4'b1101: CondEx = ~(~zero & ge);
			4'b1110: CondEx = 1'b1;
			default: CondEx = 1'bx;
		endcase
endmodule

module condlogic (
	clk,
	reset,
	Cond,
	ALUFlags,
	FlagW,
	PCS,
	RegW,
	MemW,
	PCSrc,
	RegWrite,
	MemWrite
);
	input wire clk;
	input wire reset;
	input wire [3:0] Cond;
	input wire [3:0] ALUFlags;
	input wire [1:0] FlagW;
	input wire PCS;
	input wire RegW;
	input wire MemW;
	output wire PCSrc;
	output wire RegWrite;
	output wire MemWrite;
	wire [1:0] FlagWrite;
	wire [3:0] Flags;
	wire CondEx;
	flopenr #(2) flagreg1(
		.clk(clk),
		.reset(reset),
		.en(FlagWrite[1]),
		.d(ALUFlags[3:2]),
		.q(Flags[3:2])
	);
	flopenr #(2) flagreg0(
		.clk(clk),
		.reset(reset),
		.en(FlagWrite[0]),
		.d(ALUFlags[1:0]),
		.q(Flags[1:0])
	);
	condcheck cc(
		.Cond(Cond),
		.Flags(Flags),
		.CondEx(CondEx)
	);
	assign FlagWrite = FlagW & {2 {CondEx}};
	assign RegWrite = RegW & CondEx;
	assign MemWrite = MemW & CondEx;
	assign PCSrc = PCS & CondEx;
endmodule

module cond_unit(
	clk,
	reset,
	FlagWriteE,
	CondE,
	FlagsE,
	ALUFlags,
	Flags,
	CondExE,
);
	input wire clk;
	input wire reset;
	input wire [1:0] FlagWriteE;
	input wire [3:0] CondE;
	input wire [3:0] FlagsE;
	input wire [3:0] ALUFlags;	
	output wire [3:0] Flags;
	output wire CondExE;

	wire [1:0] FlagWE;

	flopenr #(2) fg1(
		.clk(clk),
		.reset(reset),
		.en(FlagWE[1]),
		.d(ALUFlags[3:2]),
		.q(Flags[3:2])
	);
	flopenr #(2) fg2(
		.clk(clk),
		.reset(reset),
		.en(FlagWE[0]),
		.d(ALUFlags[1:0]),
		.q(Flags[1:0])
	);
	condcheck cc(
		.Cond(CondE),
		.Flags(FlagsE),
		.CondEx(CondExE)
	);
	assign FlagWE = FlagWriteE & {2 {CondExE}};
	

endmodule

module controller (
	clk,
	reset,
	Instr,
	ALUFlags,
	RegSrc,
	RegWrite,
	ImmSrc,
	ALUSrc,
	ALUControl,
	MemWrite,
	MemtoReg,
	PCSrc,
	byte
);
	input wire clk;
	input wire reset;
	input wire [31:12] Instr;
	input wire [3:0] ALUFlags;
	output wire [1:0] RegSrc;
	output wire RegWrite;
	output wire [1:0] ImmSrc;
	output wire ALUSrc;
	output wire [1:0] ALUControl;
	output wire MemWrite;
	output wire MemtoReg;
	output wire PCSrc;
	output wire Byte;
	wire [1:0] FlagW;
	wire PCS;
	wire RegW;
	wire MemW;
	decode dec(
		.Op(Instr[27:26]),
		.Funct(Instr[25:20]),
		.Rd(Instr[15:12]),
		.FlagW(FlagW),
		.PCS(PCS),
		.RegW(RegW),
		.MemW(MemW),
		.MemtoReg(MemtoReg),
		.ALUSrc(ALUSrc),
		.ImmSrc(ImmSrc),
		.RegSrc(RegSrc),
		.ALUControl(ALUControl),
		.Btye(Byte)
	);
	condlogic cl(
		.clk(clk),
		.reset(reset),
		.Cond(Instr[31:28]),
		.ALUFlags(ALUFlags),
		.FlagW(FlagW),
		.PCS(PCS),
		.RegW(RegW),
		.MemW(MemW),
		.PCSrc(PCSrc),
		.RegWrite(RegWrite),
		.MemWrite(MemWrite)
	);
endmodule

module control_unit (
	Instr,
	PCSrcD,
	RegWriteD,
	MemtoRegD,
	MemWriteD,
	ALUControlD,
	BranchD,
	ALUSrcD,
	FlagWriteD,
	ImmSrcD,
	RegSrcD,
);
	input wire [27:12] Instr;
	output wire PCSrcD;
	output wire RegWriteD;
	output wire MemtoRegD;
	output wire MemWriteD;
	output wire [1:0] ALUControlD;
	output wire BranchD;
	output wire ALUSrcD;
	output wire [1:0] FlagWriteD;
	output wire [1:0] ImmSrcD;
	output wire [1:0] RegSrcD;

	decode dec(
		.Op(Instr[27:26]),
		.Funct(Instr[25:20]),
		.Rd(Instr[15:12]),
		.PCS(PCSrcD),
		.MemW(MemWriteD),
		.MemtoReg(MemtoRegD),
		.ALUSrc(ALUSrcD),
		.ImmSrc(ImmSrcD),
		.RegSrc(RegSrcD),
		.RegW(RegWriteD),
		.Branch(BranchD),
		.ALUControl(ALUControlD),
		.FlagW(FlagWriteD)
	);
endmodule

module datapath (
    clk,
    reset,
    RegSrc,
    RegWrite,
    ImmSrc,
    ALUSrc,
    ALUControl,
    MemtoReg,
    PCSrc,
    ALUFlags,
    PC,
    Instr,
    ALUResult,
    WriteData,
    ReadData
);
    input wire clk;
    input wire reset;
    input wire [1:0] RegSrc;
    input wire RegWrite;
    input wire [1:0] ImmSrc;
    input wire ALUSrc;
    input wire [2:0] ALUControl;
    input wire MemtoReg;
    input wire PCSrc;
    output wire [3:0] ALUFlags;
    output wire [31:0] PC;
    input wire [31:0] Instr;
    output wire [31:0] ALUResult;
    output wire [31:0] WriteData;
    input wire [31:0] ReadData;

    wire [31:0] PCNext;
    wire [31:0] PCPlus4;
    wire [31:0] PCPlus8;
    wire [31:0] ExtImm;
    wire [31:0] SrcA;
    wire [31:0] SrcB;
    wire [31:0] SrcC;          // Nuevo operando SrcC para MLA y MLS
    wire [31:0] Result;
    wire [3:0] RA1;
    wire [3:0] RA2;

    // CarryIn, necesario para operaciones SBC
    wire CarryIn;
    assign CarryIn = ALUFlags[2];  // Utiliza el bit de carry de la última operación

    // Calcular siguiente PC
    mux2 #(32) pcmux(
        .d0(PCPlus4),
        .d1(Result),
        .s(PCSrc),
        .y(PCNext)
    );

    flopr #(32) pcreg(
        .clk(clk),
        .reset(reset),
        .d(PCNext),
        .q(PC)
    );

    adder #(32) pcadd1(
        .a(PC),
        .b(32'b100),
        .y(PCPlus4)
    );

    adder #(32) pcadd2(
        .a(PCPlus4),
        .b(32'b100),
        .y(PCPlus8)
    );

    // Definir los registros de origen
    mux2 #(4) ra1mux(
        .d0(Instr[19:16]),
        .d1(4'b1111),
        .s(RegSrc[0]),
        .y(RA1)
    );

    mux2 #(4) ra2mux(
        .d0(Instr[3:0]),
        .d1(Instr[15:12]),
        .s(RegSrc[1]),
        .y(RA2)
    );

    regfile rf(
        .clk(clk),
        .we3(RegWrite),
        .ra1(RA1),
        .ra2(RA2),
        .wa3(Instr[15:12]),
        .wd3(Result),
        .r15(PCPlus8),
        .rd1(SrcA),
        .rd2(WriteData)
    );

    // Conectar SrcC como el tercer operando
    assign SrcC = PCPlus8;  // Ajuste para obtener el tercer operando de MLA/MLS si es necesario

    mux2 #(32) resmux(
        .d0(ALUResult),
        .d1(ReadData),
        .s(MemtoReg),
        .y(Result)
    );

    extend ext(
        .Instr(Instr[23:0]),
        .ImmSrc(ImmSrc),
        .ExtImm(ExtImm)
    );

    mux2 #(32) srcbmux(
        .d0(WriteData),
        .d1(ExtImm),
        .s(ALUSrc),
        .y(SrcB)
    );

    alu alu_inst(
        .SrcA(SrcA),
        .SrcB(SrcB),
        .SrcC(SrcC),            
        .ALUControl(ALUControl),
        .CarryIn(CarryIn),      
        .ALUResult(ALUResult),
        .ALUFlags(ALUFlags)
    );
endmodule

module decode (
    Op,
    Funct,
    Rd,
    FlagW,
    PCS,
    RegW,
    MemW,
    MemtoReg,
    ALUSrc,
    ImmSrc,
    RegSrc,
    ALUControl,
    Branch
);
    input wire [1:0] Op;
    input wire [5:0] Funct;
    input wire [3:0] Rd;
    output reg [1:0] FlagW;
    output wire PCS;
    output wire RegW;
    output wire MemW;
    output wire MemtoReg;
    output wire ALUSrc;
    output wire [1:0] ImmSrc;
    output wire [1:0] RegSrc;
    output wire Branch;
    output reg [2:0] ALUControl;
    reg [9:0] controls;
    wire Branch;
    wire ALUOp;

    always @(*)
        casex (Op)
            2'b00:
                if (Funct[5])
                    controls = 10'b0000101001;
                else
                    controls = 10'b0000001001;
            2'b01:
                if (Funct[0])
                    controls = 10'b0001111000;
                else
                    controls = 10'b1001110100;
            2'b10: controls = 10'b0110100010;
            default: controls = 10'bxxxxxxxxxx;
        endcase

    assign {RegSrc, ImmSrc, ALUSrc, MemtoReg, RegW, MemW, Branch, ALUOp} = controls;

    always @(*)
        if (ALUOp) begin
            case (Funct[4:1])
                4'b1101: ALUControl = 4'b0010; // MOV
                4'b0100: ALUControl = 4'b0000; // ADD
                4'b0010: ALUControl = 4'b0001; // SUB
                4'b0000: ALUControl = 4'b0011; // AND
                4'b1100: ALUControl = 4'b0100; // ORR
                4'b1001: ALUControl = 4'b0110; // MUL
                4'b1010: ALUControl = 4'b0111; // MLA
                4'b1011: ALUControl = 4'b0101; // MLS
                4'b0011: ALUControl = 4'b1000; // SBC
                4'b0111: ALUControl = 4'b1001; // RSB
                4'b0101: ALUControl = 4'b1010; // ADC
                default: ALUControl = 4'bxxxx;
            endcase
            FlagW[1] = Funct[0];
            FlagW[0] = Funct[0] & ((ALUControl == 3'b000) | (ALUControl == 3'b001) | (ALUControl == 3'b010));
        end
        else begin
            ALUControl = 3'b000;
            FlagW = 2'b00;
        end

    assign PCS = ((Rd == 4'b1111) & RegW) | Branch;
endmodule

module dmem (
	clk,
	we,
	a,
	wd,
	rd,
);
	input wire clk;
	input wire we;
	input wire [31:0] a;
	input wire [31:0] wd;
	output wire [31:0] rd;
	reg [31:0] RAM [63:0];
	assign rd = RAM[a[31:2]];
	always @(posedge clk)
		if (we)
			RAM[a[31:2]] <= wd;
endmodule

module extend (
	Instr,
	ImmSrc,
	ExtImm
);
	input wire [23:0] Instr;
	input wire [1:0] ImmSrc;
	output reg [31:0] ExtImm;
	always @(*)
		case (ImmSrc)
			2'b00: ExtImm = {24'b000000000000000000000000, Instr[7:0]};
			2'b01: ExtImm = {20'b00000000000000000000, Instr[11:0]};
			2'b10: ExtImm = {{6 {Instr[23]}}, Instr[23:0], 2'b00};
			default: ExtImm = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
		endcase
endmodule

module flopenr (
	clk,
	reset,
	en,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire en;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else if (en)
			q <= d;
endmodule

module flopr (
	clk,
	reset,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else
			q <= d;
endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/20/2024 05:09:39 PM
// Design Name: 
// Module Name: hazardunit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module hazardunit(

    );
endmodule

module imem (
	a,
	rd
);
	input wire [31:0] a;
	output wire [31:0] rd;
	reg [31:0] RAM [63:0];
	initial $readmemh("memfile.asm", RAM);
	assign rd = RAM[a[31:2]];
endmodule

module mux2 (
	d0,
	d1,
	s,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] d0;
	input wire [WIDTH - 1:0] d1;
	input wire s;
	output wire [WIDTH - 1:0] y;
	assign y = (s ? d1 : d0);
endmodule

module regfile (
	clk,
	we3,
	ra1,
	ra2,
	wa3,
	wd3,
	r15,
	rd1,
	rd2
);
	input wire clk;
	input wire we3;
	input wire [3:0] ra1;
	input wire [3:0] ra2;
	input wire [3:0] wa3;
	input wire [31:0] wd3;
	input wire [31:0] r15;
	output wire [31:0] rd1;
	output wire [31:0] rd2;
	reg [31:0] rf [14:0];
	always @(posedge clk)
		if (we3)
			rf[wa3] <= wd3;
	assign rd1 = (ra1 == 4'b1111 ? r15 : rf[ra1]);
	assign rd2 = (ra2 == 4'b1111 ? r15 : rf[ra2]);
endmodule

module top (
	clk,
	reset,
	WriteData,
	DataAdr,
	MemWrite
);
	input wire clk;
	input wire reset;
	output wire [31:0] WriteData;
	output wire [31:0] DataAdr;
	output wire MemWrite;
	wire [31:0] PC;
	wire [31:0] Instr;
	wire [31:0] ReadData;
	arm arm(
		.clk(clk),
		.reset(reset),
		.PC(PC),
		.Instr(Instr),
		.MemWrite(MemWrite),
		.ALUResult(DataAdr),
		.WriteData(WriteData),
		.ReadData(ReadData)
	);
	imem imem(
		.a(PC),
		.rd(Instr)
	);
	dmem dmem(
		.clk(clk),
		.we(MemWrite),
		.a(DataAdr),
		.wd(WriteData),
		.rd(ReadData)
	);
endmodule
