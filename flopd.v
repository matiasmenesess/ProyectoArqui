module flopd(
	clk,
	reset,
	PCSrcD,
	RegWriteD,
	MemtoRegD,
	MemWriteD,
	ALUControlD,
	BranchD,
	ALUSrcD,
	FlagWriteD,
	Cond,
	Flags,
	PCSrcE,
	RegWriteE,
	MemtoRegE,
	MemWriteE,
	ALUControlE,
	BranchE,
	ALUSrcE,
	FlagWriteE,
	CondE,
	FlagsE
);
	input wire clk;
	input wire reset;
	input wire PCSrcD;
	input wire RegWriteD;
	input wire MemtoRegD;
	input wire MemWriteD;
	input wire [1:0] ALUControlD;
	input wire BranchD;
	input wire ALUSrcD;
	input wire [1:0] FlagWriteD;
	input wire [3:0] Cond;
	input wire [3:0] Flags;
	output reg PCSrcE;
	output reg RegWriteE;
	output reg MemtoRegE;
	output reg MemWriteE;
	output reg [1:0] ALUControlE;
	output reg BranchE;
	output reg ALUSrcE;
	output reg [1:0] FlagWriteE;
	output reg [3:0] CondE;
	output reg [3:0] FlagsE;
	
	always @ (posedge clk or posedge reset)
	begin
		if (reset)
		begin
			PCSrcE <= 0;
			RegWriteE <= 0;
			MemtoRegE <= 0;
			MemWriteE <= 0;
			ALUControlE <= 0;
			BranchE <= 0;
			ALUSrcE <= 0;
			FlagWriteE <= 0;
			CondE <= 0;
			FlagsE <= 0;
		end
		else
		begin
			PCSrcE <= PCSrcD;
			RegWriteE <= RegWriteD;
			MemtoRegE <= MemtoRegD;
			MemWriteE <= MemWriteD;
			ALUControlE <= ALUControlD;
			BranchE <= BranchD;
			ALUSrcE <= ALUSrcD;
			FlagWriteE <= FlagWriteD;
			CondE <= Cond;
			FlagsE <= Flags;
		end
	end


endmodule
