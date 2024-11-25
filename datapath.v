module datapath (
  clk,
	reset,
	PC,
	InstrF,
	MemWrite,
	ALUResult,
	WriteData,
	ReadData
);
	input wire clk;
	input wire reset;
	output wire [31:0] PC;
	input wire [31:0] InstrF;
	output wire MemWrite;
	output wire [31:0] ALUResult;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
	wire [3:0] ALUFlags; // cuando agreguemos Q debemos cambiarlo a 4:0, tdv
	
	controller c(
		.clk(clk),
		.reset(reset),
		.ALUFlags(ALUFlags),
		.InstrD(InstrF),
		.BranchTakenE(BranchTakenE),
		.RegSrcD(RegSrc),
		.RegWriteW(RegWriteW),
		.RegWriteM(RegWriteM),
		.ImmSrcD(ImmSrc),
		.ALUSrcE(ALUSrc),
		.ALUControlE(ALUControl),
		.MemWriteM(MemWrite),
		.MemtoRegW(MemtoReg),
		.PCSrcW(PCSrc),
		.PCSrcE(PCSrcE),
       		.PCSrcD(PCSrcD),
        	.PCSrcM(PCSrcM),
		.MemtoRegE(MemtoRegE)
	);
	
endmodule
