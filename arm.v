module arm (
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
	
	wire [3:0] ALUFlags;
	wire RegWriteW;
	wire RegWriteM;
	wire ALUSrc;
	wire MemtoRegE;
	wire PCSrc;
	wire [1:0] RegSrc;
	wire [1:0] ImmSrc;
	wire [1:0] ALUControl;
	wire [31:0] ExtImm;
	wire BranchTakenE;
	wire Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W;
    	wire [1:0] ForwardAE, ForwardBE;
    	wire Match_12D_E, FlushE, StallF, StallD;
   	wire [31:0] RD1D; //rd2 del decode
	wire [31:0] RD2D;  //rd1 del decode
	
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
	
	datapath dp(
		.clk(clk),
		.reset(reset),
		.RegSrcD(RegSrc),
		.RegWriteW(RegWriteW),
		.ImmSrcD(ImmSrc),
		.ALUSrcE(ALUSrc),
		.ALUControlE(ALUControl),
		.MemtoRegW(MemtoReg),
		.PCSrcW(PCSrc),
		.ALUFlags(ALUFlags),
		.PCF(PC),
		.InstrF(InstrF),
		.ALUOutM(ALUResult),
		.WriteDataE(WriteData),
		.ReadDataM(ReadData),
		.ExtImmD(ExtImm), 
		.BranchTakenE(BranchTakenE),
		.Match_1E_M(Match_1E_M), 
        	.Match_1E_W(Match_1E_W), 
        	.Match_2E_M(Match_2E_M), 
        	.Match_2E_W(Match_2E_W),
        	.Match_12D_E(Match_12D_E),
        	.ForwardAE(ForwardAE), 
        	.ForwardBE(ForwardBE),
		.StallF(StallF),
		.StallD(StallD),
		.RD1D(RD1D), 
		.RD2D(RD2D),
		.FlushE(FlushE)
	);
	
	hazard_unit hazard_unit(
	        .clk(clk), 
	        .reset(reset), 
	        .Match_1E_M(Match_1E_M), 
	        .Match_1E_W(Match_1E_W), 
	        .Match_2E_M(Match_2E_M), 
	        .Match_2E_W(Match_2E_W),
	        .Match_12D_E(Match_12D_E),
	        .RegWriteM(RegWriteM), 
	        .RegWriteW(RegWriteW), 
	        .BranchTakenE(BranchTakenE), 
	        .MemtoRegE(MemtoRegE),
	        .PCSrcW(PCSrcW),
	        .PCSrcE(PCSrcE),
	        .PCSrcD(PCSrcD),
	        .PCSrcM(PCSrcM),
	        .ForwardAE(ForwardAE), 
	        .ForwardBE(ForwardBE),
	        .StallD(StallD), 
	        .StallF(StallF), 
	        .FlushD(FlushD), 
	        .FlushE(FlushE)
    );
endmodule
