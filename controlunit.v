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