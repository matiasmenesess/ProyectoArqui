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