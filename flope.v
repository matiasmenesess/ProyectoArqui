module flope(
	clk,
	reset,
	PCSrcE,
	RegWriteE,
	MemtoRegE,
	MemWriteE,
	PCSrcM,
	RegWriteM,
	MemtoRegM,
	MemWriteM,
);
	input wire clk;
	input wire reset;
	input wire PCSrcE;
	input wire RegWriteE;
	input wire MemtoRegE;
	input wire MemWriteE;
	output reg PCSrcM;
	output reg RegWriteM;
	output reg MemtoRegM;
	output reg MemWriteM;
	
	always @ (posedge clk or posedge reset)
	begin
		if (reset)
		begin
			PCSrcM <= 0;
			RegWriteM <= 0;
			MemtoRegM <= 0;
			MemWriteM <= 0;
		end
		else
		begin
			PCSrcM <= PCSrcE;
			RegWriteM <= RegWriteE;
			MemtoRegM <= MemtoRegE;
			MemWriteM <= MemWriteE;
		end
	end

endmodule
