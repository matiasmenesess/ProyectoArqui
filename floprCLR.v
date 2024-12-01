module floprCLR (
	clk,
	reset,
	d,
	q,
	clr
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire clr;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset | clr)
			q <= 0;
		else
			q <= d;
endmodule
