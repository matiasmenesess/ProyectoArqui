module imem (
	a,
	rd
);
	input wire [31:0] a;
	output wire [31:0] rd;
	reg [31:0] RAM [63:0];
	initial $readmemh("D:/Proyectos-Verilog/PROYECTO-ARQUI-2024-2/PROYECTO-ARQUI-2024-2.srcs/sim_1/new/memfile.dat", RAM);
	assign rd = RAM[a[31:2]];
endmodule
