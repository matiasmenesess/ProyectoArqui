module hazardunit(
	RegWriteW,
	RegWriteM,
	Match,
	ForwardBE,
	ForwardAE
);
	input wire RegWriteW;
	input wire RegWriteM;
	input wire [3:0] Match;
	output wire [1:0] ForwardAE;
	output wire [1:0] ForwardBE;

	wire Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W;

	assign {Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W} = Match;

	ForwardAE = Match_1E_M & RegWriteM ? 2'b10 : (Match_1E_W & RegWriteW ? 2'b01 : 2'b00) 
	ForwardBE = Match_2E_M & RegWriteM ? 2'b10 : (Match_2E_W & RegWriteW ? 2'b01 : 2'b00)

    //FALTAN MAS COSAS AVANZAR MIS BROS

endmodule

