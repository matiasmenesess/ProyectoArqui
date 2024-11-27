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
            4'b0000: ALUResult = SrcA + SrcB;            // ADD
            4'b0001: ALUResult = SrcA - SrcB;            // SUB
            4'b0010: ALUResult = SrcB;                   // MOV
            4'b0011: ALUResult = SrcA & SrcB;            // AND
            4'b0100: ALUResult = SrcA | SrcB;            // ORR
            4'b0101: ALUResult = SrcC - (SrcA * SrcB);   // MLS
            4'b0110: ALUResult = SrcA ^ SrcB;            // EOR
            4'b0111: ALUResult = ~SrcB;                  // MVN
            4'b1000: ALUResult = SrcC + (SrcA * SrcB);   // MLA
            4'b1001: ALUResult = SrcB - SrcA;            // RSB
            4'b1010: ALUResult = SrcA + SrcB + CarryIn;  // ADC
            4'b1011: ALUResult = SrcA - SrcB - ~CarryIn; // SBC
            4'b1100: ALUResult = SrcA * SrcB;            // MUL
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
	MemWrite,
	Branch,
	BranchTakenE,
	FlagsE,
	FlagsNext
);
	input wire clk;
	input wire reset;
	input wire [3:0] Cond;
	input wire [3:0] ALUFlags;
	input wire [1:0] FlagW;
	input wire PCS;
	input wire RegW;
	input wire MemW;
	input wire Branch;
	input wire [3:0] FlagsE;
	output wire PCSrc;
	output wire RegWrite;
	output wire MemWrite;
	output wire BranchTakenE;
	output wire [3:0] FlagsNext;
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
	assign BranchTakenE = Branch & CondEx;
	assign FlagsNext = Flags;
	
endmodule

module controller (
	clk,
	reset,
	InstrD,
	ALUFlags,
	RegSrcD,
	RegWriteM,
	RegWriteW,
	ImmSrcD,
	ALUSrcE,
	ALUControlE,
	MemWriteM,
	MemtoRegE,
	MemtoRegW,
	PCSrcW, 
	BranchTakenE,
	PCSrcD,
    PCSrcE,
   	PCSrcM
);

	input wire clk;
	input wire reset;
	input wire [31:0] InstrD;
	input wire [3:0] ALUFlags;
	output wire [1:0] RegSrcD;
	output wire RegWriteW;
	output wire RegWriteM;
	output wire [1:0] ImmSrcD;
	output wire ALUSrcE;
	output wire [1:0] ALUControlE;
	output wire MemWriteM;
	output wire MemtoRegW;
	output wire PCSrcW;
	output wire PCSrcM;
	output wire PCSrcE; 
	output wire PCSrcD;
	output wire BranchTakenE;
	output wire MemtoRegE;
	wire RegWriteD;
	wire MemWriteD; 
	wire MemtoRegD;
	wire ALUSrcD;
	wire BranchD;
	wire [1:0] FlagWriteD;
	wire [1:0] ALUControlD;
	wire CondPCSrcE;
	wire RegWriteE;
	wire CondRegWriteE; //dsp de aplicarle cond logic 
	wire MemWriteE;
	wire CondMemWriteE;
	wire BranchE;
	wire ALUSrcE;
	wire [1:0] FlagWriteE;
	wire [3:0] CondE;
	wire FlagsE;
	wire FlagsNext;
	wire MemtoRegM, MemWriteM; 
	assign CondE = InstrD[31:28];

	decode dec (
		.Op(InstrD[27:26]),
		.Funct(InstrD[25:20]),
		.Rd(InstrD[15:12]),
		.FlagW(FlagWriteD),
		.PCS(PCSrcD),
		.RegW(RegWriteD),
		.MemW(MemWriteD),
		.MemtoReg(MemtoRegD),
		.ALUSrc(ALUSrcD),
		.ALUControl(ALUControlD),
		.ImmSrc(ImmSrcD),
		.RegSrc(RegSrcD),
		.Branch(BranchD)
	);

	//registro entre decode y execute
	flopr #(14) RegDecExec(
		.clk(clk), .reset(reset),
		.d({PCSrcD, RegWriteD, MemtoRegD, MemWriteD, ALUControlD, BranchD, ALUSrcD, FlagWriteD}),  
		.q({PCSrcE, RegWriteE, MemtoRegE, MemWriteE, ALUControlE, BranchE, ALUSrcE, FlagWriteE})
	);

	//registro para ALUFlags
	flopr #(4) RegALUFlags (
		.clk(clk), 
		.reset(reset), 
		.d(FlagsNext), 
		.q(FlagsE)
	);

	condlogic cl (
		.clk(clk),
		.reset(reset),
		.Cond(CondE),
		.ALUFlags(ALUFlags),
		.FlagW(FlagWriteE),
		.PCS(PCSrcE),
		.RegW(RegWriteE),
		.MemW(MemWriteE),
		.PCSrc(CondPCSrcE),
		.RegWrite(CondRegWriteE),
		.MemWrite(CondMemWriteE),
		.Branch(BranchE),
		.BranchTakenE(BranchTakenE),
		.FlagsE(FlagsE),
		.FlagsNext(FlagsNext)
	);

	//registro entre execute y memory
	flopr #(4) RegExecMem(
		.clk(clk), 
		.reset(reset), 
		.d({CondPCSrcE, CondRegWriteE, MemtoRegE, CondMemWriteE}), 
		.q({PCSrcM, RegWriteM, MemtoRegM, MemWriteM})
	);

	//registro entre memory y writeback
	flopr #(4) RegMemWB (
		.clk(clk), 
		.reset(reset), 
		.d({PCSrcM, RegWriteM, MemtoRegM}), 
		.q({PCSrcW, RegWriteW, MemtoRegW})
	);

endmodule

module datapath (
    clk,
    reset,
    RegSrcD,
    RegWriteW,
    ImmSrcD,
    ALUSrcE,
    ALUControlE,
    MemtoRegW,
    PCSrcW,
    ALUFlags,
    PCF,
    InstrF,
    ALUOutM,
    WriteDataE,
    ReadDataM, 
    ExtImmD, 
    RD1D, 
    RD2D, 
    BranchTakenE,
    MemtoRegE,
    Match_1E_M, // hazard
    Match_1E_W, 
    Match_2E_M,
    Match_2E_W, 
    Match_12D_E,
    ForwardAE,
    ForwardBE,
    StallF,
    StallD,
    FlushE
);

    wire [31:0] RD1E;
    wire [31:0] RD2E;
    wire [31:0] ExtImmE;
    wire [3:0] WA3E;
    wire [3:0] RA1E;
    wire [3:0] RA2E;
    wire [31:0] SrcAE;
    wire [31:0] SrcBE;
    wire [31:0] SrcCE;
    wire [31:0] ALUOutM;
    wire [31:0] WriteDataM;
    wire [3:0] WA3M;
    wire [31:0] ReadDataW;
    wire [31:0] ALUOutW;
    wire [31:0] PCNext;
    wire [31:0] ResultW;
    wire [3:0] RA1D;
    wire [3:0] RA2D;
    wire [3:0] WA3W;
    wire [31:0] ALUResultE;
    wire [31:0] PCPlus4F;
    wire [31:0] PC;
    wire [31:0] InstrD;
    wire [31:0] PCPlus8D;
       
    input wire clk;
    input wire reset;
    input wire BranchTakenE;
    input wire [1:0] RegSrcD;
    input wire [1:0] ForwardAE, ForwardBE;
    input wire MemtoRegE, StallF, StallD, FlushE;
    input wire RegWriteW;
    input wire [1:0] ImmSrcD;
    input wire ALUSrcE;
    input wire [1:0] ALUControlE;
    input wire MemtoRegW;
    input wire PCSrcW;
    input wire [31:0] InstrF;
    input wire [31:0] ReadDataM;
    
    output wire [3:0] ALUFlags;
    output wire [31:0] PCF;
    output wire [31:0] ALUOutM;
    output wire [31:0] WriteDataE;
    output wire [31:0] ExtImmD;
    output wire [31:0] RD1D;
    output wire [31:0] RD2D;
    output wire Match_1E_M;
    output wire Match_1E_W; 
    output wire Match_2E_M;
    output wire Match_2E_W; 
    output wire Match_12D_E;

    mux2 #(32) MUX_PC(
        .d0(PCPlus4F),
        .d1(ResultW),
        .s(PCSrcW),
        .y(PCNext)
    );    
    mux2 #(32) MUX_2PC(
        .d0(PCNext),
        .d1(ALUResultE),
        .s(BranchTakenE),
        .y(PC)
    );

    flopenr #(32) Reg_PC(
        .clk(clk),
        .reset(reset),
        .en(~StallF),
        .d(PC),
        .q(PCF)
    );
    
    adder #(32) PC_Adder(
        .a(PCF),
        .b(32'b100),
        .y(PCPlus4F)
    );
    
    flopr #(32) RegFToD (
        .clk(clk),
        .reset(reset),
        .d(InstrF),
        .q(InstrD)
    );
    
    //decode
    mux2 #(4) MuxRA1(
        .d0(InstrF[19:16]),
        .d1(4'b1111),
        .s(RegSrcD[0]),
        .y(RA1D)
    );
    
    mux2 #(4) MuxRA2(
        .d0(InstrF[3:0]),
        .d1(InstrF[15:12]),
        .s(RegSrcD[1]),
        .y(RA2D)
    );

    assign PCPlus8D = PCPlus4F;
    
    regfile rf(
        .clk(clk),
        .we3(RegWriteW),
        .ra1(RA1D),
        .ra2(RA2D),
        .wa3(WA3W),
        .wd3(ResultW),
        .r15(PCPlus8D),
        .rd1(RD1D),
        .rd2(RD2D)
    );
    
    extend ext(
        .Instr(InstrF[23:0]),
        .ImmSrc(ImmSrcD),
        .ExtImm(ExtImmD)
    );
    
    //execute
    flopr #(32) RegRD1(
        .clk(clk), 
        .reset(reset), 
        .d(RD1D), 
        .q(RD1E)
    );

    flopr #(32) RegRD2(
        .clk(clk), 
        .reset(reset), 
        .d(RD2D), 
        .q(RD2E)
    );

    flopr #(32) RegImm(
        .clk(clk), 
        .reset(reset), 
        .d(ExtImmD), 
        .q(ExtImmE)
    );

    assign WA3E = InstrD[15:12];
    
    flopr #(4) RegRA1(
        .clk(clk), 
        .reset(reset), 
        .d(RA1D), 
        .q(RA1E)
    );
    flopr #(4) RegRA2(
        .clk(clk), 
        .reset(reset), 
        .d(RA2D), 
        .q(RA2E)
    );
    
    mux3 #(32) Mux_ALUSrcA (
        .d0(RD1E),
        .d1(ResultW),
        .d2(ALUOutM),
        .s(ForwardAE),
        .y(SrcAE)
    );

    mux3 #(32) Mux_ALUSrcB (
        .d0(RD2E),
        .d1(ResultW),
        .d2(ALUOutM),
        .s(ForwardBE),
        .y(WriteDataE)
    );
    
    mux2 #(32) Mux_ALU_B_Imm (
        .d0(WriteDataE),
        .d1(ExtImmE),
        .s(ALUSrcE),
        .y(SrcBE)
    );

        
    alu alu(
        SrcAE,
        SrcBE,
        SrcCE,
        ALUControlE,
        CarryIn,
        ALUResultE,
        ALUFlags
    );
    
    //memory
    flopr #(32) RegALUResult(
        .clk(clk), 
        .reset(reset), 
        .d(ALUResultE), 
        .q(ALUOutM)
    );
    
    flopr #(32) RegWD(
        .clk(clk), 
        .reset(reset), 
        .d(WriteDataE), 
        .q(WriteDataM)
    );
    
    flopr #(4) RegWA3M(
        .clk(clk), 
        .reset(reset), 
        .d(WA3E), 
        .q(WA3M)
    );
    //write-back
    flopr #(32) RegALUOut(
        .clk(clk), 
        .reset(reset), 
        .d(ALUOutM), 
        .q(ALUOutW)
    );    
    flopr #(32) RegRD(
        .clk(clk), 
        .reset(reset), 
        .d(ReadDataM), 
        .q(ReadDataW)
    );
    
    flopr #(4) RegWA3W(
        .clk(clk), 
        .reset(reset), 
        .d(WA3M), 
        .q(WA3W)
    );
    mux2 #(32) MuxRes(
        .d0(ALUOutW), 
        .d1(ReadDataW), 
        .s(MemtoRegW), 
        .y(ResultW)
    );  
	
    //hazard .forwarding
    assign Match_1E_M = (RA1E == WA3M);
    assign Match_1E_W = (RA1E == WA3W);
    assign Match_2E_M = (RA2E == WA3M);
    assign Match_2E_W = (RA2E == WA3W);
    assign Match_12D_E = (RA1D == WA3E) | (RA2D == WA3E);

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
	output reg [1:0] ALUControl;
	reg [9:0] controls;
	output wire Branch;
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
	assign RegW = controls[3];
	assign MemW = controls[2];
	
	
	always @(*)
		if (ALUOp) begin
			case (Funct[4:1])
				4'b0100: ALUControl = 2'b00;
				4'b0010: ALUControl = 2'b01;
				4'b0000: ALUControl = 2'b10;
				4'b1100: ALUControl = 2'b11;
				default: ALUControl = 2'bxx;
			endcase
			FlagW[1] = Funct[0];
			FlagW[0] = Funct[0] & ((ALUControl == 2'b00) | (ALUControl == 2'b01));
		end
		else begin
			ALUControl = 2'b00;
			FlagW = 2'b00;
		end
	assign PCS = ((Rd == 4'b1111) & RegW) | Branch;
endmodule

module dmem (
	clk,
	we,
	a,
	wd,
	rd
);
	input wire clk;
	input wire we;
	input wire [31:0] a;
	input wire [31:0] wd;
	output wire [31:0] rd;
	reg [31:0] RAM [255:0];
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

module hazard_unit(
    clk,
    reset,
    Match_1E_M,
    Match_1E_W,
    Match_2E_M,
    Match_2E_W,
    ForwardAE,
    ForwardBE,
    RegWriteM,
    RegWriteW,
    MemtoRegE,
    Match_12D_E,
    FlushE,
    FlushD,
    StallF,
    StallD,
    PCSrcD,
    PCSrcE,
    PCSrcM,
    PCSrcW,
    BranchTakenE

);


    input wire clk, reset;
    input wire Match_1E_M, Match_1E_W, Match_2E_M,Match_2E_W, Match_12D_E;
    input wire RegWriteM, RegWriteW;
    input wire MemtoRegE;
    input wire PCSrcD, PCSrcE,PCSrcM, PCSrcW;
    input wire BranchTakenE;
    output reg [1:0] ForwardAE, ForwardBE;
    output wire StallF, StallD;
    output wire FlushE, FlushD;
    wire LDRStall;
    wire PCWrPendingF;

    always@(*) 
        begin
            if (Match_1E_M & RegWriteM) ForwardAE = 2'b10;
            else if (Match_1E_W & RegWriteW) ForwardAE = 2'b01;
            else ForwardAE = 2'b00;
            if (Match_2E_M & RegWriteM) ForwardBE = 2'b10;
            else if (Match_2E_W & RegWriteW) ForwardBE = 2'b01;
            else ForwardBE = 2'b00;
    end
    
    assign LDRStall = Match_12D_E & MemtoRegE;
    assign StallF = LDRStall | PCWrPendingF;
    assign PCWrPendingF = PCSrcD | PCSrcE | PCSrcM;
    assign FlushD = PCWrPendingF | PCSrcW | BranchTakenE;
    assign FlushE = LDRStall | BranchTakenE;
    assign StallD = LDRStall;
endmodule

module imem (
	a,
	rd
);
	input wire [31:0] a;
	output wire [31:0] rd;
	reg [31:0] RAM [63:0];
	initial $readmemh("memfile.mem", RAM);
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

module mux3 (
	d0,
	d1,
	d2,
	s,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] d0;
	input wire [WIDTH - 1:0] d1;
	input wire [WIDTH - 1:0] d2;
	input wire [1:0] s; 
	output wire [WIDTH - 1:0] y;
	assign y = (s == 2'b00) ? d0 :
			   (s == 2'b01) ? d1 :
			   (s == 2'b10) ? d2 :
			   d0;
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
	wire [31:0] InstrF;
	wire [31:0] ReadData;
	arm arm(
		.clk(clk),
		.reset(reset),
		.PC(PC),
		.InstrF(InstrF),
		.MemWrite(MemWrite),
		.ALUResult(DataAdr),
		.WriteData(WriteData),
		.ReadData(ReadData)
	);
	imem imem(
		.a(PC),
		.rd(InstrF)
	);
	dmem dmem(
		.clk(clk),
		.we(MemWrite),
		.a(DataAdr),
		.wd(WriteData),
		.rd(ReadData)
	);
endmodule

