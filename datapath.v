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
    BranchPredictor,
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
    input wire BranchPredictor;
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
        .s(BranchPredictor),
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
    
    flopenrCLR #(32) RegFToD (
        .clk(clk),
        .reset(reset),
        .d(InstrF),
        .q(InstrD),
        .en(~StallD),
        .clr(FlushD)
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
