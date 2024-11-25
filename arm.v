module arm (
    clk,
    reset,
    PC,
    Instr,
    MemWrite,
    ALUResult,
    WriteData,
    ReadData
);
    input wire clk;
    input wire reset;
    output wire [31:0] PC;
    input wire [31:0] Instr;
    output wire MemWrite;
    output wire [31:0] ALUResult;
    output wire [31:0] WriteData;
    input wire [31:0] ReadData;

    wire [3:0] ALUFlags;
    wire RegWrite;
    wire ALUSrc;
    wire MemtoReg;
    wire PCSrc;
    wire [1:0] RegSrcD;
    wire [1:0] ImmSrc;
    wire [3:0] ALUControl;

    controller c(
        .clk(clk),
        .reset(reset),
        .Instr(Instr[31:12]),
        .ALUFlags(ALUFlags),
        .RegSrcD(RegSrcD),
        .RegWriteW(RegWrite),
        .ImmSrcD(ImmSrc),
        .ALUSrcE(ALUSrc),
        .ALUControlE(ALUControl),
        .MemWriteM(MemWrite),
        .MemtoRegW(MemtoReg),
        .PCSrcW(PCSrc)
    );

    datapath dp(
        .clk(clk),
        .reset(reset),
        .RegSrc(RegSrc),
        .RegWrite(RegWrite),
        .ImmSrc(ImmSrc),
        .ALUSrc(ALUSrc),
        .ALUControl(ALUControl),
        .MemtoReg(MemtoReg),
        .PCSrc(PCSrc),
        .ALUFlags(ALUFlags),
        .PC(PC),
        .Instr(Instr),
        .ALUResult(ALUResult),
        .WriteData(WriteData),
        .ReadData(ReadData)
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
