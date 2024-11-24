module controller (
    clk,
    reset,
    Cond,
    Instr,
    ALUFlags,
    RegSrcD,
    ImmSrcD,
    ALUSrcE,
    ALUControlE,
    PCSrcW,
    RegWriteW,
    MemWriteM,
    MemtoRegW
);
    input wire clk;
    input wire reset;
    input wire [31:28] Cond;
    input wire [27:12] Instr;
    input wire [3:0] ALUFlags;

    output wire [1:0] RegSrcD;
    output wire [1:0] ImmSrcD;
    output wire ALUSrcE;
    output wire [1:0] ALUControlE;
    output wire PCSrcW;
    output wire RegWriteW;
    output wire MemWriteM;
    output wire MemtoRegW;

    wire PCSrcD, RegWriteD, MemtoRegD, MemWriteD, BranchD, ALUSrcD;
    wire [1:0] ALUControlD, FlagWriteD;
    wire [3:0] Flags, CondE, FlagsE;
    wire PCSrcE, RegWriteE, MemtoRegE, MemWriteE, BranchE;
    wire [1:0] FlagWriteE;
    wire PCSrcM, RegWriteM, MemtoRegM;
    wire CondExE;

    // Etapa D
    flopd D (
        .clk(clk),
        .PCSrcD(PCSrcD),
        .RegWriteD(RegWriteD),
        .MemtoRegD(MemtoRegD),
        .MemWriteD(MemWriteD),
        .ALUControlD(ALUControlD),
        .BranchD(BranchD),
        .ALUSrcD(ALUSrcD),
        .FlagWriteD(FlagWriteD),
        .Cond(Cond),
        .Flags(Flags),
        .PCSrcE(PCSrcE),
        .RegWriteE(RegWriteE),
        .MemtoRegE(MemtoRegE),
        .MemWriteE(MemWriteE),
        .ALUControlE(ALUControlE),
        .BranchE(BranchE),
        .ALUSrcE(ALUSrcE),
        .FlagWriteE(FlagWriteE),
        .CondE(CondE),
        .FlagsE(FlagsE)
    );

     condlogic cl(
    		.clk(clk),
    		.reset(reset),
    		.Cond(CondE),
    		.ALUFlags(ALUFlags),
    		.FlagW(FlagWriteE),
    		.PCS(PCSrcE),
    		.RegW(RegWriteE),
    		.MemW(MemWriteE),
    		.PCSrc(PCSrcEpostCondLogic),
    		.RegWrite(RegWriteEpostCondLogic),
    		.MemWrite(MemWriteEpostCondLogic),
    		.Branch(BranchE),
    		.BranchTakenE(BranchTakenE),
    		.FlagsE(FlagsE),
    		.FlagsNext(FlagsNext)
    );

    flope E (
        .clk(clk),
        .PCSrcE(PCSrcE),
        .RegWriteE(RegWriteE),
        .MemtoRegE(MemtoRegE),
        .MemWriteE(MemWriteE),
        .PCSrcM(PCSrcM),
        .RegWriteM(RegWriteM),
        .MemtoRegM(MemtoRegM),
        .MemWriteM(MemWriteM)
    );

    flopm M (
        .clk(clk),
        .PCSrcM(PCSrcM),
        .RegWriteM(RegWriteM),
        .MemtoRegM(MemtoRegM),
        .PCSrcW(PCSrcW),
        .RegWriteW(RegWriteW),
        .MemtoRegW(MemtoRegW)
    );

    assign RegWriteE = RegWriteE & CondExE;
    assign PCSrcE = (BranchE & CondExE) | (PCSrcE & CondExE);
    assign MemWriteE = MemWriteE & CondExE;
endmodule
