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
        Match_1E_M, //hazard
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
    

	
endmodule
