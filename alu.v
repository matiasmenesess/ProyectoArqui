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
            //4'b1110: ALUResult = SrcA << SrcB[4:0];      // LSL
            //4'b1111: ALUResult = SrcA >> SrcB[4:0];    // LSR
            //4'b1101: ALUResult = (SrcA >> SrcB[4:0]) | (SrcA << (32 - SrcB[4:0]));  // ROR
            // TST
            //4'b1110: begin
            //ALUResult = SrcA & SrcB;  // Operación AND
            //ALUFlags = {ALUResult[31], (ALUResult == 32'b0), 2'b0};  // Solo actualiza los flags
            //end

            //TEQ
            //4'b1111: begin
            //ALUResult = SrcA ^ SrcB;  // Operación XOR
            //ALUFlags = {ALUResult[31], (ALUResult == 32'b0), 2'b0};  // Solo actualiza los flags
            //end

            //4'b1100: ALUResult = SrcA & ~SrcB;  // BIQ





            


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
