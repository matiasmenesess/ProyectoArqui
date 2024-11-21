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
            4'b0000: ALUResult = SrcA + SrcB;       // ADD
            4'b0001: ALUResult = SrcA - SrcB;       // SUB
            4'b0010: ALUResult = SrcB;              // MOV
            4'b0011: ALUResult = SrcA & SrcB;       // AND
            4'b0100: ALUResult = SrcA | SrcB;       // ORR
            4'b0101: ALUResult = SrcC - (SrcA * SrcB); // MLS
            4'b0110: ALUResult = SrcA * SrcB;       // MUL
            4'b0111: ALUResult = SrcC + (SrcA * SrcB); // MLA
            4'b1000: ALUResult = SrcA - SrcB - ~CarryIn; // SBC
            4'b1001: ALUResult = SrcB - SrcA;           // RSB
            4'b1010: ALUResult = SrcA + SrcB + CarryIn; //ADC
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
