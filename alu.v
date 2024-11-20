module alu (
    input wire [31:0] SrcA,
    input wire [31:0] SrcB,
    input wire [31:0] SrcC, 
    input wire [2:0] ALUControl,
    input wire CarryIn,  
    output reg [31:0] ALUResult,
    output reg [3:0] ALUFlags
);
    wire [32:0] sum;  

    always @(*) begin
        case (ALUControl)
            3'b000: ALUResult = SrcA + SrcB;       // ADD
            3'b001: ALUResult = SrcA - SrcB;       // SUB
            3'b010: ALUResult = SrcB;              // MOV
            3'b011: ALUResult = SrcA & SrcB;       // AND
            3'b100: ALUResult = SrcA | SrcB;       // ORR
            3'b101: ALUResult = SrcC - (SrcA * SrcB); // MLS
            3'b110: ALUResult = SrcA * SrcB;       // MUL
            3'b111: ALUResult = SrcC + (SrcA * SrcB); // MLA
            3'b1000: ALUResult = SrcA - SrcB - ~CarryIn; // SBC
            3'b1001: ALUResult = SrcB - SrcA;           // RSB
            3'b1010: ALUResult = SrcA + SrcB + CarryIn; //ADC
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