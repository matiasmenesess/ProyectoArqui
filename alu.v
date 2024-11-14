module alu (
    input wire [31:0] SrcA,
    input wire [31:0] SrcB,
    input wire [1:0] ALUControl,
    output reg [31:0] ALUResult,
    output reg [3:0] ALUFlags
);
    always @(*) begin
        case (ALUControl)
            2'b00: ALUResult = SrcA + SrcB;
            2'b01: ALUResult = SrcA - SrcB;
            2'b10: ALUResult = SrcA & SrcB;
            2'b11: ALUResult = SrcA | SrcB;
            default: ALUResult = 32'b0;
        endcase

        // Set flags
        ALUFlags[0] = (ALUResult == 0);
        ALUFlags[1] = ALUResult[31];
        ALUFlags[2] = 0;
        ALUFlags[3] = 0;
    end
endmodule