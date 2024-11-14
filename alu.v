module alu (
    input wire [31:0] SrcA,
    input wire [31:0] SrcB,
    input wire [2:0] ALUControl,
    output reg [31:0] ALUResult,
    output reg [3:0] ALUFlags
);
    always @(*) begin
        case (ALUControl)
            3'b000: ALUResult = SrcA + SrcB;
            3'b001: ALUResult = SrcA - SrcB;
            3'b010: ALUResult = SrcA & SrcB;
            3'b011: ALUResult = SrcA | SrcB;
            3'b100: ALUResult = SrcA ^ SrcB;
            default: ALUResult = 32'b0;
        endcase
        
        ALUFlags[0] = (ALUResult == 0);
        ALUFlags[1] = ALUResult[31];
        ALUFlags[2] = 0;
        ALUFlags[3] = 0;
    end
endmodule
