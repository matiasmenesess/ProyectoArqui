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
            3'b101: begin
                ALUResult = SrcA - SrcB;
                ALUFlags[0] = (ALUResult == 0);
                ALUFlags[1] = ALUResult[31];
            end
            3'b110: begin
                ALUResult = SrcA & SrcB;
                ALUFlags[0] = (ALUResult == 0);
                ALUFlags[1] = ALUResult[31];
            end
            3'b111: begin
                ALUResult = SrcA ^ SrcB;
                ALUFlags[0] = (ALUResult == 0);
                ALUFlags[1] = ALUResult[31];
            end
            default: ALUResult = 32'b0;
        endcase
    end
endmodule
