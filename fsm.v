module fsm(input clk, input reset, input ipt, output otpt);
    reg [1:0] state, nextstate;
    
    parameter S0 = 2'b00; //strongly not taken
    parameter S1 = 2'b01; //weakly not taken
    parameter S2 = 2'b10; //weakly taken
    parameter S3 = 2'b11; //strongly taken
    
    always @ (posedge clk or posedge reset) begin
        if (reset)
            state <= S0;
        else 
            state <= nextstate;
    end
    
    always @ (*) begin 
        case (state)
            S0: nextstate = ipt ? S1 : S0;
            S1: nextstate = ipt ? S2 : S0;
            S2: nextstate = ipt ? S3 : S1;
            S3: nextstate = ipt ? S3 : S2;  
            default: nextstate = S0;
        endcase
    end
    
    assign otpt = (state == S2) | (state == S3);  
endmodule
