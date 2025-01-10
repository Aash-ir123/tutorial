module ALU_Control (ALU_Opin, func7, func3, ALU_ControlOut);
input [1:0] ALU_Opin; 
input [31:25] func7;
input [14:12] func3;

//(note that the only bits with different values for the four R-format instructions are bits 30, 14, 13, and 12). Thus, we only need these four funct field bits asinput for ALU control instead of all 10.


output reg [3:0] ALU_ControlOut;

always @ (*) 
begin
    case ({ALU_Opin, func7, func3})  // Concatenate ALU_Opin, func7, and func3 into a 12-bit value
        12'b00_xxxxxxx_xxx : ALU_ControlOut = 4'b0010; // ADD
        12'bx1_xxxxxxx_xxx : ALU_ControlOut = 4'b0110; // SUB
        12'b1x_0000000_000 : ALU_ControlOut = 4'b0010; // ADD
        12'b1x_0100000_000 : ALU_ControlOut = 4'b0110; // SUB
        12'b1x_0000000_111 : ALU_ControlOut = 4'b0000; // AND
        12'b1x_0000000_110 : ALU_ControlOut = 4'b0001; // OR
        default: ALU_ControlOut = 4'b1111; // Default value (if none of the cases match)
    endcase
end

endmodule
