module ALU (A,B,ALUControl_in,zero, ALU_Result);
input [31:0] A,B;		// 32-bit inputs
input [3:0] ALUControl_in; 	// 4 bit input (control signal) that determines the operation ALU performs 'add,subtract,multiply,divide'
output reg [31:0] ALU_Result;	// 32-bit output
output reg zero;		// single bit output

always @ (*)			//  block is executed whenever any signal in the block's logic changes
begin
case (ALUControl_in)
4'b0000 : begin 
zero <= 0; 			// a flag or status signal to indicate status condtion
ALU_Result <= A & B;		// Bitwise AND operation btw A and B, stored in ALU_Result
end

4'b0001 : begin 
zero <= 0; 			// a flag or status signal to indicate status condtion
ALU_Result <= A | B;		// Bitwise OR operation btw A and B, stored in ALU_Result
end

4'b0010 : begin 
zero <= (A + B == 0) ? 1 : 0; 	// The zero flag is set to 1 if the addition result equals zero, otherwise 0.
ALU_Result <= A + B;		// Bitwise ADD operation btw A and B, stored in ALU_Result
end

4'b0110 : begin 
zero <= (A - B == 0) ? 1 : 0; 	// The zero flag is set to 1 if the subtraction result equals zero, otherwise 0.
ALU_Result <= A - B;		// Bitwise SUBTRACT operation btw A and B, stored in ALU_Result
end

default: begin
zero <= 0;
ALU_Result <= 0; 		// Default case
end

endcase
end
endmodule