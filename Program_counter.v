

module Program_counter(clk,reset,PC_in,PC_out);

// declaring input output ports
input clk,reset;  	// single bit inputs
input [31:0] PC_in; 	// 32-bit input 
output reg [31:0] PC_out;	// 32-bit output 

always @ (posedge clk)	// as PC holds the current address of next instruction that needs to be executed at positive or negative edge of the clock. 
begin
if (reset)
PC_out <= 32'h00000000;	// if reset is assertive(logic is high) assign 32-bit zero hex values to PC_out
else
PC_out <= PC_in;	// otherwise PC_in is assigned to PC_out
end
endmodule 