module Instruction_Memory(reset, read_address, Instruction_out);
//declaring input output ports
input reset;	//single bit input
input [31:0] read_address;	// 32-bit input
output[31:0] Instruction_out;	// 32-bit output

//making memory

reg[31:0] Memory[63:0]; // memory consists of 64 registers each of 32 bit width

assign Instruction_out = Memory[read_address];	// assign what is placed at memory address to Instruction_out
integer k;
always @ (posedge reset)
begin

for (k = 0; k < 64; k = k + 1)
begin
Memory[k] = 32'h00000000;	// reset memory to Zero
end
end
endmodule
