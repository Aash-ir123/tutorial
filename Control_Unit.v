module Control_Unit(OpCode, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite, ALUOP_Out );
input [6:0] OpCode;
output reg Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite;
output reg [1:0] ALUOP_Out;

always @ (*)
begin
case (OpCode)
7'b0110011:	// For R-Type Instruction
begin
Branch   <= 0;
MemRead  <= 0;
MemToReg <= 0;
MemWrite <= 0;
ALUSrc   <= 0;
RegWrite <= 1;
ALUOP_Out <= 2'b10;
end


7'b0000011:	// For load type Instruction
begin
Branch   <= 0;
MemRead  <= 1;
MemToReg <= 1;
MemWrite <= 0;
ALUSrc   <= 1;
RegWrite <= 1;
ALUOP_Out <= 2'b00;
end

7'b0100011:	// For store type Instruction
begin
Branch   <= 0;
MemRead  <= 0;
MemToReg <= x;
MemWrite <= 1;
ALUSrc   <= 1;
RegWrite <= 0;
ALUOP_Out <= 2'b00;
end

7'b1100011:	// For branch equal type Instruction
begin
Branch   <= 1;
MemRead  <= 0;
MemToReg <= x;
MemWrite <= 0;
ALUSrc   <= 0;
RegWrite <= 0;
ALUOP_Out <= 2'b01;
end

endcase
end
endmodule
