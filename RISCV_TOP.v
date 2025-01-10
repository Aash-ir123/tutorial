module RISCV_TOP(
input clk, reset
);

//Instantiation of Modules
// Lets make a 32- bit wire that connects the output of Program_counter with input of Instruction_Memory 

wire [31:0] InstructionTop, InstructionoutTop, read_data1Top, read_data2Top, ALUresultTop;
wire [3:0]  ALUControlTop;
wire  RegWriteTop, MemWriteTop, MemReadTop;
wire [1:0] ALUOpTop;

Program_counter Program_counter // Program Counter
(.clk(clk),
.reset(reset),
.PC_in(),
.PC_out(InstructionTop)
);

Instruction_Memory Instruction_Memory //Instruction Memory
(.reset(reset), 
.read_address(InstructionTop),
.Instruction_out(InstructionoutTop)
);

Register_File Register_File
(.clk(clk),
.reset(reset), 
.RS1(InstructionoutTop), 
.RS2(InstructionoutTop), 
.Wreg(InstructionoutTop), 
.Write_data(ALUresultTop), 
.RegWrite(RegWriteTop), 
.Read_data1(read_data1Top), 
.Read_data2(read_data2Top)
);

ALU ALU
(.A(read_data1Top),
.B(read_data2Top),
.ALUControl_in(ALUControlTop),
.zero(), 
.ALU_Result(ALUresultTop)
);

Data_Memory Data_Memory
(.clk(), 
.reset(), 
.MemRead(MemReadTop), 
.MemWrite(MemWriteTop), 
.Address(ALUresultTop), 
.Write_data(), 
.Read_data()
);

ALU_Control ALU_Control
(.ALU_Opin(ALUOpTop), 
.func7(InstructionoutTop), 
.func3(InstructionoutTop), 
.ALU_ControlOut(ALUControlTop)
);

Control_Unit Control_Unit
(.OpCode(),
.Branch(),
.MemRead(MemReadTop),
.MemToReg(),
.MemWrite(MemWriteTop),
.ALUSrc(),
.RegWrite(RegWriteTop),
.ALUOP_Out(ALUOpTop)
);
endmodule 