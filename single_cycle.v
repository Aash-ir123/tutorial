// Program Counter
module Program_Counter (clk, reset, PC_in, PC_out);
input clk, reset;	// single bit inputs
input [31:0] PC_in;	// 32-bit input
output reg [31:0] PC_out;	// 32-bit output

always @ (posedge clk or posedge reset)
begin
if (reset == 1'b1)
PC_out <= 32'b00;
else
PC_out <= PC_in;
end
endmodule

// PC + 4 (Adder)
module PCplusfour(FromPc, NexttoPC);
input [31:0] FromPc;
output [31:0] NexttoPC;

assign NexttoPC = 4 + FromPc;

endmodule

// Instruction memory
module Instruction_mem(clk, reset, read_address, instruction_out);
input clk, reset;
input [31:0] read_address;
output reg [31:0] instruction_out;

reg[31:0] I_Mem [63:0]; // array of instruction memory with 63 locations and each location of width 32-bits 
integer k;

always @ (posedge clk or posedge reset)
begin
if (reset == 1'b1)
	begin
		for (k = 0; k < 64; k = k + 1)
			I_Mem[k] <= 32'b00;
		
	end
else 
begin
	instruction_out <= I_Mem[read_address];
end
end

initial begin
// R-type
I_Mem[0] = 32'b00000000000000000000000000000000;   // no operation
I_Mem[4] = 32'b0000000_11001_10000_000_01101_0110011; // add x13, x16, x25
I_Mem[8] = 32'b0100000_00101_00010_000_00011_0110011; // sub x5, x8, x3
I_Mem[12] = 32'b0000000_00011_00010_111_00010_0110011; // and x1, x2, x3
I_Mem[16] = 32'b0000000_00101_00010_110_00100_0110011; // or x4, x3, x5

// I-type
I_Mem[20] = 32'b000000000011_10101_000_10110_0010011;  // addi x22, x21, 3
I_Mem[24] = 32'b000000000001_01000_110_01001_0010011;  // ori x9, x8, 1

// L-type
I_Mem[28] = 32'b000000000111_01010_010_01000_0000011;  // lw x8, 15(x5)
I_Mem[32] = 32'b000000000011_01010_010_01001_0000011;  // lw x9, 3(x5)

// S-type
I_Mem[36] = 32'b0000000_11111_01010_010_01100_0100011; // sw x15, 12(x5)
I_Mem[40] = 32'b0000000_01110_01010_010_01010_0100011; // sw x14, 10(x5)

// SB-type
I_Mem[44] = 32'h00408663;                             // beq x9, x9, 12

end 
endmodule 

// Register File
module Register_file(clk, reset, rs1, rs2, RegWrite, rd , write_data, read_data1, read_data2);
input clk, reset, RegWrite;	//single-bit inputs with RegWrite Control signal to write contents into reg
input [4:0] rs1, rs2, rd; 	// 5-bit registers (rd is destination reg)
input [31:0] write_data;
output reg [31:0] read_data1, read_data2;

// making memory of Registers
reg[31:0] Registers [31:0];	// 32 registers with each of 32-bit wide
integer k ;
initial begin
Registers[0] = 0;
Registers[1] = 4;
Registers[2] = 3;
Registers[3] = 24;
Registers[4] = 5;
Registers[5] = 1;
Registers[6] = 44;
Registers[7] = 4;
Registers[8] = 2;
Registers[9] = 1;
Registers[10] = 23;
Registers[11] = 4;
Registers[12] = 90;
Registers[13] = 90;
Registers[14] = 20;
Registers[15] = 30;
Registers[16] = 40;
Registers[17] = 50;
Registers[18] = 60;
Registers[19] = 70;
Registers[20] = 80;
Registers[21] = 80;
Registers[22] = 90;
Registers[23] = 70;
Registers[24] = 60;
Registers[25] = 65;
Registers[26] = 4;
Registers[27] = 32;
Registers[28] = 12;
Registers[29] = 34;
Registers[30] = 5;
Registers[31] = 10;
end


always @ (posedge clk or posedge reset)
begin
if (reset == 1'b1)
	begin
		for (k = 0; k < 32; k = k + 1)
		begin
			Registers[k] <= 32'b00;
		end
	end
else if (RegWrite == 1'b1)
	begin
		Registers[rd] <= write_data;
	end
end

always @(*) begin
    read_data1 = Registers[rs1];
    read_data2 = Registers[rs2];
end
endmodule

// Immediate Generator
module Imm_Gen (Opcode, instruction, ImmExt);
input [6:0] Opcode;
input [31:0] instruction;
output reg [31:0] ImmExt;

always @ (*)
begin
	case (Opcode)
	7'b0000011 : ImmExt <= {{20{instruction[31]}}, instruction[31:20]}; // make 20 copies of 32nd sig. bits 0 or 1 and concatenate with instruction bits from 20 to 31
	7'b0100011 : ImmExt <= {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // same procedure with store type instruction
	7'b1100011 : ImmExt <= {{19{instruction[31]}}, instruction[31],instruction[30:25], instruction[11:8], 1'b0}; // 19 copies with same procedure with branch type instruction
	endcase
end
endmodule

// Control Unit
module control_unit(instruction, branch, memread, memtoreg, aluop, alusrc, memwrite, regwrite);
input [6:0]instruction;
output reg branch, memread, memtoreg, alusrc, memwrite, regwrite;
output reg [1:0] aluop;

always @ (*)
begin
	case (instruction)
	7'b0110011 : {alusrc, memtoreg, regwrite, memread, memwrite, branch, aluop} <= 8'b001000_10;
	7'b0000011 : {alusrc, memtoreg, regwrite, memread, memwrite, branch, aluop} <= 8'b111100_00;
	7'b0100011 : {alusrc, memtoreg, regwrite, memread, memwrite, branch, aluop} <= 8'b100010_00;
	7'b1100011 : {alusrc, memtoreg, regwrite, memread, memwrite, branch, aluop} <= 8'b000001_01;
	endcase
end
endmodule

// ALU
module ALU_unit(a,b,control_in,aluresult,zero);
input [31:0] a,b;
input [3:0] control_in;
output reg zero;
output reg [31:0] aluresult;

always @ (control_in or a or b)
begin
	case(control_in)
	4'b0000 : begin zero <= 0; aluresult <= a & b; end
	4'b0001 : begin zero <= 0; aluresult <= a | b; end
	4'b0010 : begin zero <= 0; aluresult <= a + b; end
	4'b0110 : begin if (a==b)zero <= 1; else zero <= 0; aluresult <= a - b; end
	endcase
end
endmodule 

// ALU control
module ALU_control (ALUop, func7, func3, control_out);
input [1:0] ALUop;
input  func7;
input [2:0] func3;
output reg [3:0] control_out;

always @ (*)
begin
	case({ALUop, func7, func3})
 	6'b00_0_000 : control_out <= 4'b0010;
	6'b01_0_000 : control_out <= 4'b0110;
	6'b10_0_000 : control_out <= 4'b0010;
	6'b10_1_000 : control_out <= 4'b0110;
	6'b10_0_111 : control_out <= 4'b0000;
	6'b10_0_110 : control_out <= 4'b0001;
	endcase
end
endmodule

// datamemory
module data_memory (clk, reset, memwrite, memread, read_address, write_data, memdata_out);
input clk, reset, memwrite, memread;
input [31:0] read_address, write_data;
output reg [31:0] memdata_out;

// making data memory
reg [31:0] dmemory [63:0]; 	// 64 dmemory locations that can be accessed to read/write each 32-bit wide
integer k ;
always @ (posedge clk or posedge reset)
begin
if (reset == 1'b1)
	begin
		for (k = 0; k < 64; k = k +1)
		begin
		dmemory[k] = 32'b00;
		end
	end
else if (memwrite)
	begin
		dmemory[read_address] <= write_data;	// when memwrite signal is high, write data in dmemory at read_address
	end
end
always @(*) begin
    if (memread)					// when memread is high, read data from read_address in dmem and when memread is low, memdata_out=0
        memdata_out = dmemory[read_address]; 
    else
        memdata_out = 32'b00;
end
endmodule


// Multiplexers
module mux1(sel1, a1, b1, mux1_out);
input [31:0] a1, b1;
input sel1;
output [31:0] mux1_out;

assign mux1_out = (sel1==1'b0) ? a1 : b1 ;
endmodule

//MUX2
module mux2(sel2, a2, b2, mux2_out);
input [31:0] a2, b2;
input sel2;
output [31:0] mux2_out;

assign mux2_out = (sel2==1'b0) ? a2 : b2 ;
endmodule

//MUX3

module mux3(sel3, a3, b3, mux3_out);
input [31:0] a3, b3;
input sel3;
output [31:0] mux3_out;

assign mux3_out = (sel3==1'b0) ? a3 : b3 ;
endmodule

// AND logic
module AND_logic(branch, zero, and_out);
input branch, zero;
output and_out;

assign and_out= branch & zero; 

endmodule

// Adder
module adder (in_1, in_2, sum_out);
input [31:0] in_1, in_2;
output [31:0] sum_out;

assign sum_out = in_1 + in_2;

endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// All modules instantiate here//////////////////////////////////////////////////////////////////////

module top (clk, reset); 
input clk, reset;

wire [31:0] PC_Top;		// wire to connect output of PC to read_address of inst_mem & input of PC_adder
wire [31:0] instruction_Top;	// wire that connects rs1, rs2, rd(dest.reg)
wire regwrite_top;		// wire that connects reg_write control signal to register_file
wire alusrc_Top;		// wire that connects ALU_Mux select line with ALUsrc and is single bit.
wire branch_Top;		// wire that connects branch (Control Unit output) with AND input1.
wire zero_Top;			// wire that connects zero (flag) with AND input2
wire sel2_Top;			// wire that connects AND output with Adder_Mux
wire memtoreg_Top;		// wire that connects memtoreg control output with Memory_Mux select line
wire memwrite_Top;		// wire that connects memwrite control signal with Data_mem input
wire memread_Top;		//  wire that connects memread control signal with Data_mem input
wire [1:0] aluop_Top;		// wire that connects ALUop of control Unit and ALU Control
wire [31:0] readdata1_Top;	// wire that connects readdata1 output with ALU 
wire [31:0] readdata2_Top;	// wire that connects readdata2 output with ALU_Mux
wire [31:0] ImmExt_Top;		// wire that connects ImmExt with ALU_Mux
wire [31:0] mux1_Top;		// wire that connects mux1_out with ALU 
wire [3:0] control_Top;		// wire that connects ALU_Control output with ALU 
wire [31:0] sum_out_Top;	// wire that connects Adder with Adder_Mux
wire [31:0] NexttoPC_Top;	// wire that connects PC Adder output with Adder_Mux
wire [31:0] PCin_Top; 		// wire that connects Adder_Mux output with PC_in
wire [31:0] address_Top;	// wire that connects ALU output with Memory_Mux & read_address of Data_mem
wire [31:0] Memdata_Top;	// wire that connects read_data output of Data_mem with Memory_Mux 2nd input
wire [31:0] WriteBack_Top;	// wire that connects memory_Mux output with Write_data of register file

// Program Counter
Program_Counter PC(.clk(clk) , .reset(reset) , .PC_in(PCin_Top) , .PC_out(PC_Top));

// PC Adder
PCplusfour PC_Adder(.FromPc(PC_Top) , .NexttoPC(NexttoPC_Top));

// Instruction Memory
Instruction_mem Inst_Memory(.clk() , .reset() , .read_address(PC_Top) , .instruction_out(instruction_Top));

// Register File
Register_file Reg_file(.clk(clk) , .reset(reset) , .rs1(instruction_Top[19:15]) , .rs2(instruction_Top[24:20]) , .RegWrite(regwrite_top) , .rd(instruction_Top[11:7]) , .write_data(WriteBack_Top) , .read_data1(readdata1_Top) , .read_data2(readdata2_Top));

// Immediate Generator
Imm_Gen imm_gen(.Opcode(instruction_Top[6:0]) , .instruction(instruction_Top) , .ImmExt(ImmExt_Top));

// Control Unit
control_unit control_unit(.instruction(instruction_Top[6:0]) , .branch(branch_Top) , .memread(memread_Top) , .memtoreg(memtoreg_Top) , .aluop(aluop_Top) , .alusrc(alusrc_Top) , .memwrite(memwrite_Top) , .regwrite(regwrite_top));

// ALU Control
ALU_control alu_control(.ALUop(aluop_Top) , .func7(instruction_Top[30]) , .func3(instruction_Top[14:12]) , .control_out(control_Top));

// ALU
ALU_unit alu(.a(readdata1_Top) , .b(mux1_Top) , .control_in(control_Top) , .aluresult(address_Top) , .zero(zero_Top));

// ALU Mux
mux1 alu_mux(.sel1(alusrc_Top) , .a1(readdata2_Top) , .b1(ImmExt_Top) , .mux1_out(mux1_Top));

// Adder
adder Adder(.in_1(PC_Top) , .in_2(ImmExt_Top) , .sum_out(sum_out_Top));

// AND Gate
AND_logic AND(.branch(branch_Top) , .zero(zero_Top) , .and_out(sel2_Top));

// MUX
mux2 Adder_mux(.sel2(sel2_Top) , .a2(NexttoPC_Top) , .b2(sum_out_Top) , .mux2_out(PCin_Top));

// data memory
data_memory data_mem(.clk() , .reset() , .memwrite(memwrite_Top) , .memread(memread_Top) , .read_address(address_Top) , .write_data(readdata2_Top) , .memdata_out(Memdata_Top));

// MUX
mux3 Memory_mux(.sel3(memtoreg_Top) , .a3(address_Top) , .b3(Memdata_Top), .mux3_out(WriteBack_Top));



endmodule 


// Writing of Testbench 
module tb_top;

reg clk, reset; 
top uut (.clk(clk) , .reset(reset));

initial begin 
clk = 0;
reset = 1;
#5;
reset = 0;
#400;
end

always begin 
#5 clk = ~clk;
end

endmodule 