module Data_Memory(clk, reset, MemRead, MemWrite, Address, Write_data, Read_data);
input clk, reset, MemRead, MemWrite; // single bit inputs (MemRead/MemWrite are control signals)
input [31:0] Address, Write_data;    // Address and data inputs (32-bits)
output [31:0] Read_data;             // Data output

// Declare data memory
reg [31:0] data_memory[63:0];        // 64 locations, each 32-bit wide
integer k;

// Assign Read_data based on MemRead signal
assign Read_data = (MemRead) ? data_memory[Address] : 32'h00000000; //if MemRead is 1 (load instruction), data_memory[Address] is assigned to Read_data and if MemRead is 0 (store instruction), 32'h0 is assigned


// Sequential logic to handle memory reset and write
always @ (posedge clk) begin
    if (reset == 1'b1) begin
        // Reset all memory locations to 0
        for (k = 0; k < 64; k = k + 1) begin
            data_memory[k] = 32'h00000000;
        end
    end else if (MemWrite) begin
        // Write data to memory when MemWrite is 1
        data_memory[Address] = Write_data;
    end
end
endmodule
