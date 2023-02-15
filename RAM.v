`timescale 1ns / 1ps

/*
The 8051 has an internal Data Memory (internal RAM) of 256 bytes. The
internal RAM is divided into 2 blocks: the first 128 byte block is the general
purpose RAM, and a second part composed by the indirect addressing only.
However, from:
- 00h to 1Fh the RAM stores the 4 banks of 8 bytes each (R0-R7)
- 20h to 2Fh bit addressable
- 30h to 7Fh Generic purpose (with stack)
- 80h to FFh Indirect Addressing Only
*/

/*
This module performs the following tasks:
- Clears the memory when reset signal is asserted
- Performs a synchronous read or write from / to the addressed memory location
- If bit data is requested, reads or writes to the addressed bit number of the addressed memory location
*/

`include "Defines.v"

module RAM(
    input i_clk,                    // Clock
    input [7:0] i_addr,             // Address to read or address to write 
    input [7:0] i_wr_byte,          // Data to write
    input i_wr_bit,                 // Bit to write
    input [`RAM_OP_LEN-1:0] i_op,   // Operation to do
    output reg [7:0] o_byte,        // Out byte
    output reg o_bit                // Out bit
    );
    
    parameter WIDTH = 8;                            // Datapath width
    
    parameter MEMSIZE = (1<<8);                     // Memory dimension (256 bytes)
    
    reg [WIDTH-1:0]MEM[0:MEMSIZE-1];                // Memory
    
    integer k;
    
    wire [7:0] data;
    wire [2:0] bit_data; 

    // Extract the bit adrress
    assign bit_data = i_addr[2:0];
    
    // Extract the address
    assign data = {3'b000, i_addr[7:3]};                       
    
    initial begin
        MEM[0] = 8'd5;
        MEM[1] = 8'd1;
        MEM[2] = 8'd7;
        for (k = 3; k < MEMSIZE; k = k + 1)
        begin
            MEM[k] = 8'b0;                             // Clear the memory
        end
        MEM[8] = 8'd2;  // R0 bank1 = 2  
        MEM[9] = 8'd4;  // R1 bank1 = 4
        MEM[10] = 8'd1; // R2 bank1 = 1
        
        //MEM[8'h7F] = 8'd0;    // PC[15:8]
        //MEM[8'h7E] = 8'd1;    // PC[7:0]
    end
    
    always @(posedge i_clk)
    begin
        o_byte = MEM[i_addr];
        case (i_op)
            `OP_RAM_WR_BYTE: 
                 MEM[i_addr] = i_wr_byte;
            `OP_RAM_WR_BIT: 
                if (i_wr_bit == 1'b1) begin
                    MEM[data] = MEM[data] | (8'd1<<bit_data);
                end
                else begin
                    MEM[data] = MEM[data] & ~(8'd1<<bit_data);
                end
            `OP_RAM_RD_BIT:
                o_bit = (MEM[data]>>bit_data) & 8'd1;
        endcase 
    end
    
endmodule