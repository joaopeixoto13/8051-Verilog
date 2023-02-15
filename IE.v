`timescale 1ns / 1ps

/*
    This modules implements the Interrupt Enable Register
    
    Details:
        - Write a full byte: (i_op = OP_IE_WR_BYTE && i_byte = 8'X)
        - Write a bit: (i_op = OP_IE_WR_BIT && wr_bit = i_byte[0] && addr = i_byte[3:1])
*/

`include "Defines.v"

module IE(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_ie               // IE
    );
    
    // IE register
    reg [7:0] ie;
    
    // Initial state
    initial begin
      ie <= 8'b0000_0000;   // EA (bit 7) , ES0 (bit 4) , ET0 (bit 1) , EX0 (bit 0)
    end                      
    
    // Update the output values
    assign o_ie = ie;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            ie <= 8'd0;
        end
        else begin
            if (i_op & `OP_IE_WR_BYTE) begin
                ie <= i_byte;
            end
            else if (i_op & `OP_IE_WR_BIT) begin
                if (i_byte[0] == 1'b1) begin
                    ie[i_byte[3:1]] <= 1'b1;
                end
                else begin
                    ie[i_byte[3:1]] <= 1'b0;
                end
            end
        end
    end

endmodule