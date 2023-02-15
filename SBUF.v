`timescale 1ns / 1ps
`include "Defines.v"

/*
    This modules implements the SBUF
    
    Details:
        - Write a full byte: (i_op = OP_SBUF_WR_BYTE && i_byte = 8'X)
*/

module SBUF(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_sbuf             // SBUF
    );
    
    // SBUF register
    reg [7:0] sbuf;
    
    // Initial state
    initial begin
      sbuf <= 8'd0;   
    end 
    
    // Update the output value
    assign o_sbuf = sbuf;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            sbuf <= 8'd0;
        end
        else begin
            if (i_op & `OP_SBUF_WR_BYTE) begin
                sbuf <= i_byte;
            end
        end
    end

endmodule