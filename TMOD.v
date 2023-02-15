`timescale 1ns / 1ps

/*
    This modules implements the TMOD
    
    Details:
        - Write a full byte: (i_op = OP_TMOD_WR_BYTE && i_byte = 8'X)
*/

`include "Defines.v"

module TMOD(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_tmod             // TMOD
    );
    
    // SP register
    reg [7:0] tmod;
    
    // Initial state
    initial begin
      tmod <= 8'b0000_0001;   
    end 
    
    // Update the output value
    assign o_tmod = tmod;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            tmod <= 8'd0;
        end
        else begin
            if (i_op & `OP_TMOD_WR_BYTE) begin
                tmod <= i_byte;
            end
        end
    end

endmodule