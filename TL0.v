`timescale 1ns / 1ps

/*
    This modules implements the TL0
    
    Details:
        - Write a full byte: (i_op = OP_TL0_WR_BYTE && i_byte = 8'X)
*/

`include "Defines.v"

module TL0(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_tl0              // TL0
    );
    
    // TL0 register
    reg [7:0] tl0;
    
    // Initial state
    initial begin
      tl0 <= 8'h00;   
    end 
    
    // Update the output value
    assign o_tl0 = tl0;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            tl0 <= 8'd0;
        end
        else begin
            if (i_op & `OP_TL0_WR_BYTE) begin
                tl0 <= i_byte;
            end
        end
    end

endmodule