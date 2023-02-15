`timescale 1ns / 1ps

/*
    This modules implements the TCON
    
    Details:
        - Write a full byte: (i_op = OP_TCON_WR_BYTE && i_byte = 8'X)
        - Write a bit: (i_op = OP_TCON_WR_BIT && wr_bit = i_byte[0] && addr = i_byte[3:1])
*/

`include "Defines.v"

module TCON(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_tcon             // TCON
    );
    
    // TCON register
    reg [7:0] tcon;
    
    // Initial state
    initial begin
      tcon <= 8'd0;   
    end 
    
    // Update the output value
    assign o_tcon = tcon;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            tcon <= 8'd0;
        end
        else begin
            if (i_op & `OP_TCON_WR_BYTE) begin
                tcon <= i_byte;
            end
        end
    end

endmodule