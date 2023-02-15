`timescale 1ns / 1ps

/*
    This modules implements the TH0
    
    Details:
        - Write a full byte: (i_op = OP_TH0_WR_BYTE && i_byte = 8'X)
*/

`include "Defines.v"

module TH0(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_th0              // TH0
    );
    
    // TH0 register
    reg [7:0] th0;
    
    // Initial state
    initial begin
      th0 <= 8'h00;   
    end 
    
    // Update the output value
    assign o_th0 = th0;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            th0 <= 8'd0;
        end
        else begin
            if (i_op & `OP_TH0_WR_BYTE) begin
                th0 <= i_byte;
            end
        end
    end

endmodule