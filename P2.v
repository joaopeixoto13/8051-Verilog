`timescale 1ns / 1ps
`include "Defines.v"

/*
    This modules implements the P2
    
    Details:
        - Write a full byte: (i_op = OP_P2_WR_BYTE && i_byte = 8'X)
*/

module P2(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_p2               // P2
    );
    
    // P2 register
    reg [7:0] p2;
    
    // Initial state
    initial begin
      p2 <= 8'd0;   
    end 
    
    // Update the output value
    assign o_p2 = p2;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            p2 <= 8'd0;
        end
        else begin
            if (i_op & `OP_P2_WR_BYTE) begin
                p2 <= i_byte;
            end
        end
    end

endmodule