`timescale 1ns / 1ps

/*
    This modules implements the Acumulator
    
    Details:
        - Write a full byte: (i_op = OP_ACC_WR_BYTE && i_byte = 8'X)
        - Write a bit: (i_op = OP_ACC_WR_BIT && wr_bit = i_byte[0] && addr = i_byte[3:1])
*/

`include "Defines.v"

module ACC(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_acc,             // ACC
    output o_parity                 // Parity flag
    );
    
    // Acumulator
    reg [7:0] acc;
    
    // Initial state
    initial begin
      acc <= 8'h10;
    end 
    
    // Update the parity flag
    assign o_parity = ^acc;
    
    // Update the output values
    assign o_acc = acc;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            acc <= 8'd0;
        end
        else begin
            if (i_op & `OP_ACC_WR_BYTE) begin
                acc <= i_byte;
            end
            else if (i_op & `OP_ACC_WR_BIT) begin
                if (i_byte[0] == 1'b1) begin
                    acc[i_byte[3:1]] <= 1'b1;
                end
                else begin
                    acc[i_byte[3:1]] <= 1'b0;
                end
            end
        end
    end

endmodule