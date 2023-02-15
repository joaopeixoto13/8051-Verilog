`timescale 1ns / 1ps

/*
    This modules implements the Stack Pointer
    
    Details:
        - Write a full byte: (i_op = OP_SP_WR_BYTE && i_byte = 8'X)
*/

`include "Defines.v"

module SP(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_byte,             // Byte to write
    input [`SFR_OP_LEN-1:0]i_op,    // Operation to do
    output [7:0] o_sp               // SP
    );
    
    // SP register (address)
    reg [7:0] sp;
    
    // Initial state
    initial begin
      sp <= 8'h7F;   // By default, SP points to 0x7F (safe RAM location)
    end 
    
    // Update the output value
    assign o_sp = sp;
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            sp <= 8'h7F;
        end
        else begin
            if (i_op & `OP_SP_WR_BYTE) begin
                sp <= i_byte;
            end
            else if (i_op & `OP_SP_PUSH) begin
                sp <= sp - 8'd1;
            end
            else if (i_op & `OP_SP_POP) begin
                sp <= sp + 8'd1;
            end
        end
    end

endmodule