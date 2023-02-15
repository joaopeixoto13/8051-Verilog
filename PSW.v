`timescale 1ns / 1ps

/*
    This modules implements the PSW (Program Status Word)
    
    Details:
        - Write a full byte: (i_op = OP_PSW_WR_BYTE && i_byte = 8'X)
        - Write a bit: (i_op = OP_PSW_WR_BIT && wr_bit = i_byte[0] && addr = i_byte[3:1])
        - Write all flags: (i_op = OP_PSW_WR_FLAGS && i_cy = i_byte[0], i_ac = i_byte[1], i_ov = i_byte[2])
*/

`include "Defines.v"

module PSW(
    input i_clk,                        // Clock
    input i_rst,                        // Reset
    input [7:0] i_byte,                 // Byte to write
    input i_parity,                     // Parity bit 
    input [`SFR_OP_LEN-1:0] i_op,       // Operation to do
    output [7:0] o_psw                  // PSW
    );
    
    // PSW
    reg [7:0] psw;
    
    // Initial state
    initial begin
      psw = 8'b0000_0000;   // cy = 0 && bank = 00
    end
    
    // Update the PSW
    assign o_psw = {psw[7:1], i_parity};
    
    // Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            psw = 8'd0;
        end
       else begin
           if (i_op & `OP_PSW_WR_BYTE) begin
                psw <= i_byte;
           end
           else if (i_op & `OP_PSW_WR_BIT) begin
                if (i_byte[0] == 1'b1) begin
                    psw[i_byte[3:1]] <= 1'b1;
                end
                else begin
                    psw[i_byte[3:1]] <= 1'b0;
                end
           end
           else if (i_op & `OP_PSW_WR_FLAGS) begin
                psw[7] <= i_byte[0];    // Update cy
                psw[6] <= i_byte[1];    // Update ac
                psw[2] <= i_byte[2];    // Update ov
           end 
        end
    end

endmodule