`timescale 1ns / 1ps

module Prescaler(
    input i_clk,        // Higher Clock source
    input i_rst,        // Reset
    output o_en         // Slower Clock enable
    );
    
    
    reg[32:0] counter = 32'd0;
    parameter DIVISOR = 32'd25000000; // o_em -> 5 Hz
    initial begin
        counter <= 32'd0;
    end
    
    always @(posedge i_clk)
    begin
     counter <= counter + 32'd1;
     if(counter>=(DIVISOR-1))
        counter <= 32'd0;
    end
    
    assign o_en = (counter < DIVISOR/2) ? 1'b1 : 1'b0;
endmodule

