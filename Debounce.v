`timescale 1ns / 1ps

/*
    This Module implements a Debounce 
*/

module Debounce(
    input i_clk,            // Clock
    input i_rst,            // Reset
    input i_signal,         // Signal unstable
    output o_signal         // Signal stable
    );
    
    // Variables
    wire slow_clk_en;
    wire os;
    parameter DEBOUNCE_RESOLUTION = 2'd3;
    reg [DEBOUNCE_RESOLUTION-1:0] Q;
    reg out_signal;
    reg prev_out;
    integer k;
    
    // Initial Conditions
    initial begin
       Q <= 3'd0;          
    end
    
    // Instantiate the Slow Clock module to slow the clock
    Prescaler Prescaler(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .o_en(slow_clk_en)
    );
    
    // Shift the bits along the array
    always @(posedge slow_clk_en)
    begin
        if (i_rst) begin
            Q <= 3'd0;
        end
        else begin
            Q <= {Q[1],Q[0], i_signal};
        end
    end
    
    // One shot
    assign os = Q[0] & Q[1] & ~Q[2];
    
    // Adjust one shot to CPU clock domain
    always @(posedge i_clk)
    begin
        if (os == 1'b1 && prev_out == 1'b0) begin
            out_signal = 1'd1;
            prev_out = 1'b1;
        end
        else if (prev_out == 1'b1) begin
            out_signal = 1'b0;
        end
        if (os == 1'b0) begin
            prev_out = 1'b0;
        end
    end
    
    // Assign the output signal (Stable)
    assign o_signal = out_signal;
    
endmodule 
