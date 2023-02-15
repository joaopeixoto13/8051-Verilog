`timescale 1ns / 1ps

/*
    This module implements an One-Shot circuit
*/

module OneShot(
    input i_clk,                // Clock source
    input i_rst,                // Reset 
    input i_signal,             // Input signal
    output o_one_shot           // One shot signal
    );
    
    // Register
    reg signal_dly;
    
    // Initial conditions
    initial begin
        signal_dly <= 1'b0;
    end
    
    // One shot
    always @(posedge i_clk)
    begin
        if(i_rst == 1'b1)
        begin
            signal_dly <= 1'b0;
        end 
        else begin
            signal_dly <= i_signal;
        end
    end
    
    // Assign the output
    assign o_one_shot = i_signal & ~signal_dly;
    
endmodule
