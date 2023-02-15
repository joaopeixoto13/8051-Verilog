`timescale 1ns / 1ps

/*
    This module implements the Zybo LEDS Controller.
    As Zybo-Z7 only have 4 LEDS, this module is responsible for extract the 4 LSB's from a 8bit data to 
    an 4bit value that will be connected to LEDS
        - Pin M14 <==> leds[0]
        - Pin M15 <==> leds[1]
        - Pin G14 <==> leds[2]
        - Pin D18 <==> leds[3]
*/

module LEDS_Controller(
    input i_clk,            // Clock source
    input i_rst,            // Reset 
    input i_en,             // Enable the LEDS
    input [7:0] i_data,     // Data in
    output [3:0] o_leds     // LEDS
    );
    
    // LEDS
    reg [3:0] leds;
    
    // Assign the output
    assign o_leds = leds;
    
    // Initial Conditions
    initial begin
        leds <= 4'd0;
    end
    
     // LEDS Controller
    always @(posedge i_clk or posedge i_rst)
    begin
        if (i_rst) begin
            leds <= 4'd0;
        end
        else if (i_en) begin      
            leds <= i_data[3:0];
        end
    end    
    
endmodule