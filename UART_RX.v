`timescale 1ns / 1ps

/*
    This module implements the UART Receiver.
    This receiver is able to receive 8 bits of serial data, one start bit, one stop bit,
    and no parity bit. When receive is complete o_complete will be driven high for one clock cycle.
*/


module UART_RX(
    input i_clk,            // Clock source
    input i_rst,            // Reset
    input i_rx_serial,      // Receive bit (RX line)
    input i_en,             // Enable bit
    output o_complete,      // Complete flag
    output [7:0] o_data     // Output data
    );
    
    // States definition
    parameter s_idle = 3'b000;
    parameter s_start_bit = 3'b001;
    parameter s_data_bits = 3'b010;
    parameter s_stop_bit = 3'b011;
    parameter s_cleanup = 3'b100;
    
    // Clocks per bit
    // CLKS_PER_BIT = CLOCK_FREQ / UART_FREQ
    // In this case, if the CLOCK_FREQ = 115MHz and UART_FREQ = 115200, we have:
    // ==> CLKS_PER_BIT = 125000000 / 115200 = 1085
    parameter CLKS_PER_BIT = 1085;
    
    // SIMULATION
    // CLK_FREQ = 200MHz
    // BAUDRATE = 20MHz 
    //parameter CLKS_PER_BIT = 10;
    
    // Variables
    reg [1:0] rx_data;              // Bit received (0 not stable and 1 stable)
    reg [2:0] state;                // State
    reg [10:0] clock_count;         // Clock count (11 bits because we must count 1085 pulses)
    reg [2:0] bit_index;            // Current bit index
    reg [7:0] byte;                 // Byte received (output data)
    reg complete;                   // Complete reception flag

    
    // Initial conditions
    initial begin
        state <= s_idle;
        clock_count <= 11'd0;
        bit_index <= 3'd0;
        byte <= 8'd0;
        complete <= 1'b0;
        rx_data <= 2'b11;            // Must be 1 because the Start bit pull the line to 0
    end
    
    // RX State Machine
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            state <= s_idle;
            clock_count <= 11'd0;
            bit_index <= 3'd0;
            byte <= 8'd0;
            complete <= 1'd0;
            rx_data <= 2'b11;            // Must be 1 because the Start bit pull the line to 0
        end
        else begin
            rx_data <= {rx_data[0], i_rx_serial};
            case (state)
                s_idle: begin
                    complete <= 1'd0;
                    clock_count <= 11'd0;
                    bit_index <= 3'd0;
                    if (rx_data[1] == 1'b0 && i_en == 1'b1) begin      // If the Start bit was detected and the reception is enable
                        state <= s_start_bit;
                    end
                    else begin
                        state <= s_idle;
                    end
                end
                s_start_bit: begin
                    if (clock_count == (CLKS_PER_BIT-1)/2)  // Check the middle of start bit to make sure it's still low
                    begin
                        if (rx_data == 1'b0) begin          // If is still low ==> transite to the data_bits state
                            clock_count <= 11'd0;
                            state <= s_data_bits;
                        end
                        else begin
                            state <= s_idle;
                        end
                    end
                    else begin
                        clock_count <= clock_count + 1;
                        state <= s_start_bit;
                    end
                end
                s_data_bits: begin
                    if (clock_count < (CLKS_PER_BIT-1)) begin   // If it is not time to sample the data
                        clock_count <= clock_count + 1;
                        state <= s_data_bits;
                    end
                    else begin                                  // Time to sample the data
                        clock_count <= 11'd0;
                        byte[bit_index] <= rx_data;
                        
                        if (bit_index < 7) begin                // If we have not receive all bits
                            bit_index <= bit_index + 1;
                            state <= s_data_bits;
                        end
                        else begin                              // If we receive all bits
                            bit_index <= 3'd0;
                            state <= s_stop_bit;
                        end
                    end
                end
                s_stop_bit: begin
                    if (clock_count < (CLKS_PER_BIT-1)) begin   // Wait for stop bit to finish
                        clock_count <= clock_count + 1;
                        state <= s_stop_bit;
                    end
                    else begin
                        complete <= 1'b1;                       // Set the complete flag
                        clock_count <= 11'd0;
                        state <= s_cleanup;
                    end
                end
                s_cleanup: begin                                // Stay hewre 1 clock cycle
                    state <= s_idle;
                    complete <= 1'b0;                           // Clear the complete flag for next iteration
                end
                default: begin
                    state <= s_idle;
                end
            endcase
        end
    end
    
    // Assign the outputs
    assign o_complete = complete;
    assign o_data = byte;
    
    
endmodule