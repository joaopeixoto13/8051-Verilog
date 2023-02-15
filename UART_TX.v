`timescale 1ns / 1ps

/*
    This module implements the UART Transmitter.
    This transmitter is able to transmit 8 bits of serial data, one start bit, one stop bit,
    and no parity bit.  When transmit is complete o_complete will be driven high for one clock cycle.
*/


module UART_TX(
    input i_clk,            // Clock source
    input i_rst,            // Reset
    input [7:0] i_byte,     // Byte to transmit
    input i_tx_en,          // Enable
    output o_complete,      // Complete flag
    output o_tx_serial      // Tx line
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
    reg [2:0] state;                // State
    reg [10:0] clock_count;         // Clock count (11 bits because we must count 1085 pulses)
    reg [2:0] bit_index;            // Current bit index
    reg [7:0] tx_data;              // Byte to send
    reg complete;                   // Complete reception flag
    reg tx_serial;                  // TX line
    reg [1:0] enable;               // Enable

    // Initial conditions
    initial begin
        state <= s_idle;
        clock_count <= 11'd0;
        bit_index <= 3'd0;
        tx_data <= 8'd0;
        tx_serial <= 1'b0;
        complete <= 1'b0;
        enable <= 1'b0;
    end
    
    always @(posedge i_clk)
    begin
        enable = {enable[0],i_tx_en};
    end
    
    // TX State Machine
    always @(posedge i_clk)
    begin
        
        if (i_rst == 1'b1) begin
            state <= s_idle;
            clock_count <= 11'd0;
            bit_index <= 3'd0;
            tx_data <= 8'd0;
            complete <= 1'b0;
            tx_serial <= 1'b0;
        end
        else begin
            case (state)
                s_idle: begin
                    tx_serial <= 1'b1;                    // Drive Line High for Idle for start_bit can pull down
                    complete <= 1'd0;
                    clock_count <= 11'd0;
                    bit_index <= 3'd0;
                    if ((enable[0] & ~enable[1]) == 1'b1) begin              // If enable
                        tx_data <= i_byte;     
                        state <= s_start_bit;
                    end
                    else begin
                        state <= s_idle;
                    end
                end
                s_start_bit: begin
                    tx_serial <= 1'b0;                    // Pull down the line
                    
                    if (clock_count < (CLKS_PER_BIT-1))     // Wait for start bit to finish
                    begin
                        clock_count <= clock_count + 1;
                        state <= s_start_bit;
                    end
                    else begin
                        clock_count <= 11'd0;
                        state <= s_data_bits;
                    end
                end
                s_data_bits: begin
                    tx_serial <= tx_data[bit_index];          // Put in the line the bit to send
                    
                    if (clock_count < (CLKS_PER_BIT-1)) begin   // If it is not time to sample the data
                        clock_count <= clock_count + 1;
                        state <= s_data_bits;
                    end
                    else begin                                  // Time to sample the data
                        clock_count <= 11'd0;
                        
                        if (bit_index < 7) begin                // If we have not sent all bits
                            bit_index <= bit_index + 1;
                            state <= s_data_bits;
                        end
                        else begin                              // If we sent all bits
                            bit_index <= 3'd0;
                            state <= s_stop_bit;
                        end
                    end
                end
                s_stop_bit: begin
                    tx_serial <= 1'b1;                        // Pull high the line
                
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
                s_cleanup: begin                                // Stay here 1 clock cycle
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
    assign o_tx_serial = tx_serial;
    
    
endmodule