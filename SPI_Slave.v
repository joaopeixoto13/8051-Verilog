`timescale 1ns / 1ps

/*
    This module implements the SPI Slave, used for short-distance communication, primarily in embedded systems.
    SPI devices communicate in full duplex mode using a master-slave architecture.
    In addition to setting the clock frequency, the master must also configure the clock polarity and phase with respect to the data
        - CPOL determines the polarity of the clock
            - CPOL=0 is a clock which idles at 0, and each cycle consists of a pulse of 1. That is, the leading edge is a rising edge, and the trailing edge is a falling edge.
            - CPOL=1 is a clock which idles at 1, and each cycle consists of a pulse of 0. That is, the leading edge is a falling edge, and the trailing edge is a rising edge.
        - CPHA determines the timing (i.e. phase) of the data bits relative to the clock pulses. 
            - CPHA=0 during first edge of the clock signal 
            - CPHA=1 during second edge of the clock signal
    For this purpose, the SPI Slave will run as Mode 0 (CPOL=0 && CPHA=0)
*/

module SPI_Slave(
    input i_scl,        // Master Clock (STM32)
    input i_clk,        // Zybo Clock
    input i_rst,        // Reset
    input i_mosi,       // Master Out Slave In (data output from master) 
    input i_cs,         // Chip/Slave Select 
    output o_miso,      // Master In Slave Out (data output from slave)
    output[7:0] o_data, // Output data
    output o_sync       // Output enable synchronized
    );
    
    // Data
    reg [7:0] data;
    
    // Assign the outputs
    assign o_miso = data[7];    // Data output from slave
    assign o_data = data;       // Data
    
    // Initial Conditions
    initial begin
        data <= 8'd0;
    end
    
    // Instanciate the CDC (Clock Domain Crossing module)
    CDC CDC(
        .i_clk(i_clk),          // Clock source
        .i_rst(i_rst),          // Reset
        .i_signal(i_cs),        // Input signal (not stable)
        .o_signal(o_sync)       // Output signal (stable one)
    );
    
    // SPI State Machine
    always @(posedge i_scl or posedge i_rst)
    begin
        if (i_rst == 1'b1) begin
            data <= 8'h0;
        end
        else if (~i_cs) begin
            data <= {data[6:0], i_mosi};     // Left shift
        end
    end

endmodule