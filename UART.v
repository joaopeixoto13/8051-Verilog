`timescale 1ns / 1ps

/*
    This module implements the UART (Universal Asynchronous Receiver Transmitter)
    
*/

module UART(
    input i_clk,                        // Clock source
    input i_rst,                        // Reset
    input i_rx_serial,                  // RX line (Connected to uart_rx)
    input i_rx_en,                      // RX enable flag (Connected to SCON[4])
    input [7:0] i_tx_data,              // TX data to transmit (Connected to SBUF)
    input i_tx_en,                      // TX enable flag (Connected to SCON[3])
    output o_rx_done,                   // RX done flag (Connected to SCON[7])
    output [7:0] o_rx_data,             // RX data received (Connected to SBUF)
    output o_tx_done,                   // TX done flag (Connected to SCON[6])
    output o_tx_serial                  // TX line (Connected to uart_tx)
);

    // Instanciate the UART Receiver
    UART_RX UART_RX(
        .i_clk(i_clk),                  // Clock source
        .i_rst(i_rst),                  // Reset
        .i_rx_serial(i_rx_serial),      // Receive bit (RX line)
        .i_en(i_rx_en),                 // Enable reception
        .o_complete(o_rx_done),         // Complete flag
        .o_data(o_rx_data)              // Output data
    );
    
    // Instanciate the UART Transmitter
    UART_TX UART_TX(
        .i_clk(i_clk),                  // Clock source
        .i_rst(i_rst),                  // Reset
        .i_byte(i_tx_data),             // Byte to transmit
        .i_tx_en(i_tx_en),              // TX Enable
        .o_complete(o_tx_done),         // TX Complete flag
        .o_tx_serial(o_tx_serial)       // Tx line
    );

endmodule