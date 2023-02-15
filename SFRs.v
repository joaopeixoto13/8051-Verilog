`timescale 1ns / 1ps
`include "Defines.v"

/*
    This module implements a Register file that handles all SFRs in the 8051
    Register File is a memory space present within the CPU and it's used by the CPU to fetch and hold the data.
    It is faster compared to other memory devices as it is present within the processor.
*/

module SFRs(
    input i_clk,                    // Clock
    input i_rst,                    // Reset
    input [7:0] i_acc,              // ACC to write
    input [7:0] i_psw,              // PSW to write
    input [7:0] i_ie,               // IE to write
    input [7:0] i_sp,               // SP to write
    input [7:0] i_tmod,             // TMOD to write
    input [7:0] i_th0,              // TH0 to write   
    input [7:0] i_tl0,              // TL0 to write 
    input [7:0] i_tcon,             // TCON 
    input [7:0] i_scon,             // SCON
    input [7:0] i_sbuf,             // SBUF  
    input [7:0] i_p2,               // P2
    input [`SFR_OP_LEN-1:0] i_op,   // Operation
    input i_parity,                 // Parity flag to feed the PSW
    output [7:0] o_acc,             // ACC
    output o_parity,                // Parity flag
    output [7:0] o_psw,             // PSW
    output [7:0] o_ie,              // IE
    output [7:0] o_sp,              // Stack Pointer
    output [7:0] o_tmod,            // TMOD
    output [7:0] o_th0,             // TH0
    output [7:0] o_tl0,             // TL0
    output [7:0] o_tcon,            // TCON
    output [7:0] o_scon,            // SCON
    output [7:0] o_sbuf,            // SBUF
    output [7:0] o_p2               // P2
    );
        
    // Instantiate the Acumulator
    ACC ACC(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_acc),
        .i_op(i_op),
        .o_acc(o_acc),
        .o_parity(o_parity)
    );
    
    // Instantiate the PSW
    PSW PSW(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_psw),
        .i_parity(i_parity),
        .i_op(i_op),
        .o_psw(o_psw)
    );
    
    // Instantiate the IE
    IE IE(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_ie),
        .i_op(i_op),
        .o_ie(o_ie)
    ); 
    
    // Instanciate the TMOD
    SP SP(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_sp),
        .i_op(i_op),
        .o_sp(o_sp)
    );        
    
    // Instanciate the TMOD
    TMOD TMOD(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_tmod),
        .i_op(i_op),
        .o_tmod(o_tmod)
    );    
    
    // Instanciate the TMOD
    TH0 TH0(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_th0),
        .i_op(i_op),
        .o_th0(o_th0)
    );       
    
    // Instanciate the TMOD
    TL0 TL0(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_tl0),
        .i_op(i_op),
        .o_tl0(o_tl0)
    );   
    
    // Instanciate the TCON
    TCON TCON(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_tcon),
        .i_op(i_op),
        .o_tcon(o_tcon)
    );  
    
    // Instanciate the SCON
    SCON SCON(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_scon),
        .i_op(i_op),
        .o_scon(o_scon)
    );   
    
    // Instanciate the SBUF
    SBUF SBUF(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_sbuf),
        .i_op(i_op),
        .o_sbuf(o_sbuf)
    );   
    
    // Instanciate the P2
    P2 P2(
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_byte(i_p2),
        .i_op(i_op),
        .o_p2(o_p2)
    );          
    
endmodule