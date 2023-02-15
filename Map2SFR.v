`timescale 1ns / 1ps
`include "Defines.v"

/*
    This module is responsible for mapping the SFRs into the RAM
*/

module Map2SFR(
        input [7:0]i_ir,
        input [7:0]i_ram_out,
        input [7:0]i_acc,
        input [7:0]i_psw,
        input [7:0]i_ie,
        input [7:0]i_sp,
        input [7:0]i_tmod,
        input [7:0]i_th0,
        input [7:0]i_tl0,
        input [7:0]i_tcon,
        input [7:0]i_scon,
        input [7:0]i_sbuf,
        input [7:0]i_p2,
        output [7:0]o_out
    );
    
    assign o_out = (i_ir == `ACC_ADDR) ? i_acc :
                   (i_ir == `PSW_ADDR) ? i_psw :
                   (i_ir == `IE_ADDR) ? i_ie :
                   (i_ir == `TH0_ADDR) ? i_th0 :
                   (i_ir == `TL0_ADDR) ? i_tl0 :
                   (i_ir == `TMOD_ADDR) ? i_tmod :
                   (i_ir == `SP_ADDR) ? i_sp : 
                   (i_ir == `TCON_ADDR) ? i_tcon : 
                   (i_ir == `SCON_ADDR) ? i_scon :
                   (i_ir == `SBUF_ADDR) ? i_sbuf : 
                   (i_ir == `P2_ADDR) ? i_p2 : i_ram_out;
   
endmodule