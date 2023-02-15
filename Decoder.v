`timescale 1ns / 1ps

/*
    This module is responsible for decode the operation 
*/

`include "Defines.v"

module Decoder(
    input [7:0] i_opcode,           // Opcode to decode
    output [4:0] decoded_state      // Decoded state
    );
    
    // States definition
    parameter s_start = 5'd0;       // Start state and provides an extra clock cycle for the Jump instructions (because they change the PC on the execution stage)
    parameter s_fetch1 = 5'd1;      // Fetch the first value from ROM
    parameter s_wait = 5'd2;      // Wait for ROM to put the value outside
    parameter s_fetch2 = 5'd3;      // Fetch the second value from ROM
    parameter s_decode = 5'd4;      // Extract the information from IR (opcode instruction)
    parameter s_add = 5'd5;         // ADD state (ADD A Rn , ADD A direct , ADD A immediate)
    parameter s_subb = 5'd6;        // SUBB state (SUBB A Rn , SUBB A direct , SUBB A immediate)
    parameter s_addc = 5'd7;        // ADDC state (ADDC A Rn , ADDC A direct , ADDC A immediate)
    parameter s_and = 5'd8;         // AND state (ANL A Rn , ANL A direct , ANL A immediate)
    parameter s_or = 5'd9;          // OR state (ORl A Rn , ORl A direct , ORl A immediate)
    parameter s_xor = 5'd10;        // XOR state (XRL A Rn , XRL A direct , XRL A immediate)
    parameter s_mov_toA = 5'd11;    // MOV to A state (MOV A Rn, MOV A direct, MOV A immediate)
    parameter s_mov_fromA = 5'd12;  // MOV from A state (MOV direct A, MOV Rn A)
    parameter s_jumpC = 5'd13;      // JC state
    parameter s_jumpNC = 5'd14;     // JNC state
    parameter s_jumpZ = 5'd15;      // JZ state
    parameter s_jumpNZ = 5'd16;     // JNZ state
    parameter s_reti1 = 5'd17;      // RETI state 1
    parameter s_reti2 = 5'd18;      // RETI state 2
    parameter s_reti3 = 5'd19;      // RETI state 3
    parameter s_prepInt1 = 5'd20;
    
   assign decoded_state = (i_opcode == `ADD_R || i_opcode == `ADD_D || i_opcode == `ADD_C) ? s_add :
                          (i_opcode == `SUBB_R || i_opcode == `SUBB_D || i_opcode == `SUBB_C) ? s_subb :
                          (i_opcode == `ADDC_R || i_opcode == `ADDC_D || i_opcode == `ADDC_C) ? s_addc : 
                          (i_opcode == `ANL_R || i_opcode == `ANL_D || i_opcode == `ANL_C) ? s_and :
                          (i_opcode == `ORL_R || i_opcode == `ORL_D || i_opcode == `ORL_C) ? s_or : 
                          (i_opcode == `XRL_R || i_opcode == `XRL_D || i_opcode == `XRL_C) ? s_xor : 
                          (i_opcode == `MOV_R || i_opcode == `MOV_D || i_opcode == `MOV_C) ? s_mov_toA :
                          (i_opcode == `MOV_AR || i_opcode == `MOV_AD) ? s_mov_fromA :
                          (i_opcode == `JC) ? s_jumpC :
                          (i_opcode == `JNC) ? s_jumpNC :
                          (i_opcode == `JZ) ? s_jumpZ : 
                          (i_opcode == `JNZ) ? s_jumpNZ : 
                          (i_opcode == `RETI) ? s_reti1 : s_start;                                                                                                                      
    
endmodule