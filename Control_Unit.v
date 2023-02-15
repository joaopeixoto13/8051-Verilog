`timescale 1ns / 1ps

/*
    This module implements the 8051 Control Unit 
*/

// Includes all the 8051 defines: Opcodes, Control Signals, SFR's addresses ...
`include "Defines.v"

module Control_Unit(
    input i_clk,                            // Clock 
    input i_rst,                            // Reset
    input [7:0] i_ir,                       // IR 
    input i_int_pend,                       // Interrupt pending
    output [`I_CS_LEN-1:0] o_internal_cs,   // Internal Control Signals (IRload, PCload, JMP and MOV)
    output [`ALU_CS_LEN-1:0] o_alu_cs,      // ALU Control Signals (ADD, SUBB ...)
    output [`RAM_CS_LEN-1:0] o_ram_cs       // RAM Control Signals (READ or WRITE)
    );
    
    // State variable
    reg [4:0] state;
    
    // Auxiliar register to handle the opcode
    reg [7:0] AR;
    
    // Decoded state
    wire [4:0] decoded_state;
    
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
    parameter s_reti1 = 5'd17;      // RETI state 1 (Extract from Stack the PC[15:8])
    parameter s_reti2 = 5'd18;      // RETI state 2 (Wait for RAM to output the SP value)
    parameter s_reti3 = 5'd19;      // RETI state 3 (Extract from Stack the PC[7:0]) 

    // Assign the control signal
    assign o_internal_cs = (state == s_fetch1) ? `IR_PC_LOAD1 :
                           (state == s_fetch2) ? `IR_PC_LOAD2 :
                           (state == s_mov_toA) ? `MOV_TO_A :
                           (state == s_mov_fromA) ? `MOV_FROM_A : 
                           (state == s_jumpC) ? `JMP_C_LOAD :
                           (state == s_jumpNC) ? `JMP_NC_LOAD :
                           (state == s_jumpZ) ? `JMP_Z_LOAD :
                           (state == s_jumpNZ) ? `JMP_NZ_LOAD : 
                           (state == s_reti1) ? `RETI1_CS : 
                           (state == s_reti2) ? `RETI2_CS : 
                           (state == s_reti3) ? `RETI3_CS : 
                           (state == s_start) ? `START_CS : `I_CS_NOP;
    
    // Assign the ALU Control Signal
    assign o_alu_cs = (state == s_add) ? `ALU_CS_ADD :
                      (state == s_subb) ? `ALU_CS_SUB : 
                      (state == s_addc) ? `ALU_CS_ADDC : 
                      (state == s_and) ? `ALU_CS_AND : 
                      (state == s_or) ? `ALU_CS_OR : 
                      (state == s_xor) ? `ALU_CS_XOR : `ALU_CS_NOP; 
    
    // Assign the RAM Control Signals
    assign o_ram_cs = (AR == `ADD_D || AR == `ADDC_D || AR == `SUBB_D || AR == `ANL_D || AR == `ORL_D || AR == `XRL_D || AR == `MOV_D) ? `RAM_CS_RD_D : 
                      (AR == `ADD_R || AR == `ADDC_R || AR == `SUBB_R || AR == `ANL_R || AR == `ORL_R || AR == `XRL_R || AR == `MOV_R) ? `RAM_CS_RD_R : 
                      (AR == `MOV_AD) ? `RAM_CS_WR_D : 
                      (AR == `MOV_AR) ? `RAM_CS_WR_R : 
                      (AR == `RETI) ? `RAM_CS_RD_SP : 
                      (AR == `INTERRUPT) ? `RAM_CS_WR_SP : `RAM_CS_DEFAULT;
                
    // Instanciate the Decoder to decode the Opcode                 
    Decoder Decoder(
        .i_opcode(AR),
        .decoded_state(decoded_state)
    );
    
    // Define the initial conditions
    initial
    begin
        state <= s_start;
    end
    
    // Control Unit State Machine 
    always @ (posedge i_clk)
    begin
        if (i_rst == 1'b1)                      // Reset situation
        begin
            state <= s_start;
        end
        else
        begin
            // If exist a valid interrupt to handle
            if (i_int_pend == 1'b1)            
            begin
                AR = `INTERRUPT;
                state <= s_start;
            end
            // Else
            else begin
                case (state)
                    s_start:                        // 1) Provides an extra clock cycle for the jump instructions 
                        state <= s_fetch1;
                    s_fetch1:                       // 2) Fetch the 1st value (OPCODE) from ROM
                            state <= s_wait;
                    s_wait:                       // 3) Wait for ROM to put the value outside and save the OPCODE
                        begin
                            AR = i_ir;
                            state <= s_fetch2;
                        end     
                    s_fetch2:                       // 4) Fetch the 2st value (OP1) from ROM
                        state <= s_decode;
                    s_decode:                       // 5) Decode the operation 
                       state <= decoded_state;
                    s_reti1:                        
                        state <= s_reti2;
                    s_reti2:
                        state <= s_reti3;
                    default:
                       state <= s_start;  
                endcase
            end
        end
    end     
    
endmodule