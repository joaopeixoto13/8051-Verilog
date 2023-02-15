`timescale 1ns / 1ps

/*
    The ROM is the dedicated 64KB of program memory space on the 8051
    microcontroller. It contains the binary code with the instructions to be executed.
    The ROM module takes as input the 16 bits of the program counter PC, and
    gives as output the 8-bit data of instruction op-code and operands.
*/

`include "Defines.v"

module ROM(
    input i_clk,                        // Clock
    input [15:0] i_addr,                // Address to read
    output reg [7:0] o_out              // Value read
    );
    
    parameter WIDTH = 8;                            // Datapath width

    parameter MEMSIZE = (1<<16);                    // Memory dimension (32k bytes)
    
    reg [WIDTH-1:0]MEM[0:MEMSIZE-1];                // Principal memory (byte addressble)
    
    integer k;
    
    initial begin
        
        /*
        Arithmetic instructions test: ADD
        
        MEM[0] = `ADD_D;            // ADD A, direct
        MEM[1] = 8'd2;              // RAM[2] = 7
        MEM[2] = `ADD_C;            // ADD A, immediate
        MEM[3] = 8'd127;            // immediate = #127
        MEM[4] = `ADD_R;            // ADD A, Rn
        MEM[5] = 8'd0;              // R0 = 5 (Bank 0 selected on PSW)
        */
        
        /*
        Arithmetic instructions test: ADDC, SUBB
        
        MEM[0] = `ADDC_D;            // ADDC A, direct
        MEM[1] = 8'd2;               // RAM[2] = 7
        MEM[2] = `ADDC_C;            // ADDC A, immediate
        MEM[3] = 8'd127;             // immediate = #127
        MEM[4] = `ADDC_R;            // ADDC A, Rn
        MEM[5] = 8'd0;               // R0 = 2 (Bank 1 selected on PSW)
        MEM[6] = `SUBB_D;            // SUBB A, direct
        MEM[7] = 8'd2;               // RAM[2] = 7
        MEM[8] = `SUBB_C;            // SUBB A, immediate
        MEM[9] = 8'd126;             // immediate = #126
        MEM[10] = `SUBB_R;           // SUBB A, Rn
        MEM[11] = 8'd1;              // R1 = 4 (Bank 1 selected on PSW)
        */
        
        /*
        Logical instructions test: ANL, ORL, XRL
        
        MEM[0] = `ORL_D;             // ORL A, direct
        MEM[1] = 8'd2;               // RAM[2] = 7
        MEM[2] = `ANL_C;             // ANL A, immediate
        MEM[3] = 8'd4;               // immediate = #4
        MEM[4] = `XRL_R;             // XRL A, Rn
        MEM[5] = 8'd1;               // R1 = 2 (Bank 0 selected on PSW)
        */
        
        /*
        Data transfers instructions test: MOV A, XX
        
        MEM[0] = `MOV_C;    // MOV A, #data
        MEM[1] = 8'd3;      // #3
        MEM[2] = `MOV_D;    // MOV A, direct
        MEM[3] = 8'd2;      // RAM[2] = 7
        MEM[4] = `MOV_R;    // MOV A, Rn
        MEM[5] = 8'd1;      // R1 = 2 (Bank 0 selected on PSW)
        */
        
        /*
        Data transfers instructions test: MOV XX, A
        
        MEM[0] = `MOV_AD;   // MOV direct, A
        MEM[1] = 8'd0;      // RAM[0] 
        MEM[2] = `ADD_D;    // ADD A, direct
        MEM[3] = 8'd0;      // RAM[0]
        MEM[4] = `MOV_AR;   // MOV Rn, A
        MEM[5] = 8'd1;      // R1 = A (Bank 0 selected on PSW)
        MEM[6] = `MOV_C;    // MOV A, immediate
        MEM[7] = 8'h7F;     // immediate = 0x7F
        MEM[8] = `MOV_AD;   // MOV direct, A
        MEM[9] = `SP_ADDR;  // SP (Stack Pointer) = A
        */
        
        /*
        Jump instructions test
        
        MEM[0] = `JZ;       // JZ
        MEM[1] = 8'd4;      // rel = #4
        MEM[2] = `ADD_C;    // ADD A, immediate
        MEM[3] = 8'd1;      // immediate = #1 
        MEM[4] = `JC;       // JC
        MEM[5] = 8'd8;      // rel = #8
        MEM[6] = `ADD_C;    // ADD A, immediate
        MEM[7] = 8'd3;      // immediate = #3
        */
        
        /*
        TIMER test
        
        MEM[0] = `MOV_C;            // MOV A, immediate
        MEM[1] = 8'b0000_0010;      // immediate = Timer0 8bits AutoReload
        MEM[2] = `MOV_AD;           // MOV direct, A
        MEM[3] = `TMOD_ADDR;        // TMOD = A
        MEM[4] = `MOV_C;            // MOV A, immediate
        MEM[5] = 8'b0001_0000;      // immediate = Set TR0
        MEM[6] = `MOV_AD;           // MOV direct, A
        MEM[7] = `TCON_ADDR;        // Set the TR0 in TCON
        */
        
        /*
        UART Transmission Test
        
        MEM[0] = `ADD_C;
        MEM[1] = 8'D49;                 // A = 49 (ASCII 1)
        MEM[2] = `MOV_AD;
        MEM[3] = `SBUF_ADDR;            // SBUF = 49
        MEM[4] = `MOV_C;                // MOV A, #data
        MEM[5] = 8'b0000_1000;          // Enable UART Transmission (SCON[3])
        MEM[6] = `MOV_AD;
        MEM[7] = `SCON_ADDR;            // SCON = 8 ==> Start Transmission
        */
        
        /* 
        UART Reception Test
        
        MEM[0] = `MOV_C;                // MOV A, #data
        MEM[1] = 8'b0001_0000;          // Enable UART Reception (SCON[4])
        MEM[2] = `MOV_AD;
        MEM[3] = `SCON_ADDR;            // SCON = 16 ==> Start Reception
        */
        
        for (k = 0; k < MEMSIZE; k = k + 1)
        begin
            MEM[k] = 8'b0;                          // Clear the memory
        end  
        
        /*
        TIMER 0 Interrupt Test
       
        MEM[0] = `MOV_AD;           // MOV direct, A
        MEM[1] = `TCON_ADDR;        // TCON = A

        // Timer 0 IRQ
        MEM[8'h0b] = `ORL_C;        // ORL A, immediate            
        MEM[8'h0c] = 8'h1;          // immediate = #1
        MEM[8'h0d] = `ADD_C;        // ADD A, immediate
        MEM[8'h0e] = 8'd2;          // immediate = #2
        MEM[8'h0f] = `RETI;         // RETI
        MEM[8'h10] = 8'd0;
        */
        
        
        /*
        TIMER 0 Interrupt Test + LEDS (Timer-1s.bit)
        
        
        make sure that 
            - IE=8'b1000_0010
            - TH0=TL0=0
            - TMOD=8'b0000_0001
            - o_leds=counter_leds
        
        MEM[0] = `MOV_AD;           // MOV direct, A
        MEM[1] = `TCON_ADDR;        // TCON = A
        MEM[2] = `MOV_C;            // MOV A, immediate
        MEM[3] = 8'd0;              // immediate = #0
        MEM[4] = `JNC;              // JNC rel
        MEM[5] = 8'd4;              // rel = #4

        // Timer 0 ISR
        MEM[8'h0b] = `RETI;         // RETI
        MEM[8'h0c] = 8'd0;
        */
        

        //UART Reception + Transmission with Interrupts (FullUART_Int.bit)       
        MEM[0] = `MOV_C;                // MOV A, immediate
        MEM[1] = 8'b1001_0000;          // immediate = Enable EA and ES0
        MEM[2] = `MOV_AD;               // MOV direct, A
        MEM[3] = `IE_ADDR;              // IE = Set EA and ES0
        MEM[4] = `MOV_C;                // MOV A, immediate
        MEM[5] = 8'b0001_0000;          // immediate = Enable UART Reception (SCON[4])
        MEM[6] = `MOV_AD;               // MOV direct, A
        MEM[7] = `SCON_ADDR;            // SCON = Start Reception
        MEM[8] = `JNC;                  // JNC rel
        MEM[9] = 8'd8;                  // rel = #8
        
        // UART ISR
        
        MEM[8'h23] = `MOV_C;            // MOV A, immediate
        MEM[8'h24] = 8'd1;              // immediate = #1
        MEM[8'h25] = `ANL_D;            // ANL A, direct
        MEM[8'h26] = `SCON_ADDR;        // direct = SCON
        MEM[8'h27] = `JNZ;              // JNZ rel ==> Received data 
        MEM[8'h28] = 8'h50;             // rel = 8'h50
        MEM[8'h29] = `MOV_C;            // MOV A, immediate
        MEM[8'h2A] = 8'b0001_0000;      // immediate = Clear UART transmission flag and Clear enable Transmission
        MEM[8'h2B] = `MOV_AD;           // MOV direct, A
        MEM[8'h2C] = `SCON_ADDR;        // SCON = A
        MEM[8'h2D] = `RETI;             // RETI
        MEM[8'h2E] = 8'd0;  
        
        MEM[8'h50] = `MOV_D;            // MOV A, direct
        MEM[8'h51] = `SBUF_ADDR;        // A = SBUF (data received)
        MEM[8'h52] = `MOV_AD;           // MOV direct, A
        MEM[8'h53] = `P2_ADDR;          // P2 = A (data received)
        MEM[8'h54] = `MOV_C;            // MOV A, immediate
        MEM[8'h55] = 8'b0001_1000;      // immediate = Clear UART reception flag and enable Transmission
        MEM[8'h56] = `MOV_AD;           // MOV direct, A
        MEM[8'h57] = `SCON_ADDR;        // SCON = A
        MEM[8'h58] = `RETI;             // RETI
        MEM[8'h59] = 8'd0;
        
        /*
        // External Interrupt 0 
        MEM[0] = `MOV_C;                // MOV A, immediate
        MEM[1] = 8'b1000_0001;          // immediate = Enable EA and EX0
        MEM[2] = `MOV_AD;               // MOV direct, A
        MEM[3] = `IE_ADDR;              // IE = Set EA and ES0
        MEM[4] = `MOV_C;                // MOV A, immediate
        MEM[5] = 8'd0;                  // immediate = #0
        MEM[6] = `JNC;                  // JNC rel
        MEM[7] = 8'd6;                  // rel = #8
        
        // EX0
        MEM[8'h13] = `ADD_C;            // ADD A, immediate
        MEM[8'h14] = 8'd1;              // immediate = #1 
        MEM[8'h15] = `MOV_AD;           // MOV direct, A
        MEM[8'h16] = `P2_ADDR;          // MOV P2, A
        MEM[8'h17] = `RETI;             // RETI
        MEM[8'h18] = 8'd0;
        */
        
    end
    
    always @(posedge i_clk)
    begin
        o_out = MEM[i_addr];                    // Read the content from memory 
    end
    
       
endmodule