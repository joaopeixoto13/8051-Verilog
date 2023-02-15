`timescale 1ns / 1ps

/*
    This module implements the 8051
    
    #######################
    ## Brief description ##
    #######################
    
    This version of 8051 ISA (Instruction Set Architecture) is composed by 27 instructions:
        - 9 Arithmetic instructions
        - 9 Logic instructions
        - 5 Data transfer instructions
        - 5 jump instructions
    Also, this version has:
        - 1 Timer with four modes:
            - MODE 1: 13bits timer/counter
            - MODE 2: 16bits timer/counter
            - MODE 3: 8 bits timer/counter with Autoreload
            - MODE 4: Same as above but with same differences
        - SPI Slave interface
        - UART
        - 4 Interrupts 
            - Triggered by Timer when overflow
            - Triggered by an external event
            - Triggered by a reception of UART
            - Triggered by a transmission of UART
     
     
    ########################
    ## Detail description ##
    ########################
    
    System Overview
    - 8051
        - CPU                   System control unit that manage the system state machine and send signals to Datapath
            - Decoder           Responsible to decode the opcode 
        - Datapath              Performs and executes all data operations 
            - ROM               Implement the program code memory
            - RAM               Implement the RAM memory
            - SFRs              Pack all the system SFRs
                - ACC           Acumulator
                - PSW           Program Status Word
                - IE            Interrupt Enable
                - SP            Stack Pointer
                - TMOD          Timer Mode
                - TH0           Timer High
                - TL0           Timer Low
                - TCON          Timer Control
                - SCON          Serial I/O Control
                - SBUF          Serial Buffer
            - Map2SFR           Map the SFRs in the RAM
            - ALU               Aritmetic and Logic Unit, responsible to perform all aritmetic and logic operations
            - Timer             Implements the Timer
            - UART              Implements the UART (Universal Asynchronous Receiver Transmitter)
                - UART_RX       Implements the UART Receiver
                - UART_TX       Implements the UART Transmitter
            - Interrupt         Implements the system Interrupts
                - EXT0_ISR      Handles the External 0 Interrupt 
                - TIM0_ISR      Handles the Timer 0 Interrupt 
                - UART_ISR      Handles the UART Interrupt
            - OneShot           Implements a One Shot circuit
         - SPI_Slave            Implements the SPI Slave
            - CDC               Clock Domain Crossing
         - LEDS_Controller      Implements the Zybo-Z7 LEDS Controller
         
*/

`include "Defines.v"

module Top(
    input i_clk,                        // Zybo Clock
    input i_scl,                        // SPI Master Clock (STM32)
    input i_rst,                        // Reset
    input i_mosi,                       // Master Out Slave In (data output from master)  
    input i_cs,                         // Chip/Slave Select
    input i_rx,                         // RX line
    input i_button,                     // Button (Connected to External Interrupt 0)
    output o_miso,                      // Master In Slave Out (data output from slave)
    output o_tx,                        // TX line
    output [3:0] o_leds                 // LEDS
    );
    
    wire [`I_CS_LEN-1:0] internal_cs;   // Internal Control Signals
    wire [`ALU_CS_LEN-1:0] alu_cs;      // ALU Control Signals
    wire [`RAM_CS_LEN-1:0] ram_cs;      // RAM Control Signals
    wire [7:0] ir;                      // IR (Instruction Register)
    wire int_pend;                      // Interrupt pending flag
    wire button;
    
    // Instanciate the Control Unit
    Control_Unit Control_Unit(
        .i_clk(i_clk),                  // Zybo Clock                   
        .i_rst(i_rst),                  // Reset
        .i_ir(ir),                      // Instruction Register
        .i_int_pend(int_pend),          // Interrupt pending flag
        .o_internal_cs(internal_cs),    // Internal Control Signals
        .o_alu_cs(alu_cs),              // ALU Control Signals
        .o_ram_cs(ram_cs)               // RAM Control Signals
    ); 
    
    wire [7:0] o_leds_in; 
    
    // Instanciate the Datapath
    Datapath Datapath(
        .i_clk(i_clk),                  // Zybo Clock            
        .i_rst(i_rst),                  // Reset
        .i_internal_cs(internal_cs),    // Internal Control Signals
        .i_alu_cs(alu_cs),              // ALU Control Signals
        .i_ram_cs(ram_cs),              // RAM Control Signals
        .i_uart_rx(i_rx),               // RX line
        .i_button(button),              // Button connected to EXT0
        .o_uart_tx(o_tx),               // TX line
        .o_ir(ir),                      // Instruction Register
        .o_int_pend(int_pend),           // Interrupt pending flag
        .o_leds(o_leds_in)
    );   
    
    wire [7:0] spi_data;                // SPI output data
    wire spi_leds_en;                   // Enable the LEDS to show the value received
    
    // Instanciate the SPI_Slave
    SPI_Slave SPI_Slave(
        .i_scl(i_scl),                  // Master Clock (STM32)
        .i_clk(i_clk),                  // Zybo Clock
        .i_rst(i_rst),                  // Reset
        .i_mosi(i_mosi),                // Master Out Slave In (data output from master) 
        .i_cs(i_cs),                    // Chip/Slave Select 
        .o_miso(o_miso),                // Master In Slave Out (data output from slave)
        .o_data(spi_data),              // SPI output data
        .o_sync(spi_leds_en)            // Output Clock Domain Crossing
    );
    
    // Instanciate the LEDS Controller
    LEDS_Controller LEDS_Controller(
        .i_clk(i_clk),                  // Zybo Clock
        .i_rst(i_rst),                  // Reset 
        .i_en(1'b1),                    // Enable the LEDS
        .i_data(o_leds_in),             // SPI data
        .o_leds(o_leds)                 // LEDS mapped in Zybo-Z7 constraints file
    );
    
    // Instantiate the Debounce module to debounce the Button connected to EXT0
    Debounce EXT0_Debounce (
        .i_clk(i_clk),                  // Clock
        .i_rst(i_rst),                  // Reset
        .i_signal(i_button),            // Signal unstable
        .o_signal(button)               // Signal stable
    );    
    
endmodule