`timescale 1ns / 1ps

/*
    This module implements the 8051 Datapath
*/

`include "Defines.v"

module Datapath(
    input i_clk,                                // Clock 
    input i_rst,                                // Reset
    input [`I_CS_LEN-1:0] i_internal_cs,        // Internal Control Signals
    input [`ALU_CS_LEN-1:0] i_alu_cs,           // ALU Control Signals
    input [`RAM_CS_LEN-1:0] i_ram_cs,           // RAM Control Signals
    input i_uart_rx,                            // UART RX Line
    input i_button,                             // Button
    output o_uart_tx,                           // UART TX Line
    output [7:0] o_ir,                          // Opcode / OP1
    output o_int_pend,                          // Interrupt to process
    output [7:0] o_leds                         // LED's
    );
    
    // Global parameters
    parameter WIDTH = 8;                        // RAM width
    parameter ROM_WIDTH = 16;                   // ROM wodth
    
    // ISA Storage Elements
    reg [ROM_WIDTH-1:0]PC;                      // Program Counter                           
    reg [WIDTH-1:0]IR;                          // Instruction Register
    
    reg [ROM_WIDTH-1:0]PC_save;                 // Auxiliar variable to help to store the PC
    wire [WIDTH-1:0]PC_aux;                     // Auxiliar variable to help the context saving in stack
    
    // ROM variables                            
    wire [WIDTH-1:0]rom_out;                    // ROM output byte
    wire [ROM_WIDTH-1:0]rom_addr;               // RAM address
    
    // RAM variables
    wire [WIDTH-1:0]ram_out;                    // RAM output byte
    wire [WIDTH-1:0]ram_addr;                   // RAM address
    wire [WIDTH-1:0]ram_data;                   // RAM write/read byte
    wire ram_bit;                               // RAM write/read bit (Bit addressable)
    wire [`RAM_OP_LEN-1:0]ram_op;               // RAM operations
    wire ram_out_bit;                           // RAM output bit
    
    // SFRs variables
    wire [`SFR_OP_LEN-1:0] sfr_op;              // SFR operation
    wire [`SFR_OP_LEN-1:0] sfr_op_aux1;         // SFR operation auxiliar 1
    wire [`SFR_OP_LEN-1:0] sfr_op_aux2;         // SFR operation auxiliar 2
    wire [`SFR_OP_LEN-1:0] sfr_op_aux3;         // SFR operation auxiliar 3
    wire [7:0] sfr_wr_acc;                      // SFR write ACC
    wire [7:0] sfr_wr_psw;                      // SFR write PSW
    wire [7:0] sfr_wr_ie;                       // SFR write IE
    wire [7:0] sfr_wr_sp;                       // SFR write SP
    wire [7:0] sfr_wr_tmod;                     // SFR write TMOD
    wire [7:0] sfr_wr_th0;                      // SFR write TH0
    wire [7:0] sfr_wr_tl0;                      // SFR write TL0
    wire [7:0] sfr_wr_tcon;                     // SFR write TCON
    wire [7:0] sfr_wr_scon;                     // SFR write SCON
    wire [7:0] sfr_wr_sbuf;                     // SFR write SBUF
    wire [7:0] sfr_wr_p2;                       // SFR write P2
    wire [7:0] acc;                             // ACC (Acumulator)
    wire parity;                                // Parity flag
    wire [7:0] psw;                             // PSW (Program Status Word)
    wire [1:0] bank;                            // Bank selected
    wire [7:0] ie;                              // IE (Interrupt Enable)
    wire [7:0] sp;                              // SP (Stack Pointer)
    wire [7:0] tmod;                            // TMOD (Timer Mode)
    wire [7:0] th0;                             // TH0 (Timer High)
    wire [7:0] tl0;                             // TL0 (Timer Low)
    wire [7:0] tcon;                            // TCON (Timer Control)
    wire [7:0] scon;                            // SCON (Serial I/O Control)
    wire [7:0] sbuf;                            // SBUF (Serial I/O Buffer)   
    wire [7:0] p2;                              // P2    
    
    // Map2SFR variable
    wire [7:0]addr_mapped;                      // Address mapped 
    
    // ALU variables
    wire [`ALU_CS_LEN-1:0] alu_op;              // ALU operation
    wire [7:0] alu_src1;                        // ALU source #1
    wire [7:0] alu_src2;                        // ALU source #2
    wire alu_srcC;                              // ALU source carry
    wire alu_srcAc;                             // ALU source auxiliary carry
    wire [7:0] alu_des;                         // ALU destination (handles the final result)
    wire alu_desC;                              // ALU destination carry
    wire alu_desAc;                             // ALU destination auxiliary carry
    wire alu_desOv;                             // ALU destination overflow
    
    // Timer 0 variables
    wire [7:0] th0_tim_out;                     // Output of TH0 in Timer module
    wire [7:0] tl0_tim_out;                     // Output of TL0 in Timer module  
    wire tf0_tim_out;                           // Output of overflow flag in Timer module
    wire tim0_en;                               // Timer0/Counter0 is running 
    
    // UART variables
    wire [7:0] rx_data;                         // UART RX data received 
    wire rx_done;                               // UART reception done
    wire tx_done;                               // UART transmission done
    
    // External Interrupt 0 variables
    wire ext0_int_en;                           // External 0 Interrupt enable
    reg ext0_int_in;                            // External 0 Interrupt in progress flag (No nested interrupts)
    reg ext0_int_req;                           // External 0 Interrupt request
    wire ext0_int_status;                       // External 0 Interrupt status (1 if the interrupt is eligible to execute)
    wire ext0_int_one_pulse;                    // External 0 Interrupt one pulse
    wire ext0_button;                           // Button pressed
    
    // Timer 0 Interrupt variables
    wire tim0_int_en;                           // Timer 0 Interrupt enable
    reg tim0_int_in;                            // Timer 0 Interrupt in progress flag (No nested interrupts)
    reg tim0_int_req;                           // Timer 0 Interrupt request pending
    wire tim0_int_status;                       // Timer 0 Interrupt status (1 if the interrupt is eligible to execute)
    wire tim0_int_one_pulse;                    // Timer 0 Interrupt one pulse
    
    // Serial I/O Interrupt variables
    wire uart_int_en;                           // Serial I/O Interrupt enable
    reg uart_int_in;                            // Serial I/O Interrupt in progress flag (No nested interrupts)
    reg uart_int_req;                           // Serial I/O Interrupt request
    wire uart_int_status;                       // Serial I/O Interrupt status (1 if the interrupt is eligible to execute)    
    wire uart_int_one_pulse;                    // Serial I/O Interrupt one pulse
    
    // Instanciate the ROM
    ROM ROM(
        .i_clk(i_clk),                          // Clock
        .i_addr(rom_addr),                      // Address to read 
        .o_out(rom_out)                         // Output value
    );
    
    // Instanciate the RAM
    RAM RAM(
        .i_clk(i_clk),                          // Clock
        .i_addr(ram_addr),                      // Address to read or address to write 
        .i_wr_byte(ram_data),                   // Data to write
        .i_wr_bit(ram_bit),                     // Bit to write
        .i_op(ram_op),                          // Operation to do
        .o_byte(ram_out),                       // Output byte
        .o_bit(ram_out_bit)                     // Output bit
    );
    
    // Instanciate the Register File (SFRs) 
    SFRs SFRs(
        .i_clk(i_clk),                          // Clock
        .i_rst(i_rst),                          // Reset
        .i_acc(sfr_wr_acc),                     // ACC to write
        .i_psw(sfr_wr_psw),                     // PSW to write
        .i_ie(sfr_wr_ie),                       // IE to write
        .i_sp(sfr_wr_sp),                       // Stack Pointer to write
        .i_tmod(sfr_wr_tmod),                   // TMOD to write
        .i_th0(sfr_wr_th0),                     // TH0 to write
        .i_tl0(sfr_wr_tl0),                     // TL0 to write
        .i_tcon(sfr_wr_tcon),                   // TCON to write
        .i_scon(sfr_wr_scon),                   // SCON to write
        .i_sbuf(sfr_wr_sbuf),                   // SBUF to write
        .i_p2(sfr_wr_p2),                       // P2 to write
        .i_op(sfr_op),                          // Operation
        .i_parity(parity),                      // Parity flag to feed the PSW
        .o_acc(acc),                            // ACC
        .o_parity(parity),                      // Parity flag
        .o_psw(psw),                            // PSW
        .o_ie(ie),                              // IE (Interrupt Enable)
        .o_sp(sp),                              // Stack Pointer
        .o_tmod(tmod),                          // TMOD
        .o_th0(th0),                            // TH0
        .o_tl0(tl0),                            // TL0
        .o_tcon(tcon),                          // TCON
        .o_scon(scon),                          // SCON
        .o_sbuf(sbuf),                          // SBUF
        .o_p2(p2)                               // P2
    );
    
    // Map the SFRs into the RAM
    Map2SFR Map2SFR(
        .i_ir(IR),                              // Real Address
        .i_ram_out(ram_out),                    // RAM value in the real address
        .i_acc(acc),                            // ACC 
        .i_psw(psw),                            // PSW
        .i_ie(ie),                              // IE
        .i_sp(sp),                              // SP
        .i_tmod(tmod),                          // TMOD
        .i_th0(th0),                            // TH0
        .i_tl0(tl0),                            // TL0
        .i_tcon(tcon),                          // TCON
        .i_scon(scon),                          // SCON
        .i_sbuf(sbuf),                          // SBUF
        .i_p2(p2),                              // P2
        .o_out(addr_mapped)                     // Address mapped
    );
    
    // Instanciate the ALU
    ALU ALU(
        .i_operation(alu_op),                   // ALU operation
        .i_src1(alu_src1),                      // ALU source #1
        .i_src2(alu_src2),                      // ALU source #2
        .i_srcC(alu_srcC),                      // ALU source carry
        .i_srcAc(alu_srcAc),                    // ALU source auxiliary carry
        .o_des1(alu_des),                       // ALU destination value
        .o_desC(alu_desC),                      // ALU destination carry
        .o_desAc(alu_desAc),                    // ALU destination auxiliary carry
        .o_desOv(alu_desOv)                     // ALU destination overflow
    );
    
    // Instanciate the UART
    UART UART (
        .i_clk(i_clk),                          // Clock 
        .i_rst(i_rst),                          // Reset
        .i_rx_serial(i_uart_rx),                // RX line
        .i_rx_en(scon[4]),                      // RX enable
        .i_tx_data(sbuf),                       // TX data to transmit
        .i_tx_en(scon[3]),                      // TX enable flag
        .o_rx_done(rx_done),                    // RX done flag
        .o_rx_data(rx_data),                    // RX data received
        .o_tx_done(tx_done),                    // TX done flag
        .o_tx_serial(o_uart_tx)                 // TX line
    );
    
    // Instanciate the Timer 0
    Timer Timer0 (
        .i_clk(i_clk),                          // Clock        
        .i_rst(i_rst),                          // Reset       
        .i_tmod(tmod),                          // TMOD        
        .i_thx(th0),                            // TH0     
        .i_tlx(tl0),                            // TL0   
        .i_trx(tcon[4]),                        // TR0         
        .i_intx(ie[1]),                         // INT0      
        .o_thx(th0_tim_out),                    // TH0 updated 
        .o_tlx(tl0_tim_out),                    // TL0 updated 
        .o_tfx(tf0_tim_out),                    // TF0 (overflow flag)
        .o_en(tim0_en)                          // Timer is running    
    );
    
    // Instanciate the External Interrupt 0
    Interrupt EXT0_ISR (
        .i_clk(i_clk),                          // Clock
        .i_rst(i_rst),                          // Reset
        .i_en(ext0_int_en),                     // External 0 Interrupt enable
        .i_int(ext0_int_in),                    // External 0 Interrupt in progress (No nested interrupts)
        .i_int_req(ext0_button),                // External 0 Interrupt request (Pressed by a button)
        .o_int(ext0_int_status)                 // External 0 Interrupt status (1 if the interrupt is eligible to execute)
    );
    
    // Instanciate the Timer 0 Interrupt
    Interrupt TIM0_ISR (
        .i_clk(i_clk),                          // Clock
        .i_rst(i_rst),                          // Reset
        .i_en(tim0_int_en),                     // Timer 0 Interrupt enable
        .i_int(tim0_int_in),                    // Timer 0 Interrupt in progress (No nested interrupts)
        .i_int_req(tf0_tim_out),                // Timer 0 Interrupt request
        .o_int(tim0_int_status)                 // Timer 0 Interrupt status (1 if the interrupt is eligible to execute)
    );
    
    // Instanciate the Serial I/O Interrupt
    Interrupt UART_ISR (
        .i_clk(i_clk),                          // Clock
        .i_rst(i_rst),                          // Reset
        .i_en(uart_int_en),                     // Serial I/O Interrupt enable
        .i_int(uart_int_in),                    // Serial I/O Interrupt in progress (No nested interrupts)
        .i_int_req(rx_done | tx_done),          // Serial I/O Interrupt request
        .o_int(uart_int_status)                 // Serial I/O Interrupt status (1 if the interrupt is eligible to execute)
    );
    
    // Instanciate the One Shot circuit to Timer 0 Interrupt
    // This signal is import to allert the CPU that a valid interrupt is pending
    OneShot OneShot_TIM0_INT (
        .i_clk(i_clk),                          // Clock
        .i_rst(i_rst),                          // Reset 
        .i_signal(tim0_int_in),                 // Input signal
        .o_one_shot(tim0_int_one_pulse)         // One shot signal
    );
    
    // Instanciate the One Shot circuit to External 0 Interrupt
    OneShot OneShot_EXT0_INT (
        .i_clk(i_clk),                          // Clock 
        .i_rst(i_rst),                          // Reset 
        .i_signal(ext0_int_in),                 // Input signal
        .o_one_shot(ext0_int_one_pulse)         // One shot signal
    );
    
    // Instanciate the One Shot circuit to Uart Interrupt
    OneShot OneShot_UART_INT (
        .i_clk(i_clk),                          // Clock 
        .i_rst(i_rst),                          // Reset 
        .i_signal(uart_int_in),                 // Input signal
        .o_one_shot(uart_int_one_pulse)         // One shot signal
    );
    
    // Assign the ROM adress 
    assign rom_addr = PC;
    
    // Assign IR
    assign o_ir = IR;
    
    // Assign the Bank
    assign bank = 2*psw[4] + psw[3];
    
    // Assign the ALU operation
    assign alu_op = i_alu_cs;
    
    // Assign the ALU source #1 (Always the Acumulator)                   
    assign alu_src1 = (i_alu_cs == `ALU_CS_ADD || i_alu_cs == `ALU_CS_SUB || i_alu_cs == `ALU_CS_ADDC ||
                       i_alu_cs == `ALU_CS_AND || i_alu_cs == `ALU_CS_OR || i_alu_cs == `ALU_CS_XOR) ? acc : 8'd0;
    
    // Assign the ALU source #2
    // 1) Direct address or Register
    // 2) Immediate 
    assign alu_src2 = ((i_alu_cs == `ALU_CS_ADD || i_alu_cs == `ALU_CS_SUB || i_alu_cs == `ALU_CS_ADDC ||
                        i_alu_cs == `ALU_CS_AND || i_alu_cs == `ALU_CS_OR || i_alu_cs == `ALU_CS_XOR) && 
                       (i_ram_cs == `RAM_CS_RD_D || i_ram_cs == `RAM_CS_RD_R)) ? addr_mapped : IR;
     
    // Assign the ALU carry source
    // 1) If ADDC or SUB ==> psw[7]
    // 2) Else is 0                   
    assign alu_srcC = (i_alu_cs == `ALU_CS_ADDC || i_alu_cs == `ALU_CS_SUB) ? psw[7] : 0;
    assign alu_srcAc = 1'b0;
    
    // *****************************************************************************
    // Assign the SFRs operations
    
    // ALU, RAM, MOV's
    assign sfr_op_aux1 = (i_alu_cs == `ALU_CS_ADD || i_alu_cs == `ALU_CS_SUB || i_alu_cs == `ALU_CS_ADDC ||
                         i_alu_cs == `ALU_CS_AND || i_alu_cs == `ALU_CS_OR || i_alu_cs == `ALU_CS_XOR) ? (`OP_ACC_WR_BYTE | `OP_PSW_WR_FLAGS): 
                         (i_internal_cs == `MOV_TO_A) ? `OP_ACC_WR_BYTE :                        
                         (i_internal_cs == `START_CS && i_ram_cs == `RAM_CS_WR_SP) ? `OP_SP_PUSH :
                         (i_internal_cs == `RETI1_CS) ? `OP_SP_POP : 
                         (i_ram_cs == `RAM_CS_WR_D && IR == `IE_ADDR) ? `OP_IE_WR_BYTE : 
                         (i_ram_cs == `RAM_CS_WR_D && IR == `SP_ADDR) ? `OP_SP_WR_BYTE : 
                         (i_ram_cs == `RAM_CS_WR_D && IR == `TH0_ADDR) ? `OP_TH0_WR_BYTE : 
                         (i_ram_cs == `RAM_CS_WR_D && IR == `TL0_ADDR) ? `OP_TL0_WR_BYTE : 
                         (i_ram_cs == `RAM_CS_WR_D && IR == `TMOD_ADDR) ? `OP_TMOD_WR_BYTE : 
                         (i_ram_cs == `RAM_CS_WR_D && IR == `TCON_ADDR) ? `OP_TCON_WR_BYTE : 
                         (i_ram_cs == `RAM_CS_WR_D && IR == `SCON_ADDR) ? `OP_SCON_WR_BYTE :
                         (i_ram_cs == `RAM_CS_WR_D && IR == `SBUF_ADDR) ? `OP_SBUF_WR_BYTE : 
                         (i_ram_cs == `RAM_CS_WR_D && IR == `P2_ADDR) ? `OP_P2_WR_BYTE :`OP_DEFAULT;
    
    // If the Timer0 is running ==> Update the counter registers (TH0 and TL0) and TCON (Overflow flag)        
    assign sfr_op_aux2 = (tim0_en == 1'b1) ? (sfr_op_aux1 | `OP_TH0_WR_BYTE | `OP_TL0_WR_BYTE | `OP_TCON_WR_BYTE) : sfr_op_aux1;    
    
    // If UART receives a byte or sent a byte ==> Update the R1 and T1 flags in SCON
    assign sfr_op_aux3 = (rx_done == 1'b1 || tx_done == 1'b1) ? (sfr_op_aux2 | `OP_SCON_WR_BYTE) : sfr_op_aux2;
    
    // If UART receives a byte ==> Update the SBUF with the content received
    assign sfr_op = (scon[4] == 1'b1 && rx_done == 1'b1) ? (sfr_op_aux3 | `OP_SBUF_WR_BYTE) : sfr_op_aux3;
    
    
    // *****************************************************************************
    // Assign the SFRs values

    // The Acumulator is updated when:
    // 1) ALU operations (ADD, SUB, ADDC, AND, OR, XOR)
    // 2) MOV A, #data 
    // 3) MOV A, direct or Rn (RAM source or SFR ==> addr_mapped)
    assign sfr_wr_acc = (i_alu_cs == `ALU_CS_ADD || i_alu_cs == `ALU_CS_SUB || i_alu_cs == `ALU_CS_ADDC ||
                         i_alu_cs == `ALU_CS_AND || i_alu_cs == `ALU_CS_OR || i_alu_cs == `ALU_CS_XOR) ? alu_des : 
                         (i_internal_cs == `MOV_TO_A && i_ram_cs == `RAM_CS_DEFAULT) ? IR : addr_mapped;
    
    // Two operations can change the PSW:
    // 1) MOV PSW, A
    // 2) Update the flags (cy, ac && ov)                       
    assign sfr_wr_psw = (i_ram_cs == `RAM_CS_WR_D) ? acc : alu_desC + alu_desAc*2 + alu_desOv*4;
    
    // IE = ACC (MOV IE, A)
    assign sfr_wr_ie = acc;
    
    // SP = ACC (MOV SP, A)
    assign sfr_wr_sp = acc;
    
    // If the Timer0 is running ==> Update the TH0 count variable
    assign sfr_wr_th0 = (tim0_en == 1'b1) ? th0_tim_out : acc;
    
    // If the Timer0 is running ==> Update the TL0 count variable
    assign sfr_wr_tl0 = (tim0_en == 1'b1) ? tl0_tim_out : acc;
    
    // TMOD = ACC (MOV TMOD, A)
    assign sfr_wr_tmod = acc;
    
    // If the Timer0 in running and occured an overflow ==> Set the TF0 flag in TCON
    // If the Timer0 in running and not occured an overflow ==> Reset the TF0 flag in TCON
    assign sfr_wr_tcon = (tim0_en == 1'b1 && tf0_tim_out == 1'b1) ? (tcon | 8'b0010_0000) :
                         (tim0_en == 1'b1 && tf0_tim_out == 1'b0) ? (tcon & 8'b1101_1111) : acc;
                         
    // If received one byte ==> Assign the SBUF to the data received
    assign sfr_wr_sbuf = (rx_done == 1'b1) ? rx_data : acc;
    
    // If received one byte ==> Set the R1 flag in SCON
    // If transmitted one byte ==> Set the T1 flag in SCON
    assign sfr_wr_scon = (rx_done == 1'b1) ? (scon | 8'b0000_0001) :
                         (tx_done == 1'b1) ? (scon | 8'b0000_0010) : acc;
                         
    // P2 = ACC (MOV P2, A)
    assign sfr_wr_p2 = acc;
           
                         
    // *****************************************************************************
    // Assign the RAM operations
    
    // Find the RAM address:
    // 1) Read and Write direct (IR source)
    // 2) Read and Write Register (IR source + bank)
    // 3) Read and Write Stack Pointer (Stack Pointer value)
    assign ram_addr = (i_ram_cs == `RAM_CS_RD_D || i_ram_cs == `RAM_CS_WR_D) ? IR : 
                      (i_ram_cs == `RAM_CS_RD_R || i_ram_cs == `RAM_CS_WR_R) ? (bank * 8 + IR) : 
                      (i_ram_cs == `RAM_CS_RD_SP || i_ram_cs == `RAM_CS_WR_SP) ? sp : 8'd0;  

    // Fill the data to write into RAM
    // 1) Write direct or register (ACC source ==> MOV direct, A or MOV Rn, A)
    // 2) Write into Stack Pointer
    assign ram_data = (i_ram_cs == `RAM_CS_WR_D || i_ram_cs == `RAM_CS_WR_R) ? acc : 
                      (i_ram_cs == `RAM_CS_WR_SP) ? PC_aux : 8'd0;    
    
    // RAM operations
    assign ram_op = (i_ram_cs == `RAM_CS_WR_D || i_ram_cs == `RAM_CS_WR_R || i_ram_cs == `RAM_CS_WR_SP) ? `OP_RAM_WR_BYTE : `OP_RAM_NOP;
    
    
    // *****************************************************************************
    // Assign the Interrupts flags
    
    // External 0 Interrupt is enable when:
    // EA = 1 && EX0 = 1 
    assign ext0_int_en = (ie[7] == 1'b1 && ie[0] == 1'b1) ? 1'b1 : 1'b0; 
    
    // Timer 0 Interrupt is enable when:
    // EA = 1 && ET0 = 1                     
    assign tim0_int_en = (ie[7] == 1'b1 && ie[1] == 1'b1) ? 1'b1 : 1'b0;
    
    // UART Interrupt is enable when:
    // EA = 1 && ES0 = 1 
    assign uart_int_en = (ie[7] == 1'b1 && ie[4] == 1'b1) ? 1'b1 : 1'b0; 
    
    // Update the interrupt flag that will signal the CPU that will be a new interrupt to process
    assign o_int_pend = tim0_int_one_pulse | ext0_int_one_pulse | uart_int_one_pulse;
    
    // Update the button status that will connect to External Interrupt 0
    assign ext0_button = i_button;
    //assign ext0_button = 1'b0;
    
    // Save the program counter in case of interrupt (Start State)
    assign PC_aux = (i_internal_cs == `START_CS) ? PC_save[7:0] : PC_save[15:8];   
    
    // ########################
    reg [15:0] counter;
    reg [7:0] counter_leds;
    // ########################
    
    // Assign the LEDs
    assign o_leds = p2;
    //assign o_leds = counter_leds;
    //assign o_leds = sbuf;
    
    // *****************************************************************************   
    // Define the initial conditions    
    initial
    begin
        PC <= 16'd0;
        IR <= 8'd0;
        PC_save <= 16'd0;
        ext0_int_in <= 1'b0;
        tim0_int_in <= 1'b0;
        uart_int_in <= 1'b0;
        ext0_int_req <= 1'b0;
        tim0_int_req <= 1'b0;
        uart_int_req <= 1'b0;
        // ########################
        counter <= 16'd0;
        counter_leds <= 8'd0; 
        // ########################
    end
    
    // *****************************************************************************      
    // Instruction Register (IR)   
    always @(posedge i_clk)
    begin
        if(i_rst == 1'b1)
        begin
            IR <= 0;
        end
        else if(i_internal_cs == `IR_PC_LOAD1 || i_internal_cs == `IR_PC_LOAD2)
        begin
            IR <= rom_out;
        end
    end 
    
    
    // *****************************************************************************
    // Interrupts Manager
    
    // EXT0: Higher Priority
    // UART: Medium Priority
    // TIM0: LOwer Priority 
    always @(posedge i_clk)
    begin
        if(i_rst == 1'b1)
        begin
            ext0_int_req = 1'b0;
            tim0_int_req = 1'b0;
            uart_int_req = 1'b0;
        end
        else begin
            // Priority #1
            if(ext0_int_status == 1'b1) begin       // Ocurred a valid interrupt in External 0
                ext0_int_req = 1'b1;
            end
            else if (ext0_int_in == 1'b1) begin     // The EXT0 interrupt is already taken
                ext0_int_req = 1'b0;
            end
            
            // Priority #2
            if(uart_int_status == 1'b1) begin       // Ocurred a valid interrupt in UART
                uart_int_req = 1'b1;
            end
            
            else if (uart_int_in == 1'b1) begin     // The UART interrupt is already taken
                uart_int_req = 1'b0;
            end
            
            // Priority #3
            if(tim0_int_status == 1'b1) begin       // Ocurred a valid interrupt in Timer 0
                tim0_int_req = 1'b1;
            end
            
            else if (tim0_int_in == 1'b1) begin     // The TIM0 interrupt is already taken
                tim0_int_req = 1'b0;
            end            
        end
    end
    
    
    // *****************************************************************************
    // Program Counter (PC)   
    always @(posedge i_clk)
    begin
        if(i_rst == 1'b1)
        begin
            PC <= 0;
            ext0_int_in = 1'b0;
            tim0_int_in = 1'b0;
            uart_int_in = 1'b0;
            
            // ########################
            counter = 16'd0;
            counter_leds = 8'd0;
            // ######################## 
        end
        else
        begin
            case (i_internal_cs)            
                `IR_PC_LOAD1: begin
                    if(ext0_int_req == 1'b1) begin              // If the EXT0 interrupt is pending
                        PC <= `ISR_EXT0_ADDR;                   // Update the PC 
                        ext0_int_in <= 1'b1;                    // Update the flag
                        PC_save <= PC;                          // Save the next instruction
                    end
                    else if(uart_int_req == 1'b1) begin         // If the UART interrupt is pending
                        PC <= `ISR_UART_ADDR;                   // Update the PC 
                        uart_int_in <= 1'b1;                    // Update the flag
                        PC_save <= PC;                          // Save the next instruction
                    end              
                    else if(tim0_int_req == 1'b1) begin         // If the TIM0 interrupt is pending
                        PC <= `ISR_TIM0_ADDR;                   // Update the PC 
                        tim0_int_in <= 1'b1;                    // Update the flag
                        PC_save <= PC;                          // Save the next instruction
                        
                        // ########################
                        counter <= counter + 1;
                        if (counter == 16'd2000)
                        begin
                            counter_leds <= counter_leds + 1;
                            counter <= 16'd0;
                        end
                        // ########################
                    end      
                    else begin
                       PC <= PC + 1;                            // Normal (Sequential) execution
                    end
                end           
                `IR_PC_LOAD2: begin                             // 2nd Fetch (1 address machine)
                    PC <= PC + 1;
                end
                `JMP_C_LOAD: begin                              // JC rel
                    if (psw[7] == 1'b1)
                        PC <= o_ir; 
                end
                `JMP_NC_LOAD: begin                             // JNC rel
                    if (psw[7] == 1'b0)
                        PC <= o_ir; 
                end                
                `JMP_Z_LOAD: begin                              // JZ rel
                    if (acc == 8'd0)
                        PC <= o_ir; 
                end
                `JMP_NZ_LOAD: begin                             // JNZ rel
                    if (acc != 8'd0)
                        PC <= o_ir; 
                end
                `RETI1_CS: begin
                    PC <= {ram_out, 8'd0};                      // Extract from Stack the PC[15:8]
                end
                `RETI2_CS: begin                                // Wait for RAM to output the SP value
                end
                `RETI3_CS: begin
                    PC <= (PC | ram_out);                       // Extract from Stack the PC[7:0]
                    ext0_int_in <= 1'b0;                        // Clear the flag
                    tim0_int_in <= 1'b0;                        // Clear the flag
                    uart_int_in <= 1'b0;                        // Clear the flag
                end                              
            endcase
        end      
    end
    
endmodule