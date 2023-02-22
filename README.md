# 8051-Verilog

---

### Authors
- João Peixoto
- Diogo Silvino

---

### Brief Overview

This version of 8051 ISA (Instruction Set Architecture) is composed by 28 instructions:
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
 
 ---
 
 ### Modules
      - Top
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
         - Debounce             Debounce the physical button
            - Prescaler         Implements a presclater to slow-down the clock
            
 ---
 
 More Information: [Report](PG50341_PG50479-8051.pdf)
         
