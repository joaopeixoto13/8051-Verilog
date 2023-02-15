`timescale 1ns / 1ps
`include "Defines.v"

/*
    This modules implements the Timer 0 and Timer 1 (x == 0 or x == 1)
    This architecture uses a single clock signal for all logic, rather than generating a separate, slower clock 
    from combinational logic. 
    For various reasons, is better using as few different clocks as possible (synchronize clock and Metastability)
*/

module Timer(
    input i_clk,            // Clock source
    input i_rst,            // Reset
    input [7:0] i_tmod,     // Modes
    input [7:0] i_thx,      // THx received by THx SFR   
    input [7:0] i_tlx,      // TLx received by TLx SFR
    input i_trx,            // TRx received by (TCON[4] if x == 0 or TCON[6] if x == 1)
    input i_intx,           // INTx Received by (IE[1] if x == 0 or IE[3] if x == 0)
    output [7:0] o_thx,     // THx updated
    output [7:0] o_tlx,     // TLx updated
    output o_tfx,           // Overflow flag (Must be conected to TCON[5] if x == 0 or TCON[7] if x == 1)
    output o_en             // Timer/Counter is enable            
    );
    
    // TMOD register
    reg [7:0] tmod;
    
    // TH0 register
    reg [7:0] thx;
    
    // TL0 register
    reg [7:0] tlx;
    
    // Overflow flag
    reg tfx;
    
    // Specifies how the control will work
    // GATE = 1 if Timer0 will be able to work if TR0 = 1 and INT0 = 1 (Interrupt control)
    // GATE = 0 if Timer0 will be able to work if TR0 = 1 (Software control)
    parameter GATE = 8'd4;  
    
    // Selects the mode
    // CT = 1 to define the Timer0 as Counter (DEFINIR UM PINO)
    // CT = 0 to define the Timer0 as Timer
    parameter CT = 4'b0100;
    
    // Selects the operation mode
    // M1 = 0 and M0 = 0 ==> 13bits timer/counter (TH0 as 8bit and TL0 as 5bit prescaler)
    // M1 = 0 and M0 = 1 ==> 16bits timer/counter
    // M1 = 1 and M0 = 0 ==> 8 bits timer/counter with Autoreload (TL0 as timer/counter and TH0 as autoreload)
    // M1 = 1 and M0 = 1 ==> TH0 and TL0 runs as 8bits Autoreload and control by TR0, INT0, TF0, TR1, INT1, TF1 respectively
    
    // Initial state
    initial begin
      tmod <= 8'd0;
      thx <= 8'd0;
      tlx <= 8'd0;
      tfx <= 1'b0;   
    end 
    
    // Update the enable flag
    assign o_en = (tmod[GATE] == 1'b1 && i_trx == 1'b1 && i_intx) ? 1'b1 :
                  (tmod[GATE] == 1'b0 && i_trx == 1'b1) ? 1'b1 : 1'b0;
                
    // Timer Loop
    always @(posedge i_clk)
    begin
        if (i_rst == 1'b1) begin
            tmod <= 8'd0;
            thx <= 8'd0;
            tlx <= 8'd0;
            tfx <= 1'b0;
        end
        else begin
            // If the THx, TLx or TMOD SFR change, update the internal registers 
            if (i_thx != thx) begin
                thx <= i_thx;
            end
            else if (i_tlx != tlx) begin
                tlx <= i_tlx;
            end
            else if (i_tmod != tmod) begin
                tmod <= i_tmod;
            end 
            
            // If the timer is enable
            if (o_en == 1'b1)
            begin
                case (i_tmod[1:0]) 
                    `TIMER_MODE_0: begin
                        {tfx, thx, tlx[4:0]} <= {1'b0, thx, tlx[4:0]} + 1'b1;
                    end
                    `TIMER_MODE_1: begin
                        {tfx, thx, tlx} <= {1'b0, thx, tlx} + 1'b1;
                    end
                    `TIMER_MODE_2: begin
                        if (tlx == 8'b1111_1111) begin      // If is overflow
                            tfx <= 1'b1;                    // Update the overflow flag
                            tlx <= thx;                     // Reload the timer/counter
                        end
                        else begin
                            tlx <= tlx + 8'h1;              // Increment the timer/counter
                            tfx <= 1'b0;                    // Clear the overflow flag
                        end
                     end
                    `TIMER_MODE_3: begin
                        {tfx, tlx} <= {1'b0, tlx} +1'b1;
                    end
                    default: begin
                        tmod <= 0;
                        thx <= 0;
                        tlx <= 0;
                        tfx <= 0;
                    end
                 endcase
            end
            else begin
                thx <= i_thx;
                tlx <= i_tlx;
                tfx <= 1'b0;  
            end
        end
    end
    
    // Update the overflow flag
    assign o_tfx = tfx;
    
    // Update the output values
    assign o_thx = thx;
    assign o_tlx = tlx;
    
endmodule