`timescale 1ns / 1ps

/*
    CDC (Clock Domain Crossing) is the traversal of a signal in a synchronous digital circuit
    from one clock domain into another. If a signal does not assert long enough and is not
    registered, it may appear asynchronous on the incoming clock boundary.
    
    Metastable state is one in which the output of a flip-flop inside of our FPGA is unknown, 
    or non-deterministic. When a metastable condition occurs, there is no way to tell if the output 
    of our flip-flop is going to be a 1 or a 0. A metastable condition occurs when setup or hold times
    are violated. In this context, the Metastability can cause the FPGA to exhibit very
    strange behavior. This situation can be fixed by adding two flip-flops.
    This means that the signal that is asynchronous to the clock is being sampled by the
    first flip-flop. This will create a metastable condition at the output. If we again sample
    this output, we can now fix the metastable event. The output of the second flip-flop
    will be stable.
*/

module CDC(
        input i_clk,        // Clock source
        input i_rst,        // Reset
        input i_signal,     // Input signal (not stable)
        output o_signal     // Output signal (stable one)
    );
    
    // Clock Domain Crossing implemented with 2 Flip-Flops
    reg [1:0] cdc;
    
    // Assign the stable signal (second Flip-Flop)
    assign o_signal = cdc[1];
    
    // Initial Conditions
    initial begin
        cdc <= 2'd0;
    end
    
    // Clock Domain Crossing
    always @(posedge i_clk or posedge i_rst)
    begin
        if (i_rst) begin
            cdc <= 2'b00;
        end
        else begin
            cdc <= {cdc[0], i_signal};
        end
    end 
    
    
endmodule