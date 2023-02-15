`timescale 1ns / 1ps

/*
    The Arithmetic Logic Unit, as the name suggests, performs the arithmetic
    and logical operations on the instructions being executed. 
    The ALU module receives from the controller, three 8-bit source operands, 
    status of carry flags, and the op-code for instruction type.
*/

`include "Defines.v"

module ALU(
        input[`ALU_CS_LEN-1:0] i_operation,
        input[7:0] i_src1,
        input[7:0] i_src2,
        input i_srcC,
        input i_srcAc,
        output reg [7:0] o_des1,
        output reg o_desC,
        output reg o_desAc,
        output reg o_desOv
        );

    
    /* Add operation */
    wire [4:0] add1, add2, add3, addC, add3C;
    wire [3:0] add4, add5, add6, add6C, add7, add7C;
    wire [1:0] add8, add9, add10, add10C, add11, add11C;
    
    assign add1 = {1'b0,i_src1[3:0]};
    assign add2 = {1'b0,i_src2[3:0]};
    assign add3 = add1+add2;
    assign addC = {4'b0000,i_srcC};
    assign add3C = add3 + addC; 
    
    assign add4 = {1'b0,i_src1[6:4]};
    assign add5 = {1'b0,i_src2[6:4]};
    assign add6 = {3'b0,add3[4]};
    assign add6C = {3'b0,add3C[4]};
    assign add7 = add4+add5+add6;
    assign add7C = add4+add5+add6C;
    
    assign add8 = {1'b0,i_src1[7]};
    assign add9 = {1'b0,i_src2[7]};
    assign add10 = {1'b0,add7[3]};
    assign add10C = {1'b0,add7C[3]};
    assign add11 = add8 + add9 + add10;
    assign add11C = add8 + add9 + add10C;
    
    /* Sub operation */
    wire [4:0] sub1, sub2, sub3, sub4;
    wire [3:0] sub5, sub6, sub7, sub8;
    wire [1:0] sub9, suba, subb, subc;
    
    assign sub1 = {1'b1,i_src1[3:0]};
    assign sub2 = {1'b0,i_src2[3:0]};
    assign sub3 = {4'b0,i_srcC};
    assign sub4 = sub1-sub2-sub3;
    
    assign sub5 = {1'b1,i_src1[6:4]};
    assign sub6 = {1'b0,i_src2[6:4]};
    assign sub7 = {3'b0, !sub4[4]};
    assign sub8 = sub5-sub6-sub7;
    
    assign sub9 = {1'b1,i_src1[7]};
    assign suba = {1'b0,i_src2[7]};
    assign subb = {1'b0,!sub8[3]};
    assign subc = sub9-suba-subb;
    
    initial begin
        o_des1 <= 8'd0;
        o_desC <= 1'b0;
        o_desAc <= 1'b0;
        o_desOv <= 1'b0;
    end

    always @(*)
    begin
        case(i_operation)
        
            `ALU_CS_ADD: begin
                o_des1 = {add11[0],add7[2:0],add3[3:0]};
                o_desC = add11[1];
                o_desAc = add3[4];
                o_desOv = add11[1] ^ add7[3];
            end
            
            `ALU_CS_ADDC: begin
                o_des1 = {add11C[0],add7C[2:0],add3C[3:0]};
                o_desC = add11C[1];
                o_desAc = add3C[4];
                o_desOv = add11C[1] ^ add7C[3];
            end
            
            `ALU_CS_SUB: begin
                o_des1 = {subc[0],sub8[2:0],sub4[3:0]};
                o_desC = !subc[1];
                o_desAc = !sub4[4];
                o_desOv = !subc[1] ^ !sub8[3];
            end
            
            `ALU_CS_AND: begin
                o_des1 = i_src1 & i_src2;
            end
            `ALU_CS_XOR: begin
                o_des1 = i_src1 ^ i_src2;
            end
            
            `ALU_CS_OR: begin
                o_des1 = i_src1 | i_src2;
            end
        endcase
    end
endmodule
