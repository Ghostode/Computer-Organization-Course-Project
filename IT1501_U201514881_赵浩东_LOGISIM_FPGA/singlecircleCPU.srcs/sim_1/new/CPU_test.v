`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/03/14 02:57:52
// Design Name: 
// Module Name: CPU_test
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module CPU_test(

    );
   reg clk,RST;
   wire [31:0]SyscallOut,SyscallOut2,SyscallOut3;
   integer i;
   initial begin
    clk<=0;
    RST<=0;
    for(i=0;i<1700;i=i+1) begin
     #5 clk<=~clk;
     #5 clk<=~clk;

     end
     end
     CPU test(clk,RST,SyscallOut,8'h00);
     CPU test2(clk,RST,SyscallOut2,8'h07);
     CPU test3(clk,RST,SyscallOut3,8'h04);

endmodule
