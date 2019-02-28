`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/03/14 11:27:26
// Design Name: 
// Module Name: TOP_test
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


module TOP_test(

    ); 
    reg clk,RST;
    wire [15:0]show;
    integer i;
    initial begin
    clk<=0;
    RST<=1;
     for(i=0;i<1700;i=i+1) begin
     #5 clk<=~clk;
     #5 clk<=~clk;
     if(i==1700) begin
     #5 RST<=~RST;
      #5 RST<=~RST;
      end
     end
    end
   TOP top(clk,RST,show);
   
   
    
endmodule
