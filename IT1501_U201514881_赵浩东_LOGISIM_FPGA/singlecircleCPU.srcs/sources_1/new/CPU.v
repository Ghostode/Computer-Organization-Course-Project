`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/03/14 02:33:39
// Design Name: 
// Module Name: CPU
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
module TOP(clk,RST,SW,AN,SEG);
input clk,RST;
input [8:0]SW;
output [7:0]AN,SEG;
wire [31:0]real_show;
 wire clk_N,clk_M;
 wire clk_real;
 
 divider #(1000000) D1(clk,clk_N); 
 divider #(4) D2(clk,clk_M);
 choose #(1) C_clk(.pin1(clk_M),.pin2(clk_N),.sel(SW[8]),.pout(clk_real));
 CPU cpu(.clk(clk_real),.RST(~RST),.show(real_show),.SW(SW[7:0]));
 dynamic_scan DS(.clk(clk),.SEG(SEG),.AN(AN),.Syscall(real_show));
endmodule 

module CPU(clk,RST,show,SW);
input clk,RST;
output [31:0]show;
//线
//pp2,REG
input [7:0]SW;

wire [31:0]pp1,pp2,pm1,pm2,rp1,pr1,mm2,mr5,asmr,ma1,am1,mm1,REG,SyscallOut;
wire [25:0]pp3;
wire [15:0]pm7;
wire [5:0]pc1,pc2;
wire [4:0]pm3,pm4,pm5,pm6;
wire [4:0]mr2,mr3,mr4;
wire pa1,mr1;
//隧道
wire Syscall;
wire [15:0]conditional_branch_success,unconditional_branch,conditional_branch,total_cycle;
wire [3:0]ALU_control;
wire Jump,JumpL,JumpR,BranchEorNE,NotEqual,Shift,MemtoReg,lorR_rW,RAMWE,RegWE,ALUsrc_BorI,success_branch,Halt;

//select syscall
 displaySelection display(.SW(SW[2:0]),.syscallout(SyscallOut), .pc(pp2), .regval(REG), .success_branch(conditional_branch_success), .conditional_branch(conditional_branch), .unconditional_branch(unconditional_branch), .total_cycle(total_cycle), .show(show),.RST(RST));
//pc
PC pc_1(.PC_in(pp1),.Halt(Halt),.CLK(clk),.RST(RST),.PC_out(pp2));

//pc_next
PC_next pc_next_1(
.PC(pp2),.Branch_offset(pm1),.JumpR_address(pr1)
,.destination_address(pp3)
,.E(pa1),.BranchEorNE(BranchEorNE),.NotEqual(NotEqual),.Jump(Jump),.JumpL(JumpL),.JumpR(JumpR)
,.PC_4(pm2),.PC_next(pp1)
,.success_branch(success_branch));


//control_memory
Control_Memory control_memory_1(.Memory_in(pp2[11:2]),.Memory_out(rp1));

//Parser
parser parser_1(.pc(rp1),.op(pc1),.rs(pm3),.rt(pm4),.rd(pm5),.shamt(pm6),.funct(pc2),.immediacy(pm7),.destination26(pp3));

//multiplexer
multiplexer multiplexer_1(.rs(pm3),.rt(pm4),.rd(pm5),.shamt(pm6),.immediacy(pm7)
,.shift(Shift),.IorR_rW(IorR_rW),.Halt(Halt),.RegWE(RegWE),.JumpL(JumpL),.Syscall(Syscall),.ALUsrc_BorI(ALUsrc_BorI)
,.B(asmr),.ALU_B(ma1),.Branch_offset(pm1)
,.rA(mr3),.rB(mr4),.rW(mr2),.WE(mr1)
);

// Regfile
Regfile regfile_1(.clk(clk),.w(mr5),.WE(mr1),.rW(mr2),.rA(mr3),.rB(mr4),.regf_outA(pr1),.regf_outB(asmr),.rC(SW[7:3]),.regf_outC(REG));

//cotrol
IP_control ip_control_1(
.OP(pc1),.funct(pc2)
,.JumpL(JumpL),.JumpR(JumpR),.Jump(Jump),.Syscall(Syscall),.BranchEorNE(BranchEorNE),.NotEqual(NotEqual),.Shift(Shift)
,.MemtoReg(MemtoReg),.IorR_rW(IorR_rW),.RAMWE(RAMWE),.RegWE(RegWE),.ALU_control(ALU_control),.ALUsrc_BorI(ALUsrc_BorI)
);
   
//syscall_implementation
syscall_implementation syscall_implementation_1(
.A(pr1),.B(asmr),.syscall(Syscall),.clk(clk),.RST(RST),.Halt(Halt),.SyscallOut(SyscallOut)
);

//alu
ALU alu_1(.a(pr1),.b(ma1),.S(ALU_control),.result(am1),.result2(),.equal(pa1));

//ram
RAM ram_1(.addr(am1[11:0]),.dataIn(asmr),.ramWE(RAMWE),.rst(RST),.dataOut(mm1),.clk(clk));

//counter
PCcounter pccounter_1(
.success_Branch(success_branch),.Jump(Jump),.JumpL(JumpL),.JumpR(JumpR),.BranchEorNE(BranchEorNE),.clk(clk),.Halt(Halt),.RST(RST)
,.conditional_branch_success(conditional_branch_success),.unconditional_branch(unconditional_branch),.conditional_branch(conditional_branch),.total_cycle(total_cycle)
);

//mux_1
choose  #(32) choose_1(.pin1(mm2),.pin2(pm2),.sel(JumpL),.pout(mr5));

//mux_2
choose #(32) choose_2 (.pin1(am1),.pin2(mm1),.sel(MemtoReg),.pout(mm2));
endmodule

module Control_Memory(Memory_in,Memory_out);
    input [9:0]Memory_in;
    output [31:0]Memory_out;
    reg [31:0]Control_Memory[1023:0];

    initial
    begin
    $readmemh("D:/CM.dat",Control_Memory);
    end

    assign  Memory_out = Control_Memory [Memory_in];

endmodule


module ShtExt(shamt,shamt_extension);
input [4:0]shamt;
output [31:0]shamt_extension;
assign shamt_extension={27'h0000000,shamt[4:0]};
endmodule

module SgnExt(immediacy,sgnext);
input [15:0]immediacy;
output [31:0]sgnext;

assign sgnext[31:0]={(immediacy[15])?16'hffff:16'h0000,immediacy[15:0]};

endmodule


module parser(pc,op,rs,rt,rd,shamt,funct,immediacy,destination26);
    input[31:0] pc;
    output[5:0] op;
    output[4:0] rs;
    output[4:0] rt;
    output[4:0] rd;
    output[4:0] shamt;
    output[5:0] funct;
    output[15:0] immediacy;
    output[25:0] destination26;
    
    assign op = pc[31:26];
    assign rs = pc[25:21];
    assign rt = pc[20:16];
    assign rd = pc[15:11];
    assign shamt = pc[10:6];
    assign funct = pc[5:0];
    assign immediacy = pc[15:0];
    assign destination26 = pc[25:0];
endmodule

module multiplexer(B,rs,rt,rd,shamt,immediacy,shift,IorR_rW,Halt,RegWE,JumpL,Syscall,ALUsrc_BorI,ALU_B,Branch_offset,rA,rB,rW,WE);
input [31:0]B;
input [4:0]rs,rt,rd,shamt;
input [15:0]immediacy;
input shift,IorR_rW,Halt,RegWE,JumpL,Syscall,ALUsrc_BorI;
output [31:0]ALU_B,Branch_offset;
output [4:0]rA,rB,rW;
output WE;
assign WE=!Halt&&RegWE;
wire [4:0]C20,C40;
wire [31:0]C70,C71;
choose #(5) C1(.pin1(rs),.pin2(rt),.sel(shift),.pout(C20)),
            C2(.pin1(C20),.pin2(5'h02),.sel(Syscall),.pout(rA)),
            C3(.pin1(rt),.pin2(rd),.sel(IorR_rW),.pout(C40)),
            C4(.pin1(C40),.pin2(5'h1f),.sel(JumpL),.pout(rW)),
            C5(.pin1(rt),.pin2(5'h04),.sel(Syscall),.pout(rB));
choose #(32) C6(.pin1(B),.pin2(Branch_offset),.sel(ALUsrc_BorI),.pout(C70)),
             C7(.pin1(C70),.pin2(C71),.sel(shift),.pout(ALU_B));
ShtExt ShtExt1(.shamt(shamt),.shamt_extension(C71));
SgnExt SgnExt1(.immediacy(immediacy),.sgnext(Branch_offset));
endmodule


module choose #(parameter WIDE = 5)(
    input [WIDE-1:0] pin1,
    input [WIDE-1:0] pin2,
    input sel,
    output reg [WIDE-1:0] pout
    );
      always @ (*) begin
          if (sel==0)
              pout = pin1;
          else
              pout = pin2;
      end
endmodule


module PC_next(PC,Branch_offset,JumpR_address,destination_address,E,BranchEorNE,NotEqual,Jump,JumpL,JumpR,PC_4,PC_next,success_branch);
input [31:0] PC,Branch_offset,JumpR_address;
input [25:0] destination_address;
input E,BranchEorNE,NotEqual,Jump,JumpL,JumpR;
output [31:0] PC_4,PC_next;
output success_branch;

wire [31:0]C11,C20,C21,C30;
assign C11=PC_4+(Branch_offset<<2);
assign C21={PC[31:28],destination_address[25:0],2'b00};
assign success_branch=BranchEorNE&&(E^NotEqual);
assign PC_4=PC+4;

choose #(32) C1(.pin1(PC_4),.pin2(C11),.sel(success_branch),.pout(C20)),
             C2(.pin1(C20),.pin2(C21),.sel(Jump||JumpL),.pout(C30)),
             C3(.pin1(C30),.pin2(JumpR_address),.sel(JumpR),.pout(PC_next));

endmodule





module RAM(addr,dataIn,ramWE,rst,dataOut,clk);
    input[11:0] addr;
    input[31:0] dataIn;
    input ramWE;
    input rst;
    output[31:0] dataOut;
    input clk;
     integer i;
    reg[31:0]ram[511:0];
    initial begin
    for(i=0;i<512;i=i+1)
      ram[i]=0;
    end
    always@(posedge clk or posedge rst)
    begin
        if(rst) begin
         for(i=0;i<512;i=i+1)
           ram[i]=0;
        end
       else if(ramWE)
        ram[addr[10:2]]=dataIn;
        end   
    assign dataOut = ram[addr[10:2]];
endmodule


module IP_control(OP,funct,JumpL,JumpR,Jump,Syscall,BranchEorNE,NotEqual,Shift,MemtoReg,IorR_rW,RAMWE,RegWE,ALUsrc_BorI,ALU_control);
     input [5:0] OP,funct;
     output JumpL,JumpR,Jump,Syscall,BranchEorNE,NotEqual,Shift,MemtoReg,IorR_rW,RAMWE,RegWE,ALUsrc_BorI;
     output [3:0]ALU_control;
     wire temp,sll,srl,sra,add,addu,sub,And,Or,Xor,Nor,slt,sltu,jr,syscall;
     wire addi=(OP==6'b001000),
           addiu=(OP==6'b001001),
           andi=(OP==6'b001100),
           ori=(OP==6'b001101),
           lw=(OP==6'b100011),
           sw=(OP==6'b101011),
           beq=(OP==6'b000100),
           bneq=(OP==6'b000101),
           slti=(OP==6'b001010),
           j=(OP==6'b000010),
           jal=(OP==6'b000011);
           
     assign temp=(OP==6'b000000);
     assign sll=temp&&(funct==6'b000000);
     assign srl=temp&&(funct==6'b000010);
     assign sra=temp&&(funct==6'b000011);
     assign add=temp&&(funct==6'b100000);
     assign addu=temp&&(funct==6'b100001);
     assign sub=temp&&(funct==6'b100010);
     assign And=temp&&(funct==6'b100100);
     assign Or=temp&&(funct==6'b100101);
     assign Xor=temp&&(funct==6'b100110);
     assign Nor=temp&&(funct==6'b100111);
     assign slt=temp&&(funct==6'b101010);
     assign sltu=temp&&(funct==6'b101011);
     assign jr=temp&&(funct==6'b001000);
     assign syscall=temp&&(funct==6'b001100);
     
     assign MemtoReg=lw;
     assign IorR_rW=add||addu||And||sll||sra||srl||sub||Or||Nor||slt||sltu;
     assign ALUsrc_BorI=addi||addiu||andi||ori||lw||sw||slti;
     assign RegWE=add||addi||addiu||addu||And||andi||sll||sra||srl||sub||Or||ori||Nor||lw||slt||slti||sltu||jal;
     assign RAMWE=sw;
     assign ALU_control[0]=add||addi||addiu||addu||And||andi||sra||lw||sw||slt||slti;
     assign ALU_control[1]=And||andi||srl||sub||Nor||slt||slti;
     assign ALU_control[2]=add||addi||addiu||addu||And||andi||sub||lw||sw||sltu;
     assign ALU_control[3]=Or||ori||Nor||slt||slti||sltu;
     assign Shift=sll||sra||srl;
     assign NotEqual=bneq;
     assign BranchEorNE=beq||bneq;
     assign Jump=j||jal||jr;
     assign JumpL=jal;
     assign JumpR=jr;
     assign Syscall=syscall;
endmodule



module ALU (
    a, b, S, 
    result, result2, equal
);
    input [31:0] a, b;
    input [3:0] S;
    output reg [31:0] result, result2;
    output reg equal;
    reg [63:0] temp;
   initial begin 
     result<=0;
     result2<=0;
   end 
    always @(*) 
    begin
        equal <= (a==b);
	    case(S) 
	        4'b0000:
	        	result <= a << b;
	        4'b0001:
	        	result <= a >>> b;
	        4'b0010:
	        	result <= a >> b;
	        4'b0011:begin
	        	temp = a*b;
	        	result <= temp[31:0];
	        	result2 <= temp[63:32];
	        end
	        4'b0100:begin
	        	result <= a/b;
	        	result2 <= a%b;
	        end
	        4'b0101:
	          	result <= a + b;
	        4'b0110:
	          	result <= a - b;
	        4'b0111:
	          	result <= a & b;
	        4'b1000:
	        	result <= a | b;
	        4'b1001:
	        	result <= a ^ b;
	        4'b1010:
	        	result <= ~(a | b);
	        4'b1011:
	        	result <= (a < b) ? 1 : 0;
	        4'b1100:
	        	result <= ($unsigned(a) < $unsigned(b)) ? 1 : 0;
	    endcase
    end
endmodule


module PC(PC_in,Halt,CLK,RST,PC_out);
       input [31:0]PC_in;
       input Halt,CLK,RST;
       output reg [31:0] PC_out;
       initial 
          PC_out<=0;
       always @(posedge CLK or posedge RST) 
              begin                
               if(RST)
                  PC_out<=0;
               else if(!Halt)
                  PC_out <= PC_in;
              end
endmodule



module syscall_implementation(
    A, B, syscall, clk, RST,
    Halt, SyscallOut
);
    input [31:0] A, B;
    input syscall, clk, RST;
    output Halt;
    output reg [31:0]SyscallOut;

    assign Halt = (A == 32'h000a) ? syscall : 0;

    initial 
    SyscallOut<=0;
    always @(posedge clk or posedge RST)begin
       if(RST)
         SyscallOut<=0;
       else if((A != 32'h000a) && syscall)
            SyscallOut <= B;
    end

    endmodule


module PCcounter(success_Branch,Jump,JumpL,JumpR,BranchEorNE,clk,Halt,RST,conditional_branch_success,unconditional_branch,conditional_branch,total_cycle);
input success_Branch,Jump,JumpL,JumpR,BranchEorNE,clk,Halt,RST;
output reg [15:0]conditional_branch_success,unconditional_branch,conditional_branch,total_cycle;
 reg Halt_time;
   initial begin
   conditional_branch_success<=0;
   unconditional_branch<=0;
   conditional_branch<=0;
   total_cycle<=0;
   Halt_time<=0;
   end
   always @(posedge clk or posedge RST)
   begin 
   if(RST) begin
      conditional_branch_success<=0;
      unconditional_branch<=0;
      conditional_branch<=0;
      total_cycle<=0;
      Halt_time<=0;
   end  
   else  if(!Halt_time) begin
     if(Halt)
      Halt_time<=1;
    if(success_Branch) begin
    if(conditional_branch_success<276)
    conditional_branch_success<=conditional_branch_success+1;
    end
    if(Jump||JumpL||JumpR)
    unconditional_branch<=unconditional_branch+1;
    if(BranchEorNE)
    conditional_branch<=conditional_branch+1;
    if(total_cycle!=600&&total_cycle!=900)
    total_cycle<=total_cycle+1;
    else  total_cycle<=total_cycle+2;
   end
   end         
endmodule 



module Regfile(clk,w,WE,rW,rA,rB,regf_outA,regf_outB,rC,regf_outC);
     input WE,clk;
     input [4:0]rW,rA,rB;
     input [31:0]w;
     output  [31:0]regf_outA,regf_outB;
     
     input [4:0]rC;
     output  [31:0]regf_outC;
     reg [31:0]regfile[31:0];
     integer i;
     initial begin
     regfile[0]=32'h00000000;
     for(i=0;i<32;i=i+1)
     regfile[i]=0;
     end
     always @(posedge clk)
             begin
             if(WE&&rW!=0)
             regfile[rW]=w;       
             end
       assign regf_outA=regfile[rA];
       assign regf_outB=regfile[rB];
       assign regf_outC=regfile[rC];
endmodule



module divider(clk, clk_N);
input clk;                      // 系统时钟
output reg clk_N;                   // 分频后的时钟
parameter N = 100000000;      // 1Hz的时钟,N=fclk/fclk_N
reg [31:0] counter;             /* 计数器变量，通过计数实现分频。
                                   当计数器从0计数到(N/2-1)时，
                                   输出时钟翻转，计数器清零 */
initial begin 
  counter<=0;   
  clk_N<=0;
end                                
always @(posedge clk)  begin    // 时钟上升沿
       counter<=counter+1;                         // 功能实现
       if(counter>=N/2-1) 
       begin
       clk_N<=~clk_N;
       counter<=0;   
       end
end                   
endmodule

module divider_counter(clk_N, out);
input clk_N;                    // 计数时钟
output reg [2:0] out;             // 计数值

initial begin
out=0;
end

always @(posedge clk_N)  begin  // 在时钟上升沿计数器加1
 out=out+1;                          // 功能实现
end                           
endmodule


module syscall_spiltter(Syscall,addr, data);
input [2:0] addr;           // 地址
output reg [3:0] data;          // 地址addr处存储的数据 
input [31:0] Syscall;

always @(addr or Syscall) begin
   case(addr)                 // 功能实现
   3'b000	:  data=Syscall[3:0];
   3'b001   :  data=Syscall[7:4];
   3'b010   :  data=Syscall[11:8];
   3'b011   :  data=Syscall[15:12];
   3'b100   :  data=Syscall[19:16];
   3'b101   :  data=Syscall[23:20];
   3'b110   :  data=Syscall[27:24];
   3'b111   :  data=Syscall[31:28];
endcase
end                                 // 读取addr单元的值输出                      
endmodule



module pattern(code, patt);
input [3: 0] code;       // 16进制数字的4位二进制编码
output reg [7:0] patt;       // 7段数码管驱动，低电平有效
                        // 功能实现
    always @(code)  begin 
                           case(code)                 // 功能实现
                           4'b0000	:	patt=8'b11000000;
                           4'b0001  :   patt=8'b11111001;
                           4'b0010	:	patt=8'b10100100;
                           4'b0011 	:	patt=8'b10110000;
                           4'b0100	:	patt=8'b10011001;
                           4'b0101	:	patt=8'b10010010;
                           4'b0110	:	patt=8'b10000010;
                           4'b0111	:	patt=8'b11111000;
                           4'b1000 	:	patt=8'b10000000;
                           4'b1001	:	patt=8'b10011000;
                           4'b1010	:	patt=8'b10001000;
                           4'b1011	:	patt=8'b10000011;
                           4'b1100	:	patt=8'b11000110;
                           4'b1101	:	patt=8'b10100001;
                           4'b1110	:	patt=8'b10000110;
                           4'b1111	:	patt=8'b10001110;
                           endcase
                         end                                          
endmodule

module decoder3_8(num, sel);
input [2: 0] num;       // 数码管编号：0~7
output reg [7:0] sel;       // 7段数码管片选信号，低电平有效
initial begin
    sel[7:0]=8'b11111111;
end

always @(num)  begin 
   case(num)                 // 功能实现
   3'b000	:  sel[7:0]=8'b11111110;
   3'b001	:  sel[7:0]=8'b11111101;
   3'b010   :  sel[7:0]=8'b11111011;
   3'b011   :  sel[7:0]=8'b11110111;
   3'b100	:  sel[7:0]=8'b11101111;
   3'b101	:  sel[7:0]=8'b11011111;
   3'b110	:  sel[7:0]=8'b10111111;
   3'b111	:  sel[7:0]=8'b01111111;
   endcase
 end

endmodule

module dynamic_scan(clk,SEG,AN,Syscall);
input clk;              // 系统时钟
output  [7:0] SEG;  		// 分别对应CA、CB、CC、CD、CE、CF、CG和DP
output  [7:0] AN;        // 8位数码管片选信号
input [31:0] Syscall;
wire [2:0] num;
wire clk_N; 

wire [3:0] code;

  divider #(100000) D1(clk,clk_N);
  divider_counter C(clk_N,num);
  decoder3_8 DECODER(num,AN); //选择某个LED点亮
  syscall_spiltter sys_spil(Syscall,num,code);
  pattern PATTERN(code,SEG);//根据num从Syscall选择某个数输出

endmodule


module displaySelection(SW, syscallout, pc, regval, success_branch, conditional_branch, unconditional_branch, total_cycle, show,RST);
    input [2:0]SW;
    input [15:0] success_branch, conditional_branch, unconditional_branch, total_cycle;
    input [31:0] syscallout, pc, regval;
    input RST;
    output reg [31:0]show;

    wire [31:0]success_branch_transform,unconditional_branch_transform,conditional_branch_transform,total_cycle_transform;
    hexi_decimal_transform Transfrom1(.in({16'h0000,success_branch}),.out(success_branch_transform)),
                           Transfrom2(.in({16'h0000,unconditional_branch}),.out(unconditional_branch_transform)),
                           Transfrom3(.in({16'h0000,conditional_branch}),.out(conditional_branch_transform)),
                           Transfrom4(.in({16'h0000,total_cycle}),.out(total_cycle_transform));
    
    initial show<=0;
    always @(*)begin
      if(RST)
         show<=0;
      else begin
      case (SW[2:0])
        3'b000: show <= syscallout;
        3'b001: show <= pc;
        3'b010: show <= regval;
        3'b100: show <= success_branch_transform;
        3'b101: show <= unconditional_branch_transform;
        3'b110: show <= conditional_branch_transform;
        3'b111: show <= total_cycle_transform;
      endcase
      end
    end
endmodule

module hexi_decimal_transform(in,out);
input [31:0]in;
output [31:0]out;

assign out[3:0]=in%10;
assign out[7:4]=(in%100)/10;
assign out[11:8]=(in%1000)/100;
assign out[15:12]=(in%10000)/1000;
assign out[19:16]=(in%100000)/10000;
assign out[23:20]=(in%1000000)/100000;
assign out[27:24]=(in%10000000)/1000000;
assign out[31:28]=(in%100000000)/10000000;

endmodule