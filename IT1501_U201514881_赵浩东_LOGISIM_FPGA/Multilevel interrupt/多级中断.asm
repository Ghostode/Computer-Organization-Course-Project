#interupt demo  main program 
#1st section, auto decrement counter and display
#2nd section: ccmb instruction test
.text
#设置栈顶指针
addi $sp,$zero,512

addi $s1,$zero,0x200      #initial nubmer
addi $v0,$zero,34    
counter_branch:
add $a0,$0,$s1          
syscall                 #display number
addi $s1,$s1,-1         #decrement
bne $s1,$zero,counter_branch
addi $v0,$zero,10
syscall                 #pause
############################################
# insert your ccmb benchmark program here!!!
#C1 instruction
addi $s5,$zero,0 
addi $s6,$zero,0
addi $s7,$zero,0  
addi $t0,$zero,1     #sllv ��λ����
addi $t1,$zero,3     #sllv ��λ����
addi $s1,$zero, 0x876     #
sll $s1,$s1,20     #

add $a0,$0,$s1           
addi $v0,$zero,34         # system call for print
syscall                  # print

addi $t3,$zero,8

srlv_branch:
srlv $s1,$s1,$t0     #����1λ
srlv $s1,$s1,$t1     #����3λ
add $a0,$0,$s1          
addi $v0,$zero,34         # system call for print
syscall                  # print
addi $t3,$t3, -1    
bne $t3,$zero,srlv_branch   #ѭ��8��

addi   $v0,$zero,10         # system call for exit
syscall                  # we are out of here.   


#C2 instruction
addi $s5,$zero,0 
addi $s6,$zero,0
addi $s7,$zero,0  
 
addi $t0,$zero,1     
addi $t1,$zero,3     
addi $s1,$zero,  0x876     

add $a0,$0,$s1           
addi $v0,$zero,34         # system call for print
syscall                  # print

addi $t3,$zero,8

sllv_branch:
sllv $s1,$s1,$t0     #????????
sllv $s1,$s1,$t1     #????????
add $a0,$0,$s1          
addi $v0,$zero,34         # system call for print
syscall                  # print
addi $t3,$t3, -1    
bne $t3,$zero,sllv_branch

addi   $v0,$zero,10         # system call for exit
syscall                  # we are out of here.   


#Mem instruction
addi $t1,$zero,0     #init_addr 
addi $t3,$zero,16     #counter

#Ԥ��д�����ݣ�ʵ���ǰ��ֽ�˳�����? 0x81,82,84,86,87,88,89.......�Ȳ�����
ori $s1,$zero, 0x8483  #
addi $s2,$zero, 0x0404  #
sll $s1,$s1,16
sll $s2,$s2,16
ori $s1,$s1, 0x8281  #    ע��һ�������MIPS���ô�˷��?
addi $s2,$s2, 0x0404  #   init_data= 0x84838281 next_data=init_data+ 0x04040404

lh_store:
sw $s1,($t1)
add $s1,$s1,$s2   #data +1
addi $t1,$t1,4    # addr +4  
addi $t3,$t3,-1   #counter
bne $t3,$zero,lh_store

addi $t3,$zero,32
addi $t1,$zero,0    # addr  
lh_branch:
lh $s1,($t1)     #����ָ��
add $a0,$0,$s1          
addi $v0,$zero,34         
syscall                  # print
addi $t1,$t1, 2    
addi $t3,$t3, -1    
bne $t3,$zero,lh_branch

addi   $v0,$zero,10         # system call for exit
syscall                  # we are out of here.   

#Branch instruction
addi $s1,$zero,15  #��ʼֵ
bgez_branch:
add $a0,$0,$s1          
addi $v0,$zero,34         
syscall                  # �����ǰ�?
addi $s1,$s1,-1
bgez $s1,bgez_branch   #����ָ��

addi   $v0,$zero,10         #ͣ��ָ��
syscall                  # ϵͳ����

addi   $v0,$zero,10    
syscall                  #退出


#1号中断
sw $s0,0($sp)
addi $sp,$sp,4
sw $s1,0($sp)
addi $sp,$sp,4
sw $s2,0($sp)
addi $sp,$sp,4
sw $s3,0($sp)
addi $sp,$sp,4
sw $s4,0($sp)
addi $sp,$sp,4
sw $s5,0($sp)
addi $sp,$sp,4
sw $s6,0($sp)
addi $sp,$sp,4
sw $s7,0($sp)
addi $sp,$sp,4
sw $v0,0($sp)
addi $sp,$sp,4
sw $a0,0($sp)
addi $sp,$sp,4

	
mfc0 $s2,$0
sw $s2,0($sp)
addi $sp,$sp,4
mfc0 $s2,$1
sw $s2,4($sp)
addi $sp,$sp,4


addi $s2,$zero,1
mtc0 $s2,$1
break

addi $s6,$zero,1       #中断号 1,2,3   不同中断号显示值不一样

addi $s4,$zero,6      #循环次数初始值  
addi $s5,$zero,1       #计数器累加值
###################################################################
#                逻辑左移，每次移位4位 
# 显示区域依次显示 0x000000X6 0x00000X06 0x0000X006 0x000X0006 ... X0000006  00000006 依次循环6次
###################################################################
IntLoop1:
add $s0,$zero,$s6   

IntLeftShift1:       


sll $s0, $s0, 4  
or $s3,$s0,$s4
add    $a0,$0,$s3       #display $s0
addi   $v0,$0,34         # display hex
syscall                 # we are out of here.   

bne $s0, $zero, IntLeftShift1
sub $s4,$s4,$s5      #循环次数递减
bne $s4, $zero, IntLoop1


	break
    addi $sp,$sp,-4
	lw $s2,0($sp)
	mtc0 $s2,$1
    addi $sp,$sp,-4
	lw $s2,0($sp)
	mtc0 $s2,$0
	
addi $sp,$sp,-4
lw $a0,0($sp)
addi $sp,$sp,-4
lw $v0,0($sp)
addi $sp,$sp,-4
lw $s7,0($sp)
addi $sp,$sp,-4
lw $s6,0($sp)
addi $sp,$sp,-4
lw $s5,0($sp)
addi $sp,$sp,-4
lw $s4,0($sp)
addi $sp,$sp,-4
lw $s3,0($sp)
addi $sp,$sp,-4
lw $s2,0($sp)
addi $sp,$sp,-4
lw $s1,0($sp)
addi $sp,$sp,-4
lw $s0,0($sp)
	
	eret


#2号中断
sw $s0,0($sp)
addi $sp,$sp,4
sw $s1,0($sp)
addi $sp,$sp,4
sw $s2,0($sp)
addi $sp,$sp,4
sw $s3,0($sp)
addi $sp,$sp,4
sw $s4,0($sp)
addi $sp,$sp,4
sw $s5,0($sp)
addi $sp,$sp,4
sw $s6,0($sp)
addi $sp,$sp,4
sw $s7,0($sp)
addi $sp,$sp,4
sw $v0,0($sp)
addi $sp,$sp,4
sw $a0,0($sp)
addi $sp,$sp,4

mfc0 $s2,$0
sw $s2,0($sp)
mfc0 $s2,$1
sw $s2,4($sp)
addi $sp,$sp,8
	
addi $s2,$zero,3
mtc0 $s2,$1
break	



addi $s6,$zero,2       #中断号 1,2,3   不同中断号显示值不一样

addi $s4,$zero,6      #循环次数初始值  
addi $s5,$zero,1       #计数器累加值
###################################################################
#                逻辑左移，每次移位4位 
# 显示区域依次显示 0x000000X6 0x00000X06 0x0000X006 0x000X0006 ... X0000006  00000006 依次循环6次
###################################################################
IntLoop2:
add $s0,$zero,$s6   

IntLeftShift2:       


sll $s0, $s0, 4  
or $s3,$s0,$s4
add    $a0,$0,$s3       #display $s0
addi   $v0,$0,34         # display hex
syscall                 # we are out of here.   

bne $s0, $zero, IntLeftShift2
sub $s4,$s4,$s5      #循环次数递减
bne $s4, $zero, IntLoop2


break
addi $sp,$sp,-4
lw $s2,0($sp)
mtc0 $s2,$1
addi $sp,$sp,-4
lw $s2,0($sp)
mtc0 $s2,$0
	
addi $sp,$sp,-4
lw $a0,0($sp)
addi $sp,$sp,-4
lw $v0,0($sp)
addi $sp,$sp,-4
lw $s7,0($sp)
addi $sp,$sp,-4
lw $s6,0($sp)
addi $sp,$sp,-4
lw $s5,0($sp)
addi $sp,$sp,-4
lw $s4,0($sp)
addi $sp,$sp,-4
lw $s3,0($sp)
addi $sp,$sp,-4
lw $s2,0($sp)
addi $sp,$sp,-4
lw $s1,0($sp)
addi $sp,$sp,-4
lw $s0,0($sp)
	
eret

#3号中断
sw $s0,0($sp)
addi $sp,$sp,4
sw $s1,0($sp)
addi $sp,$sp,4
sw $s2,0($sp)
addi $sp,$sp,4
sw $s3,0($sp)
addi $sp,$sp,4
sw $s4,0($sp)
addi $sp,$sp,4
sw $s5,0($sp)
addi $sp,$sp,4
sw $s6,0($sp)
addi $sp,$sp,4
sw $s7,0($sp)
addi $sp,$sp,4
sw $v0,0($sp)
addi $sp,$sp,4
sw $a0,0($sp)
addi $sp,$sp,4

mfc0 $s2,$0
	sw $s2,0($sp)
	mfc0 $s2,$1
	sw $s2,4($sp)
	addi $sp,$sp,8
	
	
	addi $s2,$zero,7
	mtc0 $s2,$1
	break	



addi $s6,$zero,3       #中断号 1,2,3   不同中断号显示值不一样

addi $s4,$zero,6      #循环次数初始值  
addi $s5,$zero,1       #计数器累加值
###################################################################
#                逻辑左移，每次移位4位 
# 显示区域依次显示 0x000000X6 0x00000X06 0x0000X006 0x000X0006 ... X0000006  00000006 依次循环6次
###################################################################
IntLoop3:
add $s0,$zero,$s6   

IntLeftShift3:       


sll $s0, $s0, 4  
or $s3,$s0,$s4
add    $a0,$0,$s3       #display $s0
addi   $v0,$0,34         # display hex
syscall                 # we are out of here.   

bne $s0, $zero, IntLeftShift2
sub $s4,$s4,$s5      #循环次数递减
bne $s4, $zero, IntLoop2


	break
    addi $sp,$sp,-4
	lw $s2,0($sp)
	mtc0 $s2,$1
    addi $sp,$sp,-4
	lw $s2,0($sp)
	mtc0 $s2,$0
	
addi $sp,$sp,-4
lw $a0,0($sp)
addi $sp,$sp,-4
lw $v0,0($sp)
addi $sp,$sp,-4
lw $s7,0($sp)
addi $sp,$sp,-4
lw $s6,0($sp)
addi $sp,$sp,-4
lw $s5,0($sp)
addi $sp,$sp,-4
lw $s4,0($sp)
addi $sp,$sp,-4
lw $s3,0($sp)
addi $sp,$sp,-4
lw $s2,0($sp)
addi $sp,$sp,-4
lw $s1,0($sp)
addi $sp,$sp,-4
lw $s0,0($sp)
	
	eret
