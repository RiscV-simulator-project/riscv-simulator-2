.data
num: .dword 5, 2, 4, 1, 10, 6
.text 
main:
    la x11, num
    ld x10, 0(x11)	#number of elements in array 
    addi x11, x11, 8	#increment input pointer to move to start of array 
    ld x9, 0(x11) 
    addi x11, x11, 8 
    addi x10, x10, -1 

loop: 
    beq x10, x0, done 
    ld x5, 0(x11)	#arr[i]
    bge x5, x9, greater 
    addi x11, x11, 8
    addi x10, x10, -1 
    jal x0, loop 

greater: 
    add x9, x5, x0 
    addi x11, x11, 8
    addi x10, x10, -1 
    jal x0, loop 

done: 
    add x10, x9, x0 
	ecall 
