.section .data
A:      
.float 2.0
B:      
.float 4.0
RES:
.float 0.0
.section .text

main:
    la      t0, A           # address of A
    la      t1, B           # address of B
    flw     ft1, 0(t1)      # LOAD: ft1 = B
    fadd.s  ft0, ft1, ft1 
    flw     ft0, 0(t0)
    fadd.s  ft2, ft0, ft0 
    la      t2, RES
    fsw     ft2, 0(t2)      # store result
    flw     ft0, 0(t2)      # store result
    fsw     ft0, 0(t2)      # store result