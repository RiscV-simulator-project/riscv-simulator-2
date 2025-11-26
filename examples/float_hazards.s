.section .data
a:      
.float 1.0        # first term a
r:      
.float 0.5        # common ratio r
n:      
.word  5          # number of terms n (integer)
sum:    
.float 0.0        # will hold the result sum
.section .text

main: 
    # Load a, r, n from memory

    # Load address of a, then load a into ft0
    la      t0, a                 # allowed pseudo: la -> (auipc+addi)
    flw     ft0, 0(t0)            # ft0 = a
    # Load address of r, then load r into ft1
    la      t1, r
    flw     ft1, 0(t1)            # ft1 = r

    # Load address of n, then load integer n into t3
    la      t2, n
    lw      t3, 0(t2)             # t3 = n (loop counter)
    fsgnj.s ft2, ft0, ft0         # ft2 = ft0  (term = a)
    # sum = 0.0 using sum = a - a
    fsub.s  ft3, ft0, ft0         # ft3 = a - a = 0.0  (sum = 0.0)

GP_LOOP:
    beq     t3, x0, GP_DONE       # if (t3 == 0) break
    fadd.s  ft3, ft3, ft2         # ft3 = ft3 + ft2  (sum += term)
    fmul.s  ft2, ft2, ft1         # ft2 = ft2 * ft1  (term *= r)
    addi    t3, t3, -1            # t3--
    jal     x0, GP_LOOP

GP_DONE:
    la      t0, sum
    fsw     ft3, 0(t0)            # store sum into memory
    ecall
