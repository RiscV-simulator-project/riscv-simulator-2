.data
size:   
.word 4
vecA:   
.double 1.5, 2.0, 3.5, 4.0
vecB:   
.double 2.0, 3.0, 2.0, 0.5
result: 
.double 0.0
.text

main:
la      x10, vecA
la      x11, vecB
la      x12, size
lw      x12, 0(x12)    # Loop count

# f0 will hold the accumulator (dot product result)
fcvt.d.w f0, x0        

loop:
beq     x12, x0, done

fld     f1, 0(x10)     # Load A[i]
fld     f2, 0(x11)     # Load B[i]

# Fused Multiply-Add: f0 = (f1 * f2) + f0
# Tests 3-register operand hazard detection
fmadd.d f0, f1, f2, f0 

addi    x10, x10, 8    # Increment A pointer
addi    x11, x11, 8    # Increment B pointer
addi    x12, x12, -1   # Decrement count
jal     x0, loop


done:
la      x13, result
fsd     f0, 0(x13)     # Store final result

# Expected Result: (1.5*2)+(2*3)+(3.5*2)+(4*0.5) = 3+6+7+2 = 18.0
