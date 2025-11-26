.data
# Matrix A (2x2) stored in row-major order
matA:   
.double 1.0, 2.0, 3.0, 4.0
# Matrix B (2x2) stored in row-major order
matB:   
.double 2.0, 0.0, 1.0, 2.0
# Result Matrix C (2x2)
matC:   
.double 0.0, 0.0, 0.0, 0.0
size:   
.word 2  # Dimension N=2
.text

main:
    # Load base addresses
    la      x10, matA       # Base A
    la      x11, matB       # Base B
    la      x12, matC       # Base C
    la      x13, size
    lw      x13, 0(x13)     # x13 = N (Dimension)

    # i Loop (Row of C)
    li      x5, 0           # i = 0

loop_i:
    bge     x5, x13, done   # if i >= N, done

    # j Loop (Col of C)
    li      x6, 0           # j = 0

loop_j:
    bge     x6, x13, next_i # if j >= N, next row

    # k Loop (Dot Product)
    li      x7, 0           # k = 0
    fcvt.d.w f0, x0         # sum = 0.0 (f0 is accumulator)

loop_k:
    bge     x7, x13, store_c # if k >= N, store sum

    # Calculate Address A[i][k] = BaseA + (i*N + k) * 8
    mul     x28, x5, x13    # x28 = i * N
    add     x28, x28, x7    # x28 = i*N + k (RAW Hazard: x28 used immediately)
    slli    x28, x28, 3     # x28 = Offset * 8
    add     x29, x10, x28   # x29 = BaseA + Offset (RAW Hazard: x28 used immediately)
    
    # Calculate Address B[k][j] = BaseB + (k*N + j) * 8
    mul     x30, x7, x13    # x30 = k * N
    add     x30, x30, x6    # x30 = k*N + j
    slli    x30, x30, 3     # x30 = Offset * 8
    add     x31, x11, x30   # x31 = BaseB + Offset

    fld     f1, 0(x29)      # Load A[i][k]
    fld     f2, 0(x31)      # Load B[k][j]
    
    fmadd.d f0, f1, f2, f0  # sum += A * B (accumulate in f0)

    addi    x7, x7, 1       # k++
    jal     x0, loop_k

store_c:
    # Calculate Address C[i][j] = BaseC + (i*N + j) * 8
    mul     x28, x5, x13    # i * N
    add     x28, x28, x6    # i * N + j
    slli    x28, x28, 3     # Offset * 8
    add     x29, x12, x28   # Address C
    
    # Store result
    fsd     f0, 0(x29)      # C[i][j] = sum

    addi    x6, x6, 1       # j++
    jal     x0, loop_j

next_i:
    addi    x5, x5, 1       # i++
    jal     x0, loop_i

done:
    # Expected Result C:
    # [ 4.0, 4.0 ]
    # [ 10.0, 8.0 ]
    la      x12, matC        # Reload Base C (safety)
    fld     f10, 16(x12)     # Load double at offset 16
