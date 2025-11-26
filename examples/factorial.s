.data
num:
.dword 5

.text

main:
    la x6, num
    ld x5, 0(x6)
    addi x10, x0, 1

factorial:
    beq x5, x0, exit      # if n == 0, done
    mul x10, x10, x5      # result *= n
    addi x5, x5, -1       # n--
    jal x0, factorial

exit:
    ecall
