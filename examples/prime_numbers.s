.data
num:
.dword 23          
.text

main:
    la   x5, num       #x5= &num
    ld   x10, 0(x5)    #x10= num       

    addi x28, x0, 2    #x28= 2

    # if num<2, not prime 
    blt  x10, x28, not_prime

    # x6= num-1 (candidate divisor) 
    addi x6, x10, -1

loop:
    # if x6<2, prime (no divisors found)
    blt  x6, x28, is_prime

    # x7 = num % x6
    rem  x7, x10, x6
    # ifremainder==0, not prime
    beq  x7, x0, not_prime

    # x6--
    addi x6, x6, -1

    jal  x0, loop

is_prime:
    addi x10, x0, 1    # a0 = 1 (prime)
    ecall 

not_prime:
    addi x10, x0, 0    # a0 = 0 (not prime)
    ecall 
