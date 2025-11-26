.data
n:      
.dword 10          # Calculate 10th Fibonacci number
result: 
.dword 0

.text
main:
la      x5, n
ld      x6, 0(x5)      # x6 = n
li      x1, 0          # Fib(0)
li      x2, 1          # Fib(1)
li      x3, 2          # Counter starts at 2

loop:
bge     x3, x6, done   # if counter >= n, finish

# RAW Hazard Chain:
# x4 depends on x1, x2 immediately.
# x1 depends on x2 immediately.
# x2 depends on x4 immediately.
add     x4, x1, x2     # x4 = x1 + x2 (Forwarding needed if prev inst wrote x1/x2)
add     x1, x0, x2     # Move x2 to x1
add     x2, x0, x4     # Move x4 to x2 (Forwarding needed from ALU result)

addi    x3, x3, 1      # increment counter
jal     x0, loop

done:
la      x5, result
sd      x2, 0(x5)      # Store result
add     x10, x2, x0    # Move result to x10 
