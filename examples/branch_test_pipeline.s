.data
array:  
.dword 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
size:   
.dword 10
.text

main:
la      x10, array     # Base address
la      x11, size
ld      x11, 0(x11)    # Size
li      x12, 0         # Loop index
li      x13, 0         # Even count

loop:
bge     x12, x11, exit # Exit if index >= size

ld      x5, 0(x10)     # Load number
andi    x6, x5, 1      # Check if odd (x6 = 1 if odd, 0 if even)

# This branch will toggle Taken/Not-Taken every iteration
bne     x6, x0, is_odd 


is_even:
addi    x13, x13, 1    # Increment even count
# Fall through to next iteration logic

is_odd:
addi    x10, x10, 8    # Next array element
addi    x12, x12, 1    # Increment index
jal     x0, loop

exit:
add     x10, x13, x0   # Return count of even numbers (should be 5)
