.section .data
f_single:    
.float 3.14
f_as_double: 
.double 0.0    # here we’ll store the single → double result
f_as_single: 
.float  0.0    # here we’ll store the double → single result
.section .text

main:
    # Load the single‐precision float
    la      x10, f_single
    flw     f0, 0(x10)          # f0 = 3.14 (32-bit float)

    # Convert single to double
    fcvt.d.s f1, f0             # f1 = double-precision representation of f0
    # Store the double result
    la      x11, f_as_double
    fsd     f1, 0(x11)          # save f1 into f_as_double

    # convert back double to single
    fcvt.s.d f2, f1             # f2 = single-precision version of f1
    # Store the single result
    la      x12, f_as_single
    fsw     f2, 0(x12)          # save f2 into f_as_single

