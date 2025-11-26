.data
.text
    addi  x12, x0, 1           # x12 = 1
    addi  x15, x0, 0           # x15 = 0
    add   x14, x12, x11        # x14 = 1 + 0 = 1
L2:
    beq   x15, x14, L1         # 0 == 1? NO → Fall through
    ld    x13, 8(x13)          # ← NOW THIS EXECUTES!
    beq   x12, x13, L2         # ← Load-use hazard on x13!
L1:
    and   x13, x15, x13
    ld    x11, 8(x13)
    sd    x13, 0(x15)