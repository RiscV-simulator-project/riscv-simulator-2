.data
.text

main:
    # Initial Setup
    addi x1, x0, 10      # x1 = 10
    addi x2, x0, 20      # x2 = 20
    addi x3, x0, 5       # x3 = 5

    # Test 1: EX/MEM Forwarding

    add  x4, x1, x2      # x4 = 10 + 20 = 30

    # Test 2: MEM/WB Forwarding & Load-Use Hazard
    sw   x3, 0(x0)       # Store 5 at address 0
    lw   x5, 0(x0)       # Load 5 into x5 (Stall required here!)

    # Test 3: Arithmetic using the loaded value
    add  x6, x5, x5      # x6 = 5 + 5 = 10

    # Test 4: EX/MEM Forwarding and load-use hazard 
    sub  x7, x4, x6      # x7 = 30 - 10 = 20

