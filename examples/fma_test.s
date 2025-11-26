.section .data
f_one:      
.word 0x3f800000      # 1.0f
f_two:      
.word 0x40000000      # 2.0f
f_three:    
.word 0x40400000      # 3.0f

.section .text

main: 
    la      x10, f_one

    flw     f0, 0(x10)           # f0 = 1.0
    fadd.s  f1, f0, f0           # f1 = 2.0 (should use forwarded f0)

    feq.s   x5, f1, f0           # x5 = (f1 == f0) ? 1 : 0  => expect 0

    fadd.s  f2, f0, f0           # f2 = 1.0 + 1.0 = 2.0 (immediate producer)
    feq.s   x6, f1, f2           # x6 = (f1 == f2) ? 1 : 0
    fmv.x.w x7, f1               # x7 = bit pattern of f1 (2.0f)
    add     x8, x7, x7           # use x7 right away (tests int hazard/forwarding)

    li      x9, 3                # x9 = 3
    fmv.w.x f3, x9               # f3 gets bit pattern of x9

    # fcvt.w.s x11, f3, rtz  ; some assemblers require rm, some infer default
    fcvt.w.s x11, f3            # x11 = (int) f3
    add      x12, x11, x11      # use x11 right away


    la      x13, f_one
    flw     f10, 0(x13)          # f10 = 1.0
    la      x14, f_two
    flw     f11, 0(x14)          # f11 = 2.0
    la      x15, f_three
    flw     f12, 0(x15)          # f12 = 3.0

    fadd.s  f10, f10, f10        # f10 = 1.0 + 1.0 = 2.0
    fadd.s  f11, f11, f11        # f11 = 2.0 + 2.0 = 4.0
    # rs3 producer
    fadd.s  f12, f12, f12        # f12 = 3.0 + 3.0 = 6.0

    fmadd.s f13, f10, f11, f12   # uses f10 (rs1), f11 (rs2), f12 (rs3)

    fcvt.w.s x20, f13            # x20 should be 14

end:
