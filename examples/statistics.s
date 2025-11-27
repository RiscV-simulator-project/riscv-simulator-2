.data
num: .dword 5, 2, 4, 1, 10, 6   # first = count (5), rest = elements
choice: .dword 2                 # 1=mean, 2=median, 3=mode

.text
main:
    # load array length and pointer to first element
    la   x11, num
    ld   x10, 0(x11)           # x10 = number of elements (N)
    addi x11, x11, 8           # x11 = &num[1] = start of array

    # load choice
    la   x28, choice
    ld   x29, 0(x28)           # x29 = choice
    addi x30, x0, 1            # x30 = 1

    beq  x29, x30, mean        # choice == 1 -> mean
    addi x30, x30, 1           # x30 = 2
    beq  x29, x30, median      # choice == 2 -> median
    addi x30, x30, 1           # x30 = 3
    beq  x29, x30, mode        # choice == 3 -> mode

    # invalid choice -> just exit
    jal  x0, exit

# Mean: result in x10
mean:
    add  x31, x10, x0          # preserve original count

    addi x6, x0, 0             # x6 = sum = 0

mean_sum:
    beq  x10, x0, mean_done    # if count == 0, done
    ld   x7, 0(x11)            # load next element
    add  x6, x6, x7            # sum += element
    addi x11, x11, 8           # advance pointer
    addi x10, x10, -1          # decrement remaining count
    jal  x0, mean_sum

mean_done:
    div  x10, x6, x31          # x10 = sum / N (integer division)
    jal  x0, exit

# Median sort: bubble sort array in-place
median_sort:
    addi x5, x10, -1           # x5 = N-1
    bge  x0, x5, sort_done     # if N <= 1, no sorting

outer_sort:
    addi x12, x0, 0            # i = 0
    addi x14, x11, 0           # x14 = base pointer

sorting:
    bge  x12, x5, next_pass    # if i >= limit, next pass
    ld   x6, 0(x14)            # a[i]
    ld   x7, 8(x14)            # a[i+1]
    bge  x7, x6, no_swap       # if a[i+1] >= a[i], no swap

    # swap a[i] and a[i+1]
    sd   x7, 0(x14)
    sd   x6, 8(x14)

no_swap:
    addi x12, x12, 1           # i++
    addi x14, x14, 8           # move pointer to next element
    jal  x0, sorting

next_pass:
    addi x5, x5, -1            # reduce effective length
    blt  x0, x5, outer_sort    # while x5 > 0, do another pass

sort_done:
    jalr x0, 0(x1)             # return to caller

# Median: result in x10
median:
    jal  x1, median_sort       # sort array in-place

    # N is still in x10
    andi x13, x10, 1
    beq  x13, x0, median_even  # if (N & 1) == 0 -> even

    # Odd N: median = a[N/2] (0-based index)
    srai x13, x10, 1           # idx = N / 2
    slli x12, x13, 3           # offset = idx * 8
    add  x12, x11, x12
    ld   x10, 0(x12)           # x10 = median
    jal  x0, median_done

median_even:
    # Even N: median = (a[N/2 - 1] + a[N/2]) / 2
    srai x13, x10, 1           # half = N / 2
    addi x13, x13, -1          # idx_low = half - 1
    slli x12, x13, 3           # offset for idx_low
    add  x12, x11, x12
    ld   x6, 0(x12)            # low = a[idx_low]
    ld   x7, 8(x12)            # high = a[idx_low + 1]
    add  x6, x6, x7            # low + high
    srai x10, x6, 1            # divide by 2 (integer)
    # fall through

median_done:
    jal  x0, exit

# Mode: result in x10
mode:
    beq  x10, x0, mode_zero    # if N == 0, mode = 0

    jal  x1, median_sort       # reuse sort function; array sorted ascending

    # After sort: x11 base, x10 = N
    addi x5, x10, -1           # remaining elements after first
    addi x14, x11, 0           # pointer
    ld   x6, 0(x14)            # current value
    addi x14, x14, 8           # pointer to next element

    addi x7, x0, 1             # current run length
    add  x8, x6, x0            # best value
    addi x9, x0, 1             # best count

mode_loop:
    beq  x5, x0, mode_after_loop   # no more elements

    ld   x13, 0(x14)           # next value
    beq  x13, x6, mode_same    # same as current value?

    # run ended: compare current run (x7) with best (x9)
    bge  x9, x7, mode_new_run  # if best >= current, keep best
    add  x9, x7, x0            # best count = current
    add  x8, x6, x0            # best value = current value

mode_new_run:
    add  x6, x13, x0           # new current value
    addi x7, x0, 1             # current run length = 1
    addi x14, x14, 8           # next element
    addi x5, x5, -1            # remaining--
    jal  x0, mode_loop

mode_same:
    addi x7, x7, 1             # extend current run
    addi x14, x14, 8           # next element
    addi x5, x5, -1            # remaining--
    jal  x0, mode_loop

mode_after_loop:
    # After finishing scan, compare final run with best
    bge  x9, x7, mode_done_value
    add  x9, x7, x0
    add  x8, x6, x0

mode_done_value:
    add  x10, x8, x0           # x10 = mode value
    jal  x0, exit

mode_zero:
    addi x10, x0, 0            # define mode(0 elements) = 0
    jal  x0, exit

exit:
    ecall
