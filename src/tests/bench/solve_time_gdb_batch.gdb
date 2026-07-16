set pagination off
set confirm off
target remote :3333
break jerk_trajectory.cpp:101
continue
# DWT/SCS register access requires the core to be actively running normal
# code (not halted mid-WFI/sleep, which some debug-port memory accesses
# fail against) -- enable the cycle counter HERE, right at the breakpoint
# hit, rather than immediately after attach (attach can land mid-WFI in
# CODAL's idle loop, where 0xE000EDFC was observed to be inaccessible).
set *(unsigned int*)0xE000EDFC = *(unsigned int*)0xE000EDFC | 0x01000000
set *(unsigned int*)0xE0001004 = 0
set *(unsigned int*)0xE0001000 = *(unsigned int*)0xE0001000 | 0x00000001
printf "HIT pc=%p\n", $pc
printf "T1_CYCCNT=%u\n", *(unsigned int*)0xE0001004
next
printf "T2_CYCCNT=%u\n", *(unsigned int*)0xE0001004
delete
detach
quit
