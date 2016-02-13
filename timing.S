// This function delays execution of the program for APPROXIMATELY
// "t_microsends" microseconds. Note that time spent in an interrupt routine
// will NOT count towards this delay.
// void MicroWait(uint32_t t_microseconds)

.extern micro_wait_multiplier_

.text
.align 2
.global MicroWait
MicroWait:
  ldr r3, =micro_wait_multiplier_
  ldr r3, [r3]
  mul r0, r3, r0
  MICRO_WAIT_LOOP:
  subs r0, r0, #1
  bne MICRO_WAIT_LOOP
  bx lr
