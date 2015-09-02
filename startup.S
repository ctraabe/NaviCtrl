// =============================================================================
// User specified values:

// Stack sizes
.set USER_MODE_STACK_SIZE, 1024*4
.set FIQ_MODE_STACK_SIZE, 64*4
.set IRQ_MODE_STACK_SIZE, 1024*4
.set SUPERVISOR_MODE_STACK_SIZE, 64*4
.set ABORT_MODE_STACK_SIZE, 64*4
.set UNDEFINED_MODE_STACK_SIZE, 64*4

// Set the following when working in JTAG debug mode. You have also to uncomment
// the "DEBUG" define in 91x_conf.h
.set DEBUG_MODE, 0

// Set the following when working in buffered mode. You have also to
// uncomment the "Buffered" define in 91x_conf.h
.set BUFFERED_MODE, 0

// No need to set up FMI since it is handled by the MK bootloader.
.set FMI_SETUP, 0


// =============================================================================
// Device definitions:

// Standard definitions of mode bits and flags in PSRs:
.set USER_MODE, 0x10
.set FIQ_MODE, 0x11
.set IRQ_MODE, 0x12
.set SUPERVISOR_MODE, 0x13
.set ABORT_MODE, 0x17
.set UNDEFINED_MODE, 0x1B
.set SYSTEM_MODE, 0x1F

.set T_NIT, 0x20  // Thumb mode active
.set F_BIT, 0x40  // FIQ disabled
.set I_BIT, 0x80  // IRQ disabled

// STR9X specific definitions:
.set BOOT_BANK_BASE_ADDRESS, 0x00000000
.set BOOT_BANK_SIZE, 0x4  // 2^BOOT_BANK_SIZE * 32KB = 512KB
.set NON_BOOT_BANK_BASE_ADDRESS, 0x00080000
.set NON_BOOT_BANK_SIZE, 0x2  // 2^NON_BOOT_BANK_SIZE * 8KB = 32KB

.set SCU_BASE_ADDRESS, 0x5C002000
.set SCU_CLKCNTR_OFFSET, 0x00  // Clock control register offset
.set SCU_PLLCONF_OFFSET, 0x04  // PLL configuration register offset
.set SCU_SYSSTATUS_OFFSET, 0x08  // SCU status register offset
.set SCU_PCGR0_OFFSET, 0x14 // Peripheral clock gating register 0 offset
.set SCU_PRR0_OFFSET, 0x1C
.set SCU_SCR0_OFFSET, 0x34  // System configuration register 0 offset

.set FMI_BASE_ADDRESS, 0x54000000  // FMI base address
.set FMI_BBSR_OFFSET, 0x00  // Boot bank size register offset
.set FMI_NBBSR_OFFSET, 0x04  // Non-boot bank size register offset
.set FMI_BBADR_OFFSET, 0x0C  // Boot bank base address register offset
.set FMI_NBBADR_OFFSET, 0x10  // Non-boot bank base address register offset
.set FMI_CR_OFFSET, 0x18  // Control register offset

.set RAM_BASE_ADDRESS, 0x04000000
.set RAM_SIZE, 0x18000  // 96KB
.set STACK_TOP, RAM_BASE_ADDRESS + RAM_SIZE

.set VIC0_VAR_ADDRESS, 0xFFFFF030  // VIC0 current vector address register
.set VIC0_DVAR_ADDRESS, 0xFFFFF034  // VIC0 default vector address register
.set VIC1_VAR_ADDRESS, 0xFC000030  // VIC1 current vector address register
.set VIC1_DVAR_ADDRESS, 0xFC000034  // VIC0 default vector address register


// =============================================================================
// External definitions:

// Externally defined routines:
.extern main
.extern Undefined_Handler
.extern SWI_Handler
.extern Prefetch_Handler
.extern Abort_Handler
.extern FIQ_Handler
.extern DefaultVector_Handler


// =============================================================================
// Macros:

// Save the workspace plus the current return address lr_ mode into the stack.
.macro SAVE_CONTEXT $r0, $r4
  stmfd sp!, {r0-r4,lr}
.endm

//------------------------------------------------------------------------------
// Return to the instruction following the exception interrupt.
.macro RESTORE_CONTEXT $r0, $r4
  ldmfd sp!, {r0-r4,pc}^
.endm

//------------------------------------------------------------------------------
.macro COPY_SECTION sec_name, source, destination, destination_end
  ldr r1, =\source
  ldr r2, =\destination
  ldr r3, =\destination_end
_cplp_\sec_name:
  cmp r2, r3
  ldrlo r0, [r1], #4
  strlo r0, [r2], #4
  blo _cplp_\sec_name
.endm

//------------------------------------------------------------------------------
.macro CLEAR_SECTION sec_name, source, source_end
  mov r0, #0
  ldr r1, =\source
  ldr r2, =\source_end
_cllp_\sec_name:
  cmp r1, r2
  strlo r0, [r1], #4
  blo _cllp_\sec_name
.endm


// =============================================================================
// Program:

.section .text.exception_vector,"ax",%progbits
  ldr pc, =ResetHandler
  ldr pc, =Undefined_Handler
  ldr pc, =SWIHandler
  ldr pc, =Prefetch_Handler
  ldr pc, =Abort_Handler
  nop  // Reserved vector
  ldr pc, =IRQHandler

//------------------------------------------------------------------------------
FIQHandler:
  sub lr, lr, #4  // Update the link register
  SAVE_CONTEXT r0, r4
  bl FIQ_Handler  // Branch to FIQ_Handler
  RESTORE_CONTEXT r0, r4

//------------------------------------------------------------------------------
SWIHandler:
  SAVE_CONTEXT r0, r4
  bl SWI_Handler  // Branch to SWI_Handler
  RESTORE_CONTEXT r0, r4

//------------------------------------------------------------------------------
IRQHandler:
  sub lr, lr ,#4
  SAVE_CONTEXT r0, r4
  LDR r0, =VIC0_VAR_ADDRESS
  LDR r0, [r0]  // Read the handler address from VIC0 Vector Address register
  BLX r0  // Branch with link to the handler
  RESTORE_CONTEXT r0, r4


//------------------------------------------------------------------------------
ResetHandler:

  ldr r0, =SCU_BASE_ADDRESS

  // Configure PLL to 48 MHZ (seems to be the only possibility)
  ldr r1, =0x000BC019  // PLL enabled with P = 3, N = 192, M = 25 (48 MHz)
  str r1, [r0, #SCU_PLLCONF_OFFSET]
WaitForPLLLock:
  ldr r1, [r0, #SCU_SYSSTATUS_OFFSET]
  ands r1, r1, #0x00000001  // PLL locked bit
  beq WaitForPLLLock  // Wait until PLL is Locked
  ldr r1, =0x00020000  // Set PLL as the master clock source
  str r1, [r0, #SCU_CLKCNTR_OFFSET]

  //Set up RAM
  ldr r1, =0x00000197  // 96 KB
  str r1, [r0, #SCU_SCR0_OFFSET]


//------------------------------------------------------------------------------
// This delay is added to let JTAG debuggers able to connect to STR91x micro
// then load the program even if a previously flash code is halting the CPU
// core. This may happen when the flash content is corrupt by containing a "bad"
// code like entering soon to IDLE or SLEEP low power modes.
.if DEBUG_MODE
  mov r0, #0x4000
DebugStartupLoop:
  subs r0, r0, #1
  subs r0, r0, #1
  subs r0, r0, #1
  subs r0, r0, #1
  subs r0, r0, #1
  subs r0, r0, #1
  subs r0, r0, #1
  subs r0, r0, #1
  bne DebugStartupLoop
.endif

//------------------------------------------------------------------------------
.if BUFFERED_MODE
  // Enable buffered mode.
  mrc p15, 0, r0, c1, c0, 0  // Read CP15 register 1 into r0
  orr r0, r0, #0x8  // Enable write buffer on AHB
  mcr p15, 0, r0, c1, c0, 0  // Write CP15 register 1
  // Write Buffer in ITCM may cause the Flash “write then read” command order to
  // reverse, causing a flash error. To maintain the correct order, bit 18
  // (instruction TCM order bit) in the Configuration Registers of the ARM966E-S
  // core must be set.
  mov r0, #0x40000
  mcr p15, 0x1, r0, c15, c1, 0  // Write CP15 register 1
.endif

//------------------------------------------------------------------------------
.if FMI_SETUP
  ldr r6, =FMI_BASE_ADDRESS
  ldr r7, =BOOT_BANK_SIZE
  str r7, [r6, #FMI_BBSR_OFFSET]
  ldr r7, =NON_BOOT_BANK_SIZE
  str r7, [r6, #FMI_NBBSR_OFFSET]
  ldr r7, =BOOT_BANK_BASE_ADDRESS
  mov r7, r7, LSR #0x2
  str r7, [r6, #FMI_BBADR_OFFSET]
  ldr r7, =NON_BOOT_BANK_BASE_ADDRESS
  mov r7, r7, LSR #0x2
  str r7, [r6, #FMI_NBBADR_OFFSET]
  ldr r7, =0x18  // Clear the status registers
  str r7, [r6, #FMI_CR_OFFSET]
  ldr r0, =SCU_BASE_ADDRESS
  ldr r1, =0x3  // Clear the status registers
  str r1, [r0, #SCU_SCR0_OFFSET]
.endif

//------------------------------------------------------------------------------
InitializeStackPointers:
  ldr r0, =STACK_TOP
  // Initialize the stack pointer for fast interrupt mode.
  msr CPSR_c, #FIQ_MODE | I_BIT | F_BIT
  mov sp, r0
  sub r0, r0, #FIQ_MODE_STACK_SIZE
  // Initialize the stack pointer for vectored interrupt mode.
  msr CPSR_c, #IRQ_MODE | I_BIT | F_BIT
  mov sp, r0
  sub r0, r0, #IRQ_MODE_STACK_SIZE
  // Initialize the stack pointer for abort mode.
  msr CPSR_c, #ABORT_MODE | I_BIT | F_BIT
  mov sp, r0
  sub r0, r0, #ABORT_MODE_STACK_SIZE
  // Initialize the stack pointer for undefined instruction mode.
  msr CPSR_c, #UNDEFINED_MODE | I_BIT | F_BIT
  mov sp, r0
  sub r0, r0, #UNDEFINED_MODE_STACK_SIZE
  // Initialize the stack pointer for supervisor mode.
  msr CPSR_c, #SUPERVISOR_MODE | I_BIT | F_BIT
  mov sp, r0
  sub r0, r0, #SUPERVISOR_MODE_STACK_SIZE
  // Initialize the stack pointer for user mode.
  msr CPSR_c, #SYSTEM_MODE
  mov sp, r0

//------------------------------------------------------------------------------
  // Initialize RAM
  COPY_SECTION data, __rom_end__, __data_start__, __data_end__

  // Clear 0-initialized data
  CLEAR_SECTION bss, __bss_start__, __bss_end__

//------------------------------------------------------------------------------
  ldr PC, =main
