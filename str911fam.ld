MEMORY
{
  rom (rx) : ORIGIN = 0x00000000, LENGTH = 512k
  ram (rw) : ORIGIN = 0x04000000, LENGTH = 96k
}

SECTIONS
{
  /* .text: instructions */
  .text :
  {
    *(.text.exception_vector)
    *(.text .text.*)
    *(.glue_7t) *(.glue_7) *(.vfp11_veneer) *(.v4_bx)
  } >rom =0


  .eh_frame : ALIGN (4)
  {
    KEEP (*(.eh_frame)) *(.eh_frame.*)
  } >rom


  /* .ARM.exidx is sorted, so has to go in its own output section.  */
  PROVIDE_HIDDEN (__exidx_start__ = .);
  .ARM.exidx :
  {
    *(.ARM.exidx*)
  } >rom
  PROVIDE_HIDDEN (__exidx_end__ = .);


  /* .rodata: read-only data (constants) */
  .rodata . : ALIGN (4)
  {
    *(.rodata .rodata.*)

    . = ALIGN(4);
    KEEP(*(.init))

    . = ALIGN(4);
    __preinit_array_start__ = . ;
    KEEP (*(.preinit_array))
    __preinit_array_end__ = . ;

    . = ALIGN(4);
    __init_array_start__ = . ;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end__ = . ;

    . = ALIGN(4);
    KEEP(*(.fini))

    . = ALIGN(4);
    __fini_array_start__ = . ;
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    __fini_array_end__ = . ;
  } >rom

  . = ALIGN(4);
  __rom_end__ = . ;


  /* .data: initialized data */
  .data : AT (__rom_end__)
  {
    __data_start__ = . ;
    *(.data .data.*)
    *(.ram)
  } >ram

  . = ALIGN(4);
  __data_end__ = . ;


  /* .bss: uninitialized data */
  .bss :
  {
    __bss_start__ = . ;
    *(.bss .bss.*)
  } >ram

  . = ALIGN(4);
  __bss_end__ = . ;


  .stab 0 (NOLOAD) : { *(.stab) }
  .stabstr 0 (NOLOAD) : { *(.stabstr) }
  /* DWARF debug sections.
   * Symbols in the DWARF debugging sections are relative to
   * the beginning of the section so we begin them at 0.
   */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* DWARF 2.1 */
  .debug_ranges   0 : { *(.debug_ranges) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }

  .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
  .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) }
}