/* Linker script for the STM32G474RE (Nucleo-G474RE). Ours: ST's cmsis-device-g4 package
 * ships none for GCC (ADR 0016).
 *
 * Memory, from the STM32G474xE datasheet (DS12288) memory map: 512 KB flash at 0x08000000;
 * SRAM1 (80 KB) + SRAM2 (16 KB) contiguous at 0x20000000. CCM SRAM (32 KB) is NOT used yet;
 * keeping RAM at 96 KB is the conservative reading. [UNCLEAR] confirm sizes against RM0440.
 *
 * Symbols required by ST's startup_stm32g474xx.s: _sidata _sdata _edata _sbss _ebss _estack.
 * No heap: hard rule 2. The __heap symbols nano.specs' sbrk would want are deliberately
 * absent, so any malloc that sneaks in fails at link time instead of at run time.
 */

ENTRY(Reset_Handler)

/* Main stack, grows down from the top of SRAM. Initial guess: the deepest path is the motor-
 * loop ISR preempting a main-loop frame encode; 4 KB leaves a wide margin. Measure with a
 * stack-paint at bring-up. */
_Min_Stack_Size = 0x1000;

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*)
    *(.glue_7) *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
  } > FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata) *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM

  /* Fails the link if static data leaves less than the stack reservation. */
  ._stack_reserve (NOLOAD) :
  {
    . = ALIGN(8);
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM

  /* ST's convention: drop whatever library sections no rule above has already claimed (debug
   * and comment sections). Input sections go to the FIRST matching rule, so library code and
   * data matched by .text/.data above are kept. */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
