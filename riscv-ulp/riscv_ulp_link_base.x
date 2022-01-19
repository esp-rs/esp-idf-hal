PROVIDE(UserSoft = DefaultHandler);
PROVIDE(SupervisorSoft = DefaultHandler);
PROVIDE(MachineSoft = DefaultHandler);
PROVIDE(UserTimer = DefaultHandler);
PROVIDE(SupervisorTimer = DefaultHandler);
PROVIDE(MachineTimer = DefaultHandler);
PROVIDE(UserExternal = DefaultHandler);
PROVIDE(SupervisorExternal = DefaultHandler);
PROVIDE(MachineExternal = DefaultHandler);

PROVIDE(DefaultHandler = DefaultInterruptHandler);
PROVIDE(ExceptionHandler = DefaultExceptionHandler);

ENTRY(reset_vector)

MEMORY
{
    ram(RW) : ORIGIN = 0, LENGTH = _ram_size
}

PROVIDE(_stext = ORIGIN(ram));

SECTIONS
{
    .text.dummy (NOLOAD) :
    {
        /* This section is intended to make _stext address work */
        . = ABSOLUTE(_stext);
    } > ram

    .text _stext :
    {
        KEEP(*(.init)); // Default reset vector must link to offset 0x0
        KEEP(*(.start.rust));
        KEEP(*(.trap.rust));

        *(.text .text.*);
    } > ram

    .rodata ALIGN(4):
    {
        *(.srodata .srodata.*);
        *(.rodata .rodata.*);

        /* 4-byte align the end (VMA) of this section.
        This is required by LLD to ensure the LMA of the following .data
        section will have the correct alignment. */
        . = ALIGN(4);
    } > ram

    .data ALIGN(4):
    {
        /* Must be called __global_pointer$ for linker relaxations to work. */
        PROVIDE(__global_pointer$ = . + 0x800);
        *(.sdata .sdata.* .sdata2 .sdata2.*);
        *(.data .data.*);
        . = ALIGN(4);
    } > ram

    .bss ALIGN(4) :
    {
        *(.sbss .sbss.* .bss .bss.*);
        . = ALIGN(4);
    } > ram

    /* fake output .got section */
    /* Dynamic relocations are unsupported. This section is only used to detect
       relocatable code in the input files and raise an error if relocatable code
       is found */
    .got (INFO) :
    {
        KEEP(*(.got .got.*));
    }

    .eh_frame (INFO) :
    {
        KEEP(*(.eh_frame))
    }

    .eh_frame_hdr (INFO) :
    {
        *(.eh_frame_hdr)
    }

    _stack_top = ORIGIN(ram) + LENGTH(ram);
}