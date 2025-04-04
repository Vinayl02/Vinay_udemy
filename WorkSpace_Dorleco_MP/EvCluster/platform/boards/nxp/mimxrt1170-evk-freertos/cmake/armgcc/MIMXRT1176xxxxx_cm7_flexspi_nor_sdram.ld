/*
** ###################################################################
**     Processors:          MIMXRT1176AVM8A_cm7
**                          MIMXRT1176CVM8A_cm7
**                          MIMXRT1176DVMAA_cm7
**
**     Compiler:            GNU C Compiler
**     Reference manual:    IMXRT1170RM, Rev 1, 02/2021
**     Version:             rev. 1.0, 2020-12-29
**     Build:               b210709
**
**     Abstract:
**         Linker file for the GNU C Compiler
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2021 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
** ###################################################################
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Specify the memory areas */
MEMORY
{
  m_flash_config        (RX)  : ORIGIN = 0x30000400, LENGTH = 0x00000C00
  m_ivt                 (RX)  : ORIGIN = 0x30001000, LENGTH = 0x00001000
  m_interrupts          (RX)  : ORIGIN = 0x30002000, LENGTH = 0x00000400
  m_text                (RX)  : ORIGIN = 0x30002400, LENGTH = 0x00FFDC00
  m_qacode              (RX)  : ORIGIN = 0x00000000, LENGTH = 0x00040000
  m_data                (RW)  : ORIGIN = 0x80000000, LENGTH = 0x02600000
  m_preload             (RW)  : ORIGIN = 0x82600000, LENGTH = 0x00A00000
  m_ncache              (RW)  : ORIGIN = 0x83000000, LENGTH = 0x01000000
  m_data2               (RW)  : ORIGIN = 0x20000000, LENGTH = 0x00040000
  OCRAM2                (RW)  : ORIGIN = 0x202C0000, LENGTH = 512K
}

/* Define output sections */
SECTIONS
{
  __NCACHE_REGION_START = ORIGIN(m_ncache);
  __NCACHE_REGION_SIZE  = LENGTH(m_ncache);

  .flash_config :
  {
    . = ALIGN(4);
    __FLASH_BASE = .;
    KEEP(* (.boot_hdr.conf))     /* flash config section */
    . = ALIGN(4);
  } > m_flash_config

  ivt_begin= ORIGIN(m_flash_config) + LENGTH(m_flash_config);

  .ivt : AT(ivt_begin)
  {
    . = ALIGN(4);
    KEEP(* (.boot_hdr.ivt))           /* ivt section */
    KEEP(* (.boot_hdr.boot_data))     /* boot section */
    KEEP(* (.boot_hdr.dcd_data))      /* dcd section */
    . = ALIGN(4);
  } > m_ivt

  /* The startup code goes first into internal RAM */
  .interrupts :
  {
    __VECTOR_TABLE = .;
    __Vectors = .;
    . = ALIGN(4);
    KEEP(*(.isr_vector))     /* Startup code */
    . = ALIGN(4);
  } > m_interrupts

  __VECTOR_RAM = __VECTOR_TABLE;
  __RAM_VECTOR_TABLE_SIZE_BYTES = 0x0;

  /* The program code and other data goes into internal RAM */
  .text :
  {
    . = ALIGN(4);
    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)               /* glue arm to thumb code */
    *(.glue_7t)              /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
  } > m_text

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > m_text

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_text

 .ctors :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       from the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > m_text

  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > m_text

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > m_text

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > m_text

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > m_text

  . = ALIGN(4);
  __etext = .;    /* define a global symbol at end of code */
  __DATA_ROM = .; /* Symbol is used by startup for data initialization */

  .data :
  {
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;      /* create a global symbol at data start */
    *(m_usb_dma_init_data)
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
  } > m_data AT> m_text

  .ncache.init :
  {
    __noncachedata_start__ = .;   /* create a global symbol at ncache data start */
    *(NonCacheable.init)
    . = ALIGN(4);
    __noncachedata_init_end__ = .;   /* create a global symbol at initialized ncache data end */
  } > m_ncache AT> m_text

  .ncache :
  {
    *(NonCacheable)
    . = ALIGN(4);
    __noncachedata_end__ = .;     /* define a global symbol at ncache data end */
    __noncacheheap_start = .;     /* define a gobal start symbol for non cacheable heap, needed for frame buffers */
    __noncacheheap_end = ORIGIN(m_ncache) + LENGTH(m_ncache);
  } > m_ncache

  /* Uninitialized data section */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    . = ALIGN(4);
    __START_BSS = .;
    __bss_start__ = .;
    *(m_usb_dma_noninit_data)
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    __END_BSS = .;
  } > m_data

  QulFontResourceData :
  {
    . = ALIGN(8);
    *(QulFontResourceData)
    . = ALIGN(8);
  } > m_text

  QulModuleResourceData :
  {
    . = ALIGN(8);
    __ModuleResourceDataCacheStart = .;
    *(QulModuleResourceData)
    . = ALIGN(8);
    __ModuleResourceDataCacheEnd = .;
  } > m_data AT> m_text

  __ModuleResourceDataStart = LOADADDR(QulModuleResourceData);

  QulResourceData :
  {
    . = ALIGN(8);
    *(QulResourceData)
  } > m_text

  QulPreprocessCache :
  {
    . = ALIGN(8);
    __preprocessCacheStart = .;
    . += DEFINED(__qul_preprocess_cache_size__) ? __qul_preprocess_cache_size__ : 512K;
    . = ALIGN(8);
    __preprocessCacheEnd = .;
  } > m_data

  .heap :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    __HeapBase = .;
    *(.heap*)
    __HeapLimit = .;
    __heap_limit = .; /* Add for _sbrk */
  } > m_data

  __preloadRamStart = ORIGIN(m_preload);
  __preloadRamEnd = ORIGIN(m_preload) + LENGTH(m_preload);

  .stack (COPY):
  {
    *(.stack*)
  } > OCRAM2

  /* Initializes stack on the end of block */
  __StackTop   = ORIGIN(OCRAM2) + LENGTH(OCRAM2);
  __StackLimit = ORIGIN(OCRAM2);
  PROVIDE(__stack = __StackTop);

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

