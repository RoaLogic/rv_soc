
/* Specify memory areas */
MEMORY
{
    ROM  (rx)         : ORIGIN = 0x00100000, LENGTH = 4K
    RAM  (rwx)        : ORIGIN = 0x00110000, LENGTH = 4K
    DRAM (rwx)        : ORIGIN = 0x80000000, LENGTH = 8M
}

OUTPUT_ARCH("riscv")

/* entry point */
ENTRY( _start)

SECTIONS{
	

	/*startup address for rv12*/
	 . = 0x00100000;
	  .init ALIGN(128): {
      		KEEP(*(SORT_NONE(.init)))   
 	 }	


	/* TEXT section */
	.text ALIGN(4) :
	{
	  *(.text )	/* text sections */
	  *(.text*)
	  *(.rodata )   /* read only data sections */
	  *(.rodata*)
	  _etext = .;   /* global signal end of text */
	} >ROM 		/* placement in ROM */


	
  	/** Cleanup section 
  	.fini           : { KEEP(*(SORT_NONE(.fini))) }
  
  	/** Thread Data sections 
  	.tdata          : { *(.tdata .tdata.*) }
  	.tbss           : { *(.tbss .tbss.*) *(.tcommon) }
  	.preinit_array  :
  	{
    		PROVIDE_HIDDEN (__preinit_array_start = .);
    		KEEP (*(.preinit_array))
    		PROVIDE_HIDDEN (__preinit_array_end = .);
  	}


  	/** Initialisation 
  	.init_array     :
  	{
   	 	PROVIDE_HIDDEN (__init_array_start = .);
    		KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    		PROVIDE_HIDDEN (__init_array_end = .);
  	}
  	.fini_array     :
  	{
    		PROVIDE_HIDDEN (__fini_array_start = .);
    		KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
    		PROVIDE_HIDDEN (__fini_array_end = .);
  	}

	/** Constructors 
  	.ctors          :
  	{
    		KEEP (*(SORT(.ctors.*)))
    		KEEP (*(.ctors))
  	}

  	/** Destructors 
  	.dtors          :
  	{
   		KEEP (*(SORT(.dtors.*)))
    		KEEP (*(.dtors))
  	}
	*/

	/* DATA section */
	  . = ALIGN(4); /* align data for transfer rom to ram function */
	_sidata = .;
	.data : AT(_sidata)
	{
	   _sdata = .;  /* global signal start data */
	   *(.data )
	   *(.data*)
	   _gp = . + 0x800; /* global pointer for linker relaxation */
	   *(.sdata )
	   *(.sdata*)
	  . = ALIGN(4);
	   _edata   = . ;
	} >RAM

	/* BSS section */	
	.bss :
	{
	  _sbss = .;
		
	  __bss_start__ = _sbss;
	  *(.bss)
	  *(.bss*)
	  *(.sbss)
	  *(.sbss*)
	  *(COMMON)
	  . = ALIGN(4);
	  _ebss = .;	
	} >RAM

	_end = .;

  __stack = ORIGIN(RAM) + LENGTH(RAM);
}
