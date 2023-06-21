#define __ASSEMBLY__
#include <target_mem_config.h>

MEMORY {
#if (ICCM_SIZE != 0)
	ICCM :		ORIGIN = ICCM_START,	LENGTH = ICCM_SIZE
#endif
#if (DCCM_SIZE !=0)
	DCCM :		ORIGIN = DCCM_START,	LENGTH = DCCM_SIZE
#endif
#if (EXT_ROM_SIZE != 0)
	EXT_ROM :	ORIGIN = EXT_ROM_START,	LENGTH = EXT_ROM_SIZE
#endif
#if (EXT_TEXT_XIP_SIZE != 0)
    EXT_TEXT_XIP : ORIGIN = EXT_TEXT_XIP_START, LENGTH = EXT_TEXT_XIP_SIZE
#endif
#if (EXT_RAM_SIZE != 0)
	EXT_RAM :	ORIGIN = EXT_RAM_START,	LENGTH = EXT_RAM_SIZE
#endif
}

ENTRY(_start)

SECTIONS {

	GROUP : {
		.init_bootstrap:{
			*(.init_vector)
			*(.init_bootstrap)
		}
	} > REGION_ROM

#if (EXT_TEXT_XIP_SIZE != 0)
	GROUP : {
		.text_xip ALIGN(4): {
			_f_text_xip = .;
			*flash.o (.text .text.* .rodata)
			*dw_ssi_obj.o (.text .text.* .rodata)
			*clkgen.o (.text .text.* .rodata)
			*dw_ssi.o (.text .text.* .rodata)
			*FLASH_TYPE.o (.text .text.* .rodata)
#if (NOR_FLASH_CFI == 1)
			*cfi.o(.text .text.* .rodata)
#else
			*sfdp.o(.text .text.* .rodata)
#endif
			*alps_timer.o (.text .text.* .rodata)
			*arc_timer.o (.text .text.* .rodata)
			*xip_hal.o (.text .text.* .rodata)
			*arc_exception.o (.text .text.* .rodata)
			_e_text_xip = .;
		}
	} > EXT_TEXT_XIP AT > REGION_ROM
#endif

	GROUP : {
		.vector ALIGN(1024): {
			_f_vector = .;
			*(.vector)
			_e_vector = .;
		}

		.text ALIGN(4): {
			_f_text = .;
			*(TYPE text)
			_e_text = .;
		}

		.rodata  ALIGN(4): {
			_f_rodata = .;

			_fctors = .;
			*(.ctors*)
			_ectors = .;
			_fdtors = .;
			*(.dtors*)
			_edtors = .;
			_feh_frame = .;
			*(.eh_frame*)
			_eeh_frame = .;

			*(TYPE lit)

			_e_rodata = .;
		}

	} > REGION_ROM

#if (EXT_TEXT_XIP_SIZE != 0)
	GROUP : {
		.nvm_header ALIGN(8): {
			*(.nvm_header)
		}
	} > REGION_ROM
#else
	GROUP : {
		.nvm_header ALIGN(8): {
			*(.nvm_header)
		}
	} > REGION_RAM AT > REGION_ROM
#endif

	GROUP : {
		.data ALIGN(8): {
			_f_data = .;
			_f_sdata = .;
			*(.sdata)
			_e_sdata = .;
			*(TYPE data)
		}
		.tls ALIGN(8): {
			*(.tls*)
			_e_data = .;
		}
	} > REGION_RAM AT > REGION_ROM

	GROUP (NOLOAD) : {
		.bss ALIGN(8): {
			_f_bss = .;
			*(TYPE bss)
			_e_bss = .;
		}
		.stack ALIGN(4) SIZE(_STACKSIZE): {}
		.heap? ALIGN(4) SIZE(_HEAPSIZE): {}
	} > REGION_RAM

	_f_stack = ADDR(.stack);
	_e_stack = ADDR(.stack) + SIZEOF(.stack);
	_f_heap = ADDR(.heap);
	_e_heap = ADDR(.heap) + SIZEOF(.heap);

	_load_addr_text = LOADADDR(.text);
	_load_addr_rodata = LOADADDR(.rodata);
	_load_addr_data = LOADADDR(.data);
#if (EXT_TEXT_XIP_SIZE != 0)
	_load_addr_text_xip = LOADADDR(.text_xip);
#endif

}
