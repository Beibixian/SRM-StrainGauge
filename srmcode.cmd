-heap  0x800
-stack 0x1000

MEMORY
{
	// IPC interrupt setting register
	IPC_ISR_REG:	o = 0x02620244		l = 0x00000004

	// Shared L2 1MB
	DDR3_INIT_CODE (RWX):	o = 0x0C000000	l = 0x00018000	// for initialization code of C6657
	USER_SRAM_1_4:			o = 0x0C018000  l = 0x000E8000	// for user code/data required middle speed

	// LL2 on Core 0
	KNL0_SRAM_1_4:			o = 0x10800000	l = 0x000FFF80	// for real time WAVE process
	KNL0_BOOT_AREA:			o = 0x108FFF80	l = 0x0000007C	// for kernel 0 boot parameters
	KNL0_MAGIC_ADDRESS:		o = 0x108FFFFC	l = 0x00000004	// for kernel 0 magic address

	// LL2 on Core 1
	CTRL_PARAS:				o = 0x11800000	l = 0x00000100	// for control parameters between cores
	KNL1_SRAM_1_2:			o = 0x11800100	l = 0x0003FF00	// for kernel 1 process
	CSL_INTC:				o = 0x11840000	l = 0x00010000	// for kernel 1 & user interrupt process
	USER_MAGIC_ADDRESS (R):	o = 0x118FFF80	l = 0x00000004	// for user magic address
	KNL1_BOOT_AREA:			o = 0x118FFF84	l = 0x00000078	// for kernel 1 boot parameters
	KNL1_MAGIC_ADDRESS:		o = 0x118FFFFC	l = 0x00000004	// for kernel 1 magic address

	// L1D & L1P on Core 1
    USER_SRAM_2_4:		o = 0x11E00000	l = 0x00008000	// L1P for user code required higher speed
    USER_SRAM_3_4:		o = 0x11F00000	l = 0x00008000	// L1D for user data required higher speed

	// Boot ROM
	TI_BOOT_ROM:		o = 0x20B00000	l = 0x00020000	// boot rom area reserved by TI

	// External DDR3, upto 2GB per core
	KNL0_SRAM_2_4: 		o = 0x80000000  l = 0x000fe000	// for kennel 0 process which could be booted
	USER_SRAM_4_4:		o = 0x80100000  l = 0x01800000	// for user code/data required lower speed which could be booted
	KNL0_SRAM_3_4: 		o = 0x82000000  l = 0x0D000000	// for kennel 0 data which could not be booted
	KNL0_SRAM_4_4: 		o = 0x8F000000  l = 0x01000000	// for kennel 0 / peos share erea( non-chached )
	KNL1_SRAM_2_2: 		o = 0x90000000  l = 0x10000000	// for kennel 1 data which could not be booted
}

SECTIONS
{
	.csl_vect	 	>	CSL_INTC 		, align = 128
	.text           >	USER_SRAM_1_4	, align = 128
	.switch         >	USER_SRAM_1_4	, align = 128
	.cinit          >	USER_SRAM_1_4	, align = 128
	.cio            >	USER_SRAM_1_4
	.const          >	USER_SRAM_1_4	, align = 128

    .neardata   	>	USER_SRAM_3_4
	.stack          >   USER_SRAM_3_4
    .rodata     	>	USER_SRAM_3_4
	.sysmem         >   USER_SRAM_1_4
    .bss        	>   USER_SRAM_3_4
	.far            >   USER_SRAM_1_4
	.fardata        >   USER_SRAM_1_4

    .CODE_ON_HIGHER_SPEED	>	USER_SRAM_2_4	, align = 128
    .DATA_ON_HIGHER_SPEED	>	USER_SRAM_3_4	, align = 128

    .CODE_ON_LOWER_SPEED	>	USER_SRAM_4_4	, align = 128
    .DATA_ON_LOWER_SPEED	>	USER_SRAM_4_4	, align = 128

    .DATA_ON_MIDDLE_SPEED	>	USER_SRAM_1_4	, align = 128
    
	.FLASH_ROM_RW_BUF 		> 	KNL1_SRAM_2_2						// for writing flashrom
    .L2_buffers     		>	USER_SRAM_3_4	, align = 128		// for uppbus using in peos
    .CAN_LIB				>	USER_SRAM_1_4	, align = 128		// for can using in peos

	.text_2					>	USER_SRAM_2_4	, align = 128
	{
		-l = mwio4.lib <PEV_INV_LIB.obj EMIF16.obj timer_lib.obj PEV_PIO_LIB.obj ADC_LIB.obj mwintc.obj csl2_intcHwControl.obj gpio_int.obj csl2_intcOpen.obj _csl2_intcIntrEnDisRes_eabi.obj _csl2_intcDispatcher.obj _csl2_intcDispatcher.obj > (.text)
	}

	.data_2					>	USER_SRAM_3_4	, align = 128
	{
		-l = mwio4.lib <PEV_INV_LIB.obj ADC_LIB.obj gpio_int.obj timer_lib.obj > (.far)
		-l = mwio4.lib <csl2_intcHwControl.obj > (.switch)
		-l = mwio4.lib <gpio_int.obj > (.fardata)
		-l = mwio4.lib <_mwsintbl.obj EMIF16.obj> (.const)
	}

}


