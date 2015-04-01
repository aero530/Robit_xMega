/**
 * \file
 *
 * \brief User board initialization template
 *
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	ioport_configure_pin(GPIO_A1, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );
	ioport_configure_pin(LED_D5, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );
	
	ioport_configure_pin(GPIO_C0,	IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );
	ioport_configure_pin(GPIO_C3,	IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );
	
	ioport_configure_pin(GPIO_C1,	IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );
	ioport_configure_pin(GPIO_C2,	IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );

	ioport_configure_pin(GPIO_C5,	IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT );
	ioport_configure_pin(GPIO_C6,	IOPORT_DIR_INPUT );
	ioport_configure_pin(GPIO_C7,	IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT );
		

}
