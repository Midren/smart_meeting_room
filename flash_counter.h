#ifndef FLASH_COUNTER_H_
#define FLASH_COUNTER_H_

#include "cybsp.h"
#include "cybsp_types.h"
#include "cyhal.h"
#include "cy_pdl.h"

void flash_counter_init();

uint16_t get_flash_counter_value();

void set_flash_counter_value(uint16_t counter);

void increment_flash_counter();

#define QSPI_BUS_FREQUENCY_HZ   (50000000lu)
#define MEM_SLOT_NUM            (0u)      /* Slot number of the memory to use */
#define FLASH_COUNTER_SIZE		(2u)	/* Number of bytes, which will count number of resets */
#define FLASH_COUNTER_LOCATION	(0x0u) /* location a reset counter in flash */



#endif /* FLASH_COUNTER_H_ */
