#include "flash_counter.h"

#include <stdio.h>

#include "cy_serial_flash_qspi.h"
#include "cycfg_qspi_memslot.h"

void flash_counter_init() {
    cy_rslt_t result = cy_serial_flash_qspi_init(smifMemConfigs[MEM_SLOT_NUM], CYBSP_QSPI_D0,
              CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
              CYBSP_QSPI_SCK, CYBSP_QSPI_SS, QSPI_BUS_FREQUENCY_HZ);

    if(result != CY_RSLT_SUCCESS) {
		printf("[INFO] Serial Flash initialization failed \r\n");
    }

}

uint16_t get_flash_counter_value() { uint8_t buf[FLASH_COUNTER_SIZE];
    uint16_t counter = 0;

    cy_rslt_t result = cy_serial_flash_qspi_read(FLASH_COUNTER_LOCATION, FLASH_COUNTER_SIZE, buf);
    if(result != CY_RSLT_SUCCESS) {
		printf("[INFO] Serial Flash read failed \r\n");
    }
    memcpy((uint8_t*)&counter, buf, FLASH_COUNTER_SIZE);

    return counter;
}

void set_flash_counter_value(uint16_t counter) {
    size_t sectorSize = cy_serial_flash_qspi_get_erase_size(FLASH_COUNTER_LOCATION);
    cy_rslt_t result = cy_serial_flash_qspi_erase(FLASH_COUNTER_LOCATION, sectorSize);

    if(result != CY_RSLT_SUCCESS) {
		printf("[INFO] Serial Flash erase failed \r\n");
    }

	uint8_t buf[FLASH_COUNTER_SIZE];
    memcpy(buf, (uint8_t*)&counter, FLASH_COUNTER_SIZE);

    result = cy_serial_flash_qspi_write(FLASH_COUNTER_LOCATION, FLASH_COUNTER_SIZE, buf);
    if(result != CY_RSLT_SUCCESS) {
		printf("[INFO] Serial Flash write failed \r\n");
    }
}

void increment_flash_counter() {
	uint16_t counter = get_flash_counter_value();
	set_flash_counter_value(counter + 1);
}
