#ifndef BLE_FIND_ME_H
#define BLE_FIND_ME_H

#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_ble.h"
#include "FreeRTOS.h"
#include "task.h"

/******************************************************************************
 * Function prototypes
 *****************************************************************************/
void enter_low_power_mode(void);
void mcwdt_interrupt_handler(void);
void stack_event_handler(uint32_t event, void* eventParam);
void main_fsm(void* pvParameters);

cy_stc_ble_conn_handle_t app_conn_handle;

#endif  /* BLE_FIND_ME_H */


/* END OF FILE [] */
