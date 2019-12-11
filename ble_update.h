/******************************************************************************
* File Name: ble_findme.h
*
* Description: This file is public interface of ble_findme.c
*
* Related Document: Readme.md
*
*******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_ble.h"
#include "FreeRTOS.h"
#include "task.h"
/******************************************************************************
 * Include guard
 *****************************************************************************/
#ifndef BLE_FIND_ME_H
#define BLE_FIND_ME_H

uint8_t data[4096];

/*******************************************************************************
* Macros
********************************************************************************/
#define MCWDT_INTR_PRIORITY     (7u)

/******************************************************************************
 * Function prototypes
 *****************************************************************************/
void ble_task_init(void);
void ble_task_process(void*);
void enter_low_power_mode(void);
void mcwdt_interrupt_handler(void);

typedef struct {
	char* name;
	int name_len;
	uint8_t *serviceUUID;
	uint8_t servUUID_len;
} advInfo_t;

typedef enum {
	MCU_STATE_DEEP_SLEEP,
	MCU_STATE_SHUT_DOWN_BLUETOOTH,
	MCU_STATE_CONNECTING,
	MCU_STATE_UPDATING_INFO,
	MCU_STATE_UPDATING_DISPLAY,
	MCU_STATE_UPDATING_DISPLAY_FINISHED,
	MCU_STATE_ERROR
} mcu_state_t;

advInfo_t currentAdvInfo;
mcu_state_t curr_state;

bool mcwdt_intr_flag;
bool gpio_intr_flag;
cy_stc_ble_conn_handle_t app_conn_handle;

#endif  /* BLE_FIND_ME_H */


/* END OF FILE [] */
