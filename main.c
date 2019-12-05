/******************************************************************************
* File Name: main.c
*
* Related Document: See Readme.md
*
* Description: This code example demonstrates displaying graphics on an EInk
* display using EmWin graphics library.
*
*******************************************************************************
* Copyright (2018-2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
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
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp_types.h"
#include "cy_retarget_io.h"
#include "cybsp.h"
#include "cyhal.h"

#include "einkTask.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ble_update.h"
#include "cycfg_ble.h"
#include <limits.h>

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

void readMsg() {
	cy_stc_ble_gattc_read_req_t myVal = {
		.attrHandle = cy_ble_customCServ[CY_BLE_CUSTOMC_LINEDATA_SERVICE_INDEX].customServChar[CY_BLE_CUSTOMC_LINEDATA_DATA_CHAR_INDEX].customServCharHandle[0],
		.connHandle = cy_ble_connHandle[0]
	};

	if(Cy_BLE_GATTC_ReadCharacteristicValue(&myVal) != CY_BLE_SUCCESS) {
		printf("BLE GATTC read error \r\n");
	}
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
* 	1) Create task for E-Ink
* 	2) Schedule the task
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Configure switch SW2 as hibernate wake up source */
    Cy_SysPm_SetHibWakeupSource(CY_SYSPM_HIBPIN1_LOW);

    /* Unfreeze IO if device is waking up from hibernate */
    if(Cy_SysPm_GetIoFreezeStatus())
    {
        Cy_SysPm_IoUnfreeze();
    }

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, \
                                 CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }


    /* Initialize the User LEDs */
    result = cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED2, CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    ble_task_init();
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */

    printf("\x1b[2J\x1b[;H");

	printf("**********************************************************\r\n");
	printf("PSoC 6 MCU emWin E-Ink\r\n");
	printf("**********************************************************\r\n");

    eInkInit();
//	vTaskDelay(2000);

	__enable_irq();
//
//	if(Cy_MCWDT_Init(MCWDT_STRUCT0, &CYBSP_MCWDT_config) != 0) {
//		printf("Couldn't init MCWDT\r\n");
//	}
//	Cy_MCWDT_Enable(MCWDT_STRUCT0, CY_MCWDT_COUNTER0 | CY_MCWDT_COUNTER1, 93u);

	curr_state = MCU_STATE_CONNECTING;

	for(;;) {
		switch(curr_state) {
			case MCU_STATE_DEEP_SLEEP: {
				enter_low_power_mode();
				break;
			}
			case MCU_STATE_CONNECTING: {
				Cy_BLE_ProcessEvents();
				if(CY_BLE_CONN_STATE_CLIENT_DISCOVERED == Cy_BLE_GetConnectionState(app_conn_handle)) {
					curr_state = MCU_STATE_UPDATING_INFO;
					readMsg();
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_ON);
				} else {
					cyhal_gpio_toggle((cyhal_gpio_t)CYBSP_USER_LED1);
				}
				break;
			}
			case MCU_STATE_UPDATING_INFO: {
				Cy_BLE_ProcessEvents();
				break;
			}
			case MCU_STATE_UPDATING_DISPLAY: {
				//TODO: add forming display
				eInkTask();
				curr_state = MCU_STATE_DEEP_SLEEP;
				break;
			}
			case MCU_STATE_ERROR: {
				break;
			}
		}
	}

//	xTaskCreate(eInkTask,"eInkTask", 2000,  NULL,  2,  &update_scr_task);
//	xTaskCreate(ble_task_process, "ble_update", 2000, (void*) &update_scr_task, 1, NULL);
//	vTaskStartScheduler();

	while(1);// Will never get here
}
