/******************************************************************************
* File Name: ble_findme.c
*
* Description: This file contains BLE related functions.
*
* Related Document: README.md
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
* code solely for use in connection with Cyptemperature sensor we discussed about wants to keep track of all devices that have connected to it in its lifetime (let’s ignore for a moment potential security issues). In that case, the Central should contain a GATT database itself. There, inside the GAP Serviceress's integrated circuit products.
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
*******************************************************************************/


/******************************************************************************
 * Include header files
 *****************************************************************************/
#include "ble_update.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "einkTask.h"

/*******************************************************************************
* Global Variables
********************************************************************************/


/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void ble_init(void);
static void bless_interrupt_handler(void);
static void stack_event_handler(uint32 event, void* eventParam);
static void mcwdt_init(void);
static void ble_ias_callback(uint32 event, void *eventParam);


void findAdvInfo(uint8_t *adv, uint8_t len) {
	memset(&currentAdvInfo, 0, sizeof(currentAdvInfo));

	for(uint8_t i=0; i<len;) {
		switch(adv[i+1]) {
		case 0x07:
			currentAdvInfo.serviceUUID = &adv[i+2];
			currentAdvInfo.servUUID_len = adv[i]-1;
			break;
		case 0x09:
			currentAdvInfo.name = (char*) &adv[i+2];
			currentAdvInfo.name_len = adv[i]-1;
			break;
		}
		i = i + adv[i]+1;
	}
}
/*******************************************************************************
* Function Name: ble_findme_init
********************************************************************************
* Summary:
* This function initializes the BLE and MCWDT.
*
*******************************************************************************/
void ble_task_init(void)
{
	mcwdt_intr_flag = false;
	gpio_intr_flag = false;
	alert_level = CY_BLE_NO_ALERT;

    /* Configure BLE */
    ble_init();
    
    /* Configure MCWDT */
//    mcwdt_init();
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
* Function Name: ble_findme_process
********************************************************************************
* Summary:
*  This function processes the BLE events and configures the device to enter
*  low power mode as required.
*
*******************************************************************************/
void ble_task_process(void* pvParameters)
{
//	update_scr_task = *((TaskHandle_t*) pvParameters);
	printf("Task state: %d\r\n", eTaskGetState(update_scr_task));
	for(;;) {
//		printf("Count: %d\r\n", (int) Cy_MCWDT_GetCountCascaded(CYBSP_MCWDT_HW));
		if(mcwdt_intr_flag) {
			printf("Tried to print from irq\r\n");
			mcwdt_intr_flag = false;
		}
		switch(curr_state) {
			case MCU_STATE_DEEP_SLEEP: {
				printf("Entering low power mode \r\n");
				Cy_BLE_Disable();
				curr_state = MCU_STATE_SHUT_DOWN_BLUETOOTH;
				break;
			}
			case MCU_STATE_CONNECTING: {
				Cy_BLE_ProcessEvents();
				if(Cy_BLE_GetConnectionState(app_conn_handle) == CY_BLE_CONN_STATE_CLIENT_DISCOVERED) {
					curr_state = MCU_STATE_UPDATING_INFO;
					readMsg();
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_ON);
				} else {
					cyhal_gpio_toggle((cyhal_gpio_t)CYBSP_USER_LED1);
				}
				break;
			}
			case MCU_STATE_SHUT_DOWN_BLUETOOTH:
			case MCU_STATE_UPDATING_INFO: {
				Cy_BLE_ProcessEvents();
				break;
			}
			case MCU_STATE_UPDATING_DISPLAY: {
				//TODO: add forming display
				vTaskResume(update_scr_task);
				while(curr_state != MCU_STATE_UPDATING_DISPLAY_FINISHED) {
					taskYIELD();
				}
				curr_state = MCU_STATE_DEEP_SLEEP;
				break;
			}
			case MCU_STATE_ERROR: {
				break;
			}
		}
		taskYIELD();
	}
}


/*******************************************************************************
* Function Name: ble_init
********************************************************************************
* Summary:
*  This function initializes the BLE and registers IAS callback function.
*
*******************************************************************************/
static void ble_init(void)
{
    static const cy_stc_sysint_t bless_isr_config =
    {
      /* The BLESS interrupt */
      .intrSrc = bless_interrupt_IRQn,

      /* The interrupt priority number */
      .intrPriority = BLESS_INTR_PRIORITY
    };

    /* Hook interrupt service routines for BLESS */
    (void) Cy_SysInt_Init(&bless_isr_config, bless_interrupt_handler);

    /* Store the pointer to blessIsrCfg in the BLE configuration structure */
    cy_ble_config.hw->blessIsrConfig = &bless_isr_config;

    /* Registers the generic callback functions  */
    Cy_BLE_RegisterEventCallback(stack_event_handler);

    /* Initializes the BLE host */
    Cy_BLE_Init(&cy_ble_config);

    /* Enables BLE */
    Cy_BLE_Enable();

    /* Enables BLE Low-power mode (LPM)*/
    Cy_BLE_EnableLowPowerMode();

    /* Register IAS event handler */
    Cy_BLE_IAS_RegisterAttrCallback(ble_ias_callback);
}


/******************************************************************************
* Function Name: bless_interrupt_handler
*******************************************************************************
* Summary:
*  Wrapper function for handling interrupts from BLESS.
*
******************************************************************************/
static void bless_interrupt_handler(void)
{
    Cy_BLE_BlessIsrHandler();
}


/*******************************************************************************
* Function Name: stack_event_handler
********************************************************************************
*
* Summary:
*   This is an event callback function to receive events from the BLE Component.
*
* Parameters:
*  uint32 event:      event from the BLE component
*  void* eventParam:  parameters related to the event
*
*******************************************************************************/
static void stack_event_handler(uint32_t event, void* eventParam)
{
    switch(event)
    {
        /**********************************************************************
         * General events
         *********************************************************************/

        /* This event is received when the BLE stack is started */
        case CY_BLE_EVT_STACK_ON:
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
            printf("[INFO] : Starting scan \r\n");
            Cy_BLE_GAPC_StartScan(CY_BLE_SCANNING_FAST, 0);
            break;
        }


        /* This event indicates completion of Set LE event mask */
        case CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE:
        {
            printf("[INFO] : Set LE mask event mask command completed\r\n");
            break;
        }

        /* This event indicates set device address command completed */
        case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
        {
            printf("[INFO] : Set device address command has completed \r\n");
            break;
        }

        /* This event indicates set Tx Power command completed */
        case CY_BLE_EVT_SET_TX_PWR_COMPLETE:
        {
            printf("[INFO] : Set Tx power command completed\r\n");
            break;
        }

        /* This event indicates BLE Stack Shutdown is completed */
        case CY_BLE_EVT_STACK_SHUTDOWN_COMPLETE:
        {
            printf("[INFO] : BLE shutdown complete\r\n");
			enter_low_power_mode();
            break;
        }


        /**********************************************************************
         * GAP events
         *********************************************************************/

        case CY_BLE_EVT_GAPC_SCAN_PROGRESS_RESULT: {
        	printf("Device ");
        	cy_stc_ble_gapc_adv_report_param_t *scanProgressParam = (cy_stc_ble_gapc_adv_report_param_t *) eventParam;
        	printf("BD Address = ");
        	for(unsigned int i = 0; i < CY_BLE_BD_ADDR_SIZE; i++) {
        		printf("%02X", scanProgressParam->peerBdAddr[i]);
        	}
        	printf(" Length = %d ", scanProgressParam->dataLen);
        	findAdvInfo(scanProgressParam->data, scanProgressParam->dataLen);
        	if(currentAdvInfo.name_len != 0) {
        		printf("%.*s",currentAdvInfo.name_len, currentAdvInfo.name);
        	}

        	printf("\r\n");

        	if(currentAdvInfo.name_len > 0) {
				currentAdvInfo.name[currentAdvInfo.name_len] = '\0';

				if(!strcmp(currentAdvInfo.name, "BLE UART Target"))
				{
					printf("Found LineData Service \r\n");
					cy_stc_ble_bd_addr_t connectAddr;
					memcpy(&connectAddr.bdAddr[0], &scanProgressParam->peerBdAddr[0], CY_BLE_BD_ADDR_SIZE);
					connectAddr.type = scanProgressParam->peerAddrType;
					Cy_BLE_GAPC_ConnectDevice(&connectAddr, 0);
					Cy_BLE_GAPC_StopScan();
				}
        	}
        	break;
        }


        /**********************************************************************
         * GATT events
         *********************************************************************/

        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device
         */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
//            app_conn_handle = *(cy_stc_ble_conn_handle_t *)eventParam;
            printf("[INFO] : GATT device connected\r\n");
            Cy_BLE_GATTC_StartDiscovery(cy_ble_connHandle[0]);
            break;
        }

        /* This event is generated at the GAP Peripheral end after disconnection */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            printf("[INFO] : GATT device disconnected\r\n");
            break;
        }

        /* This event indicates that the 'GATT MTU Exchange Request' is received */
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
        {
            printf("[INFO] : GATT MTU Exchange Request received \r\n");
            break;
        }

        /* This event received when GATT read characteristic request received */
        case CY_BLE_EVT_GATTC_ERROR_RSP:
        {
        	printf("[INFO] : GATTC error response\r\n");
        	break;
        }

        /* This event received when GATT read characteristic request received */
        case CY_BLE_EVT_GATTC_WRITE_RSP:
        {
        	printf("[INFO] : GATTC write response\r\n");
            break;
        }

        case CY_BLE_EVT_GATTC_READ_RSP:
        {
        	printf("[INFO] : GATTC read response\r\n");
        	cy_stc_ble_gattc_read_rsp_param_t * readRspParam = (cy_stc_ble_gattc_read_rsp_param_t*)eventParam;
        	printf("[INFO] : Read len - %d\r\n", readRspParam->value.len);
        	for(int i = 0; i < readRspParam->value.len; i++) {
        		printf("%x", readRspParam->value.val[i]);
        	}
        	printf("\r\n");
        	curr_state = MCU_STATE_UPDATING_DISPLAY;
            break;
        }
        default:
        {
            printf("[INFO] : BLE Event 0x%lX\r\n", (unsigned long) event);
        }
    }

}


/*******************************************************************************
* Function Name: ble_ias_callback
********************************************************************************
* Summary:
*  This is an event callback function to receive events from the BLE, which are
*  specific to Immediate Alert Service.
*
* Parameters:
*  uint32 event:      event from the BLE component
*  void* eventParams: parameters related to the event
*
*******************************************************************************/
void ble_ias_callback(uint32 event, void *eventParam)
{
    /* Alert Level Characteristic write event */
    if(event == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Read the updated Alert Level value from the GATT database */
        Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL,
            sizeof(alert_level), &alert_level);
        /* Update CYBSP_USER_LED2 to indicate current alert level */
        switch(alert_level)
        {
            case CY_BLE_NO_ALERT:
            {
                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED2, CYBSP_LED_STATE_OFF);
                break;
            }
            case CY_BLE_MILD_ALERT:
            {
                cyhal_gpio_toggle((cyhal_gpio_t)CYBSP_USER_LED2);
                break;
            }
            case CY_BLE_HIGH_ALERT:
            {
                cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED2, CYBSP_LED_STATE_ON);
                break;
            }
            default:
            {
                break;
            }
        }
    }

    /* Remove warning for unused parameter */
    (void)eventParam;
}


/*******************************************************************************
* Function Name: mcwdt_interrupt_handler
********************************************************************************
* Summary:
*  MCWDT interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_lptimer_irq_event_t event (unused)
*
*******************************************************************************/
//void mcwdt_interrupt_handler(void)
//void mcwdt_interrupt_handler(void *handler_arg, cyhal_lptimer_event_t event)
void mcwdt_interrupt_handler(void)
{
    /* Set the interrupt flag */
    mcwdt_intr_flag = true;

    /* Reload the timer to get periodic interrupt */
//    cyhal_lptimer_reload(&mcwdt);

    if(curr_state == MCU_STATE_SHUT_DOWN_BLUETOOTH)
		curr_state = MCU_STATE_CONNECTING;
}


/*******************************************************************************
* Function Name: mcwdt_init
********************************************************************************
* Summary:
*  Initialize MCWDT for generating interrupt.
*
*******************************************************************************/
static void mcwdt_init(void)
{
    cyhal_lptimer_init(&mcwdt);
    cyhal_lptimer_set_time(&mcwdt, MCWDT_MATCH_VALUE);
    cyhal_lptimer_reload(&mcwdt);
    cyhal_lptimer_register_callback(&mcwdt, mcwdt_interrupt_handler, NULL);
    cyhal_lptimer_enable_event(&mcwdt, CYHAL_LPTIMER_COMPARE_MATCH,
                               MCWDT_INTR_PRIORITY, true);
}


/*******************************************************************************
* Function Name: enter_low_power_mode
********************************************************************************
* Summary:
*  Configures the device to enter low power mode.
*
*  The function configures the device to enter deep sleep - whenever the
*  BLE is idle and the UART transmission/reception is not happening.
*
*  In case if BLE is  turned off, the function configures the device to
*  enter hibernate mode.
*
*******************************************************************************/
void enter_low_power_mode(void)
{
    /* Enter hibernate mode if BLE is turned off  */
//    if(CY_BLE_STATE_STOPPED == Cy_BLE_GetState())
//    {
        printf("[INFO] : Entering deep sleep mode\r\n");
//
        /* Turn of user LEDs */
        cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_OFF);
        cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_OFF);

//        /* Wait until UART transfer complete  */
//        while(0UL == Cy_SCB_UART_IsTxComplete(cy_retarget_io_uart_obj.base));

//        if(Cy_SysPm_Hibernate() != CY_SYSPM_SUCCESS) {
//        	printf("[ERROR] Couldn't enter hibernate");
//        }
//    }
//    else
//    {
        Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
//    }
}


/* [] END OF FILE */
