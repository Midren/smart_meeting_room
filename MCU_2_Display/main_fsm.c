#include "main_fsm.h"

#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"

#include "eink_task.h"
#include "cfg.h"


typedef struct {
	char* name;
	int name_len;
	uint8_t *serviceUUID;
	uint8_t servUUID_len;
} advInfo_t;

advInfo_t currentAdvInfo;

static void findAdvInfo(uint8_t *adv, uint8_t len) {
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

void readMsg(uint16_t characteristic_char_index) {
	cy_stc_ble_gattc_read_req_t myVal = {
		.attrHandle = cy_ble_customCServ[CY_BLE_CUSTOMC_BOOKING_INFO_SERVICE_INDEX].customServChar[characteristic_char_index].customServCharHandle[0],
		.connHandle = cy_ble_connHandle[0]
	};

	if(Cy_BLE_GATTC_ReadCharacteristicValue(&myVal) != CY_BLE_SUCCESS) {
		printf("BLE GATTC read error \r\n");
	}
}

void main_fsm(void* pvParameters) {
	for(;;) {
		if(mcwdt_intr_flag) {
			printf("[INFO] IRQ happened \r\n");
			printf("[INFO] MCU_STATE: %d\r\n", curr_state);
			mcwdt_intr_flag = false;
		}
		switch(curr_state) {
			case MCU_STATE_DEEP_SLEEP: {
				Cy_BLE_Disable();
				curr_state = MCU_STATE_SHUT_DOWN_BLUETOOTH;
				break;
			}
			case MCU_STATE_STARTING: {
				Cy_BLE_Enable();
				curr_state = MCU_STATE_CONNECTING;
				break;
			}
			case MCU_STATE_CONNECTING: {
				Cy_BLE_ProcessEvents();
				if(Cy_BLE_GetConnectionState(app_conn_handle) == CY_BLE_CONN_STATE_CLIENT_DISCOVERED) {
					curr_state = MCU_STATE_UPDATING_INFO;
					curr_upd_state = UPDATING_INFO_START_TIME;
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_ON);
				} else {
					cyhal_gpio_toggle((cyhal_gpio_t)CYBSP_USER_LED1);
				}
				break;
			}
			case MCU_STATE_UPDATING_INFO: {
				switch(curr_upd_state) {
					case UPDATING_INFO_START_TIME: {
						readMsg(CY_BLE_CUSTOMC_BOOKING_INFO_STARTTIME_CHAR_INDEX);
						curr_state = MCU_STATE_UPDATING_INFO_PROCESSING;
						break;
					}
					case UPDATING_INFO_END_TIME: {
						readMsg(CY_BLE_CUSTOMC_BOOKING_INFO_ENDTIME_CHAR_INDEX);
						curr_state = MCU_STATE_UPDATING_INFO_PROCESSING;
						break;
					}
					case UPDATING_INFO_OWNER_NAME: {
						readMsg(CY_BLE_CUSTOMC_BOOKING_INFO_OWNER_CHAR_INDEX);
						curr_state = MCU_STATE_UPDATING_INFO_PROCESSING;
						break;
					}
					case UPDATING_INFO_OCCUPATION_STATUS: {
						readMsg(CY_BLE_CUSTOMC_BOOKING_INFO_OCCUPATIONSTATUS_CHAR_INDEX);
						curr_state = MCU_STATE_UPDATING_INFO_PROCESSING;
						break;
					}
					case UPDATING_INFO_FINISHED: {
						printf("[DEBUG] : Start time: %lu\r\n", (uint32_t) booking_info.start_time);
						printf("[DEBUG] : End   time: %lu\r\n", (uint32_t) booking_info.end_time);
						printf("[DEBUG] : Owner name: ");
						for(int i = 0; i < booking_info.owner_name_len; i++) {
							printf("%c", booking_info.owner_name[i]);
						}
						printf("\r\n");
						printf("[DEBUG] : Occupation status: %d\r\n", booking_info.occupation_status);
						curr_state = MCU_STATE_UPDATING_DISPLAY;
						break;
					}
				}
				break;
			}
			case MCU_STATE_SHUT_DOWN_BLUETOOTH:
			case MCU_STATE_UPDATING_INFO_PROCESSING: {
				Cy_BLE_ProcessEvents();
				break;
			}
			case MCU_STATE_UPDATING_DISPLAY: {
				//TODO: add forming display
				xSemaphoreGive(bleSemaphore);
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
//		taskYIELD();
	}
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
void stack_event_handler(uint32_t event, void* eventParam) {
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

        case CY_BLE_EVT_GAPC_SCAN_START_STOP:
        {
        	printf("[INFO] : GAPC Start/Stop scanning \r\n");
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
        	cy_stc_ble_gapc_adv_report_param_t *scanProgressParam = (cy_stc_ble_gapc_adv_report_param_t *) eventParam;
        	printf("[INFO] Device: BD Address = ");
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
					printf("[INFO] : Found LineData Service \r\n");
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

			static BookingInfo info = {
				.owner_name = {},
				.owner_name_len = 0,
				.start_time = 0lu,
				.end_time = 0lu,
				.occupation_status = 1
			};

        	printf("[INFO] : GATTC read response\r\n");
        	cy_stc_ble_gattc_read_rsp_param_t * readRspParam = (cy_stc_ble_gattc_read_rsp_param_t*)eventParam;
        	switch(curr_upd_state) {
        	case UPDATING_INFO_START_TIME:
				memcpy((uint8_t*)&info.start_time, readRspParam->value.val, readRspParam->value.len);
        		curr_upd_state = UPDATING_INFO_END_TIME;
        		curr_state = MCU_STATE_UPDATING_INFO;
        		break;
        	case UPDATING_INFO_END_TIME:
				memcpy((uint8_t*)&info.end_time, readRspParam->value.val, readRspParam->value.len);
        		curr_upd_state = UPDATING_INFO_OWNER_NAME;
        		curr_state = MCU_STATE_UPDATING_INFO;
        		break;
        	case UPDATING_INFO_OWNER_NAME:
				memcpy((uint8_t*)&info.owner_name, readRspParam->value.val, readRspParam->value.len);
				info.owner_name_len = readRspParam->value.len;
				info.owner_name[info.owner_name_len] = '\0';
        		curr_upd_state = UPDATING_INFO_OCCUPATION_STATUS;
        		curr_state = MCU_STATE_UPDATING_INFO;
        		break;
        	case UPDATING_INFO_OCCUPATION_STATUS:
				memcpy((uint8_t*)&info.occupation_status, readRspParam->value.val, readRspParam->value.len);
        		curr_upd_state = UPDATING_INFO_FINISHED;
				curr_state = MCU_STATE_UPDATING_INFO;
				booking_info = info;
				break;
        	case UPDATING_INFO_FINISHED:
        		printf("Redundant read req was made \r\n");
        		break;
        	}
            break;
        }
        default:
        {
            printf("[INFO] : BLE Event 0x%lX\r\n", (unsigned long) event);
        }
    }

}

void mcwdt_interrupt_handler(void)
{
    /* Set the interrupt flag */
    mcwdt_intr_flag = true;

	/* Clear WDT Interrupt */
#if(LOW_POWER_MODE == LOW_POWER_DEEP_SLEEP)
    Cy_MCWDT_ClearInterrupt(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1);
#endif
#if(LOW_POWER_MODE == LOW_POWER_HIBERNATE)
    Cy_WDT_ClearInterrupt();
#endif

    if(curr_state == MCU_STATE_SHUT_DOWN_BLUETOOTH) {
		curr_state = MCU_STATE_STARTING;
    }
}


void enter_low_power_mode(void) {
    /* Enter hibernate mode if BLE is turned off  */
	printf("[INFO] : Entering deep sleep mode\r\n");

	cyhal_gpio_write((cyhal_gpio_t)CYBSP_USER_LED1, CYBSP_LED_STATE_OFF);
	cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_OFF);


#if(LOW_POWER_MODE == LOW_POWER_DEEP_SLEEP)
	Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
#endif
#if(LOW_POWER_MODE == LOW_POWER_HIBERNATE)
        /* Wait until UART transfer complete  */
        while(0UL == Cy_SCB_UART_IsTxComplete(cy_retarget_io_uart_obj.base));
        Cy_SysPm_Hibernate();
#endif
}


/* [] END OF FILE */
