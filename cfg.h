#ifndef CFG_H_
#define CFG_H_

#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_ble.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main_fsm.h"
#include "flash_counter.h"

typedef enum {
	MCU_STATE_DEEP_SLEEP,
	MCU_STATE_SHUT_DOWN_BLUETOOTH,
	MCU_STATE_STARTING,
	MCU_STATE_CONNECTING,
	MCU_STATE_UPDATING_INFO,
	MCU_STATE_UPDATING_INFO_PROCESSING,
	MCU_STATE_UPDATING_DISPLAY,
	MCU_STATE_UPDATING_DISPLAY_FINISHED,
	MCU_STATE_ERROR
} mcu_state_t;

typedef enum {
	UPDATING_INFO_START_TIME,
	UPDATING_INFO_END_TIME,
	UPDATING_INFO_OWNER_NAME,
	UPDATING_INFO_FINISHED
} updating_state_t;

mcu_state_t curr_state;
updating_state_t curr_upd_state;

#define BLESS_INTR_PRIORITY		(3u)
#define MCWDT_INTR_PRIORITY     (7u)

#define LOW_POWER_HIBERNATE 1
#define LOW_POWER_DEEP_SLEEP 2

//#define LOW_POWER_MODE LOW_POWER_HIBERNATE
#define LOW_POWER_MODE LOW_POWER_DEEP_SLEEP

bool mcwdt_intr_flag;
bool gpio_intr_flag;

int init_peripherial();

#endif /* CFG_H_ */
