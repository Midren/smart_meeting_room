#include <limits.h>

#include "cybsp.h"
#include "cybsp_types.h"
#include "cyhal.h"
#include "cy_pdl.h"

#include "cy_retarget_io.h"
#include "cycfg_ble.h"
#include "FreeRTOS.h"
#include "task.h"

#include "cfg.h"
#include "eink_task.h"
#include "main_fsm.h"
#include "flash_counter.h"


void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}


int main(void)
{
    cy_rslt_t result;

#if(LOW_POWER_MODE == LOW_POWER_HIBERNATE)
    /* Configure switch WDT as hibernate wake up source */
    Cy_SysPm_SetHibWakeupSource(CY_SYSPM_HIBWDT);

    /* Unfreeze IO if device is waking up from hibernate */
    if(Cy_SysPm_GetIoFreezeStatus())
    {
        Cy_SysPm_IoUnfreeze();
    }
#endif

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

    init_peripherial();

    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");


	printf("**********************************************************\r\n");
	printf("PSoC 6 MCU emWin E-Ink\r\n");
	printf("**********************************************************\r\n");

    flash_counter_init();

    printf("Reset reason: %d\r\n", (int) Cy_SysLib_GetResetReason());
    if(CY_SYSLIB_RESET_HIB_WAKEUP == Cy_SysLib_GetResetReason())
    {
    	printf("[INFO] : Returned from hibernate using WDT \r\n");
		increment_flash_counter();
		uint16_t counter = get_flash_counter_value();
		printf("[INFO] : Counter value: %d \r\n", counter);
		if(counter < 5) {
			enter_low_power_mode();
		} else {
			set_flash_counter_value(0u);
		}
    } else {
		printf("[INFO] : Init Flash Counter \r\n");
    	set_flash_counter_value(0u);
    }

#if(LOW_POWER_MODE == LOW_POWER_DEEP_SLEEP)
    Cy_MCWDT_ClearInterrupt(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1);
#endif
#if(LOW_POWER_MODE == LOW_POWER_HIBERNATE)
    Cy_WDT_ClearWatchdog();
#endif
	curr_state = MCU_STATE_CONNECTING;
	curr_upd_state = UPDATING_INFO_FINISHED;

	xTaskCreate(e_ink_task,"e_ink_task", 2000,  NULL,  2,  &update_scr_task);
	xTaskCreate(main_fsm, "main_fsm", 2000, (void*) &update_scr_task, 1, NULL);
	vTaskStartScheduler();

	while(1);// Will never get here
}
