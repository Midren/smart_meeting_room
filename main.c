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

#if(LOW_POWER_MODE == LOW_POWER_HIBERNATE)
    /* Configure switch SW2 as hibernate wake up source */
    Cy_SysPm_SetHibWakeupSource(CY_SYSPM_HIBPIN1_LOW);
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


    /* Initialize the User LEDs */
    result = cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED2, CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    init_peripherial();

	__enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");


	printf("**********************************************************\r\n");
	printf("PSoC 6 MCU emWin E-Ink\r\n");
	printf("**********************************************************\r\n");

    flash_counter_init();


    if(CY_SYSLIB_RESET_SWWDT0 == Cy_SysLib_GetResetReason())
    {
    	printf("[INFO] : Returned from deep sleep using MCWDT \r\n");
    }

    size_t counter = get_flash_counter_value();
    printf("[INFO] Counter value: %d \r\n", counter);

    set_flash_counter_value(23u);

    counter = get_flash_counter_value();
    printf("[INFO] Counter value: %d \r\n", counter);

    set_flash_counter_value(24u);

    counter = get_flash_counter_value();
    printf("[INFO] Counter value: %d \r\n", counter);

    Cy_MCWDT_ClearInterrupt(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1);

	curr_state = MCU_STATE_CONNECTING;

	xTaskCreate(eInkTask,"eInkTask", 2000,  NULL,  2,  &update_scr_task);
	xTaskCreate(main_fsm, "ble_update", 2000, (void*) &update_scr_task, 1, NULL);
	vTaskStartScheduler();

	while(1);// Will never get here
}
