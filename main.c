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

void MCWDT_init()
{
     /* Step 1 - Unlock WDT */
    Cy_MCWDT_Unlock(CYBSP_MCWDT_HW);

    /* Step 2 - Initial configuration of MCWDT */
	Cy_MCWDT_Init(CYBSP_MCWDT_HW, &CYBSP_MCWDT_config);

    /* Step 3 - Clear match event interrupt, if any */
    Cy_MCWDT_ClearInterrupt(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1);

    /* Step 4 - Enable ILO */
    Cy_SysClk_IloEnable();

#if(LOW_POWER_MODE == LOW_POWER_DEEP_SLEEP)
    const cy_stc_sysint_t mcwdt_isr_config =
    {
      .intrSrc = (IRQn_Type)CYBSP_MCWDT_IRQ,
      .intrPriority = MCWDT_INTR_PRIORITY
    };

    /* Step 5 - Enable interrupt if periodic interrupt mode selected */
    Cy_SysInt_Init(&mcwdt_isr_config, mcwdt_interrupt_handler);
    NVIC_EnableIRQ(mcwdt_isr_config.intrSrc);
	Cy_MCWDT_SetInterruptMask(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1);
#endif
    /* Step 6- Enable WDT */
	Cy_MCWDT_Enable(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1, 93u);

    /* Step 7- Lock WDT configuration */
    Cy_MCWDT_Lock(CYBSP_MCWDT_HW);
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

    ble_task_init();

    MCWDT_init();

	__enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");


	printf("**********************************************************\r\n");
	printf("PSoC 6 MCU emWin E-Ink\r\n");
	printf("**********************************************************\r\n");
//    printf("[INFO] : Watchdogs enable status: %d, %d\r\n", (int) Cy_MCWDT_GetEnabledStatus(CYBSP_MCWDT_HW, CY_MCWDT_COUNTER0), (int) Cy_MCWDT_GetEnabledStatus(CYBSP_MCWDT_HW, CY_MCWDT_COUNTER1));
//    printf("[INFO] : Watchdogs mode status: %d, %d\r\n", (int) Cy_MCWDT_GetMode(CYBSP_MCWDT_HW, CY_MCWDT_COUNTER0), (int) Cy_MCWDT_GetMode(CYBSP_MCWDT_HW, CY_MCWDT_COUNTER1));
//    printf("[INFO] : Mask status of interrupts: %d\r\n", (int) Cy_MCWDT_GetInterruptMask(CYBSP_MCWDT_HW));
    if(CY_SYSLIB_RESET_SWWDT0 == Cy_SysLib_GetResetReason())
    {
    	printf("[INFO] : Oh my God! We have returned from nowhere using MCWDT \r\n");
    }

    Cy_MCWDT_ClearInterrupt(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1);

	curr_state = MCU_STATE_CONNECTING;

	xTaskCreate(eInkTask,"eInkTask", 2000,  NULL,  2,  &update_scr_task);
	xTaskCreate(ble_task_process, "ble_update", 2000, (void*) &update_scr_task, 1, NULL);
	vTaskStartScheduler();

	while(1);// Will never get here
}
