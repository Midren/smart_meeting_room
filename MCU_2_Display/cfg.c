#include "cfg.h"

static void bless_interrupt_handler(void)
{
    Cy_BLE_BlessIsrHandler();
}

void ble_init(void)
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
}

/* WDT Interrupt Number */
#define WDT_IRQ__INTC_NUMBER                srss_interrupt_IRQn
#define WDT_IRQ__INTC_CORTEXM4_PRIORITY     7

void mcwdt_init()
{
#if(LOW_POWER_MODE == LOW_POWER_DEEP_SLEEP)
     /* Step 1 - Unlock WDT */
    Cy_MCWDT_Unlock(CYBSP_MCWDT_HW);

    /* Step 2 - Initial configuration of MCWDT */
	Cy_MCWDT_Init(CYBSP_MCWDT_HW, &CYBSP_MCWDT_config);

    /* Step 3 - Clear match event interrupt, if any */
    Cy_MCWDT_ClearInterrupt(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1);

    /* Step 4 - Enable ILO */
    Cy_SysClk_IloEnable();

    const cy_stc_sysint_t mcwdt_isr_config =
    {
      .intrSrc = (IRQn_Type)CYBSP_MCWDT_IRQ,
      .intrPriority = MCWDT_INTR_PRIORITY
    };

    /* Step 5 - Enable interrupt if periodic interrupt mode selected */
    Cy_SysInt_Init(&mcwdt_isr_config, mcwdt_interrupt_handler);

    NVIC_EnableIRQ(mcwdt_isr_config.intrSrc);

	Cy_MCWDT_SetInterruptMask(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1);
    /* Step 6- Enable WDT */
	Cy_MCWDT_Enable(CYBSP_MCWDT_HW, CY_MCWDT_CTR0 | CY_MCWDT_CTR1, 93u);

    /* Step 7- Lock WDT configuration */
    Cy_MCWDT_Lock(CYBSP_MCWDT_HW);
#endif
#if(LOW_POWER_MODE == LOW_POWER_HIBERNATE)
     /* Step 1- Unlock WDT */
    Cy_WDT_Unlock();

    /* Step 2- Write the ignore bits - operate with full 16 bits */
    Cy_WDT_SetIgnoreBits(0);

	Cy_WDT_SetMatch(0);

    /* Step 4- Clear match event interrupt, if any */
    Cy_WDT_ClearInterrupt();

    /* Step 5- Enable ILO */
    Cy_SysClk_IloEnable();

    /* Step 7- Enable WDT */
    Cy_WDT_Enable();

    /* Step 8- Lock WDT configuration */
    Cy_WDT_Lock();
#endif
}

int init_peripherial() {
	mcwdt_intr_flag = false;
	gpio_intr_flag = false;

	ble_init();
    mcwdt_init();

    /* Initialize the User LEDs */
    cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_init((cyhal_gpio_t)CYBSP_USER_LED2, CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

	return 0;
}
