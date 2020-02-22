#include "eink_task.h"

#include "cyhal.h"
#include "cybsp.h"
#include "GUI.h"
#include "pervasive_eink_hardware_driver.h"
#include "cy_eink_library.h"
#include "LCDConf.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main_fsm.h"
#include "cfg.h"

/* Display frame buffer cache */
uint8 imageBufferCache[PV_EINK_IMAGE_SIZE] = {0};

void UpdateDisplay(cy_eink_update_t updateMethod, bool powerCycle)
{
    cy_eink_frame_t* pEmwinBuffer;

    /* Get the pointer to Emwin's display buffer */
    pEmwinBuffer = (cy_eink_frame_t*)LCD_GetDisplayBuffer();

    /* Update the EInk display */
    Cy_EINK_ShowFrame(imageBufferCache, pEmwinBuffer, updateMethod, powerCycle);

    /* Copy the EmWin display buffer to the imageBuffer cache*/
    memcpy(imageBufferCache, pEmwinBuffer, CY_EINK_FRAME_SIZE);
}


void show_booking_info(BookingInfo info) {
    /* Set font size, foreground and background colors */
    GUI_SetColor(GUI_BLACK);
    GUI_SetBkColor(GUI_WHITE);
    GUI_SetTextMode(GUI_TM_NORMAL);
    GUI_SetTextStyle(GUI_TS_NORMAL);

    /* Clear the screen */
    GUI_Clear();

    /* Display page title */
    GUI_SetFont(GUI_FONT_20B_1);
    GUI_SetTextAlign(GUI_TA_HCENTER);

    GUI_DispStringAt("Room booking status", 132, 5);

	static char occupation_duration_line[24] = "Duration: ";

    struct tm *timeinfo = localtime(&info.start_time);
    sprintf(occupation_duration_line + 10,
    		"%02d:%02d - ",
			timeinfo->tm_hour, timeinfo->tm_min);

    timeinfo = localtime(&info.end_time);
    sprintf(occupation_duration_line + 18,
    		"%02d:%02d",
			timeinfo->tm_hour, timeinfo->tm_min);

    occupation_duration_line[35] = '\0';

    GUI_SetFont(GUI_FONT_16B_1);
    GUI_DispStringAt(occupation_duration_line, 5, 53);

    GUI_DispStringAt("Booked by: ", 5, 73);
    GUI_DispStringAt(info.owner_name, 5 + GUI_GetStringDistX("Booked by: "), 73);

    if(info.occupation_status) {
		GUI_DispStringAt("Status: Occupied", 5, 93);
    } else {
		GUI_DispStringAt("Status: Free", 5, 93);
    }

    /* Send the display buffer data to display*/
    UpdateDisplay(CY_EINK_FULL_4STAGE, true);
}


void ClearScreen(void)
{
    GUI_SetColor(GUI_BLACK);
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
    UpdateDisplay(CY_EINK_FULL_4STAGE, true);
}


void e_ink_init(void) {
	bleSemaphore = xSemaphoreCreateCounting(1000, 0);
    /* Configure Switch and LEDs*/
//    cyhal_gpio_init((cyhal_gpio_t)CYBSP_LED_RGB_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
//    cyhal_gpio_init((cyhal_gpio_t)CYBSP_SW2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_init((cyhal_gpio_t)CYBSP_LED_RGB_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize EmWin driver*/
    GUI_Init();

    /* Start the eInk display interface and turn on the display power */
	Cy_EINK_Start(20);
	Pv_EINK_HardwarePowerOn();

}

void e_ink_task(void*arg)
{
    e_ink_init();

	for(;;)
	{
		cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_ON);
		xSemaphoreTake(bleSemaphore, portMAX_DELAY);

		show_booking_info(booking_info);

		cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_OFF);

		curr_state = MCU_STATE_UPDATING_DISPLAY_FINISHED;
	}
}
