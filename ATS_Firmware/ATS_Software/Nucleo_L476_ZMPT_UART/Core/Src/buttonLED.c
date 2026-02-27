#include "buttonLED.h"
#include "main.h"
#include "uart_display.h"

/* Keep state private to this module */
static uint8_t s_lastButtonState = GPIO_PIN_SET; /* active-low button; default not pressed */
static uint8_t s_ledState = 0;                   /* 0 = OFF, 1 = ON */

void ButtonLED_Init(void)
{
    /* Capture initial state to avoid false trigger on boot */
    s_lastButtonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    /* Start LED off */
    s_ledState = 0;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    uart_printf("ButtonLED: init complete\r\n");
}

void ButtonLED_Task(void)
{
    uint8_t currentButton = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    /* Detect falling edge: released -> pressed */
    if (currentButton == GPIO_PIN_RESET && s_lastButtonState == GPIO_PIN_SET)
    {
        s_ledState = (uint8_t)!s_ledState;

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                          s_ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);

        uart_printf("Button pressed. LD2 is now %s\r\n",
                    s_ledState ? "ON" : "OFF");
    }

    s_lastButtonState = currentButton;
}