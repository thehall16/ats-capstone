/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * Demo mode switches.
 *
 * Only enable ONE main demo at a time to keep behavior predictable.
 * Demo_ButtonLed() is safe to run in all modes.
 */
#define DEMO_VOLTAGE_TEST      0   // ZMPT + RMS + UART demo
#define DEMO_RELAY_TEST        0   // Simple relay LED pattern demo
#define DEMO_FAILOVER          1   // Voltage-driven MAIN/GEN failover demo
#define DEMO_GENSIM            0   // RESERVED: future GenSim with status LEDs
#define DEMO_FINAL_COMBINED    0   // Future final demo

// Hard-coded ADC DC offset (midpoint) for the ZMPT output.
const uint16_t ADC_OFFSET = 3080;

// From calibration: ~0.61 Vrms at module OUT when line is 120 Vrms.
// 120 / 0.61 ≈ 196.7  --> 185.5f more accurate after tuning
const float lineScaleFactor = 185.5f;      // converts module Vrms -> line Vrms

// Failover / mains behavior config
// Wide window for deciding "mains is bad"
#define MAINS_MIN_VRMS        108.0f
#define MAINS_MAX_VRMS        132.0f

// Tighter window for deciding "mains is good enough to come back"
#define MAINS_RETURN_MIN_VRMS 117.0f
#define MAINS_RETURN_MAX_VRMS 123.0f

#define MAINS_BAD_TIME_MS      500    // must be bad for 0.5 s before leaving MAIN
#define SWITCH_DELAY_MS       1000    // 1 second between staged switching steps
#define MAINS_RETURN_TIME_MS  2000    // 2 seconds of good mains before switching back

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Button → LD2 state for the button demo
uint8_t  lastButtonState = GPIO_PIN_SET;   // assume not pressed at start
uint8_t  ledState        = 0;              // LED off

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

// Demo function prototypes
void Demo_ButtonLed(void);        // Blue button toggles LD2 + UART print
void Demo_Voltage(void);          // ZMPT voltage measurement + UART print
void Demo_RelayTest(void);        // Relay-only GPIO demo (temp LEDs on RELAY_* pins)
void Demo_Failover(void);         // Voltage-driven MAIN/GEN failover using relays
void Demo_GenSim(void);           // RESERVED: full generator simulator w/ LEDs
void Demo_FinalCombined(void);    // Future final combined project demo

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart_printf(const char *fmt, ...)
{
    // Increased buffer size so large multi-line prints (ASCII box) fit.
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len <= 0)
        return;

    if (len > (int)sizeof(buffer))
        len = sizeof(buffer);

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

// Measure AC RMS (module output) using hard-coded ADC_OFFSET
float get_ac_rms(uint16_t samples)
{
    float vref = 3.3f;    // ADC reference voltage
    float sumSq = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        if (HAL_ADC_Start(&hadc1) == HAL_OK)
        {
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            {
                uint32_t raw = HAL_ADC_GetValue(&hadc1);
                // remove DC offset using the hard-coded constant
                int32_t ac_raw = (int32_t)raw - (int32_t)ADC_OFFSET;

                // AC component in volts (can be positive or negative)
                float v = (ac_raw * vref) / 4095.0f;

                sumSq += v * v;
            }
            HAL_ADC_Stop(&hadc1);
        }
    }

    float meanSq = sumSq / (float)samples;
    return sqrtf(meanSq);   // RMS of AC part at module output, in volts
}

/*
 * Demo: blue button (B1) toggles the on-board LED (LD2)
 * and prints the new state over UART.
 *
 * This can safely run alongside any other demo.
 */
void Demo_ButtonLed(void)
{
    uint8_t currentButton = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    // Button is active LOW on the Nucleo board
    if (currentButton == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
    {
        ledState = !ledState;

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                          ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);

        uart_printf("Button pressed. LED is now %s\r\n",
                    ledState ? "ON" : "OFF");
    }

    lastButtonState = currentButton;
}

/*
 * Demo: Voltage measurement via ZMPT.
 * Prints module Vrms and calculated line Vrms once every 2 seconds.
 */
void Demo_Voltage(void)
{
    static uint32_t lastPrint = 0;
    uint32_t now = HAL_GetTick();

    if (now - lastPrint >= 2000)    // 2000 ms = 2 seconds
    {
        float v_ac_rms  = get_ac_rms(12000);     // module RMS
        float line_vrms = v_ac_rms * lineScaleFactor;

        // ignore tiny noise when no AC is present
        if (line_vrms < 5.0f)
            line_vrms = 0.0f;

        int displayVolts = (int)(line_vrms + 0.5f);

        uart_printf("Module Vrms: %.3f V | Line: %d V\r\n",
                    v_ac_rms, displayVolts);

        lastPrint = now;
    }
}

/*
 * Relay helper functions.
 * Use ONLY the RELAY_* pins here.
 * For bench testing, you connect LEDs + resistors from each RELAY_* pin → GND.
 */

static void Relay_AllOff(void)
{
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
}

static void Relay_MainOnly(void)
{
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
}

static void Relay_GenOnly(void)
{
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
}

static void Relay_GenWithTransfer(void)
{
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_SET);
}

/*
 * Optional: status LED helpers (reserved for future GenSim / final demo).
 * These use LED_S1/S2/S3 but are NOT used in the relay-only or failover demos.
 */

static void Status_AllOff(void)
{
    HAL_GPIO_WritePin(LED_S1_GPIO_Port, LED_S1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_S2_GPIO_Port, LED_S2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_S3_GPIO_Port, LED_S3_Pin, GPIO_PIN_RESET);
}

static void Status_Set(uint8_t s1, uint8_t s2, uint8_t s3)
{
    HAL_GPIO_WritePin(LED_S1_GPIO_Port, LED_S1_Pin, s1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_S2_GPIO_Port, LED_S2_Pin, s2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_S3_GPIO_Port, LED_S3_Pin, s3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * Demo: Relay-only pattern.
 */
void Demo_RelayTest(void)
{
    static uint32_t lastStep = 0;
    static int step = 0;
    uint32_t now = HAL_GetTick();

    if (now - lastStep >= 1000)   // every 1 second
    {
        lastStep = now;
        step = (step + 1) % 4;

        switch (step)
        {
            case 0:
                uart_printf("Relay demo: all OFF\r\n");
                Relay_AllOff();
                break;

            case 1:
                uart_printf("Relay demo: MAIN ON\r\n");
                Relay_MainOnly();
                break;

            case 2:
                uart_printf("Relay demo: GEN ON\r\n");
                Relay_GenOnly();
                break;

            case 3:
                uart_printf("Relay demo: TRANSFER ON (GEN+TRANSFER)\r\n");
                Relay_GenWithTransfer();
                break;
        }
    }
}

/*
 * FAILOVER demo state machine + pretty UART status.
 * This is the voltage-driven MAIN/GEN switcher.
 */

typedef enum {
    FAILOVER_ON_MAIN = 0,               // MAIN on, others off
    FAILOVER_SWITCH_TO_GEN_ALL_OFF,     // all off for 1s
    FAILOVER_SWITCH_TO_GEN_GEN_ONLY,    // GEN on alone for 1s
    FAILOVER_ON_GEN,                    // GEN+TRANSFER on
    FAILOVER_SWITCH_TO_MAIN_GEN_OFF,    // GEN off, TRANSFER still on, delay
    FAILOVER_SWITCH_TO_MAIN_TRANSFER_OFF, // TRANSFER off (all off), delay
    FAILOVER_SWITCH_TO_MAIN_MAIN_ON     // MAIN on, then back to ON_MAIN
} FailoverState;

static const char* Failover_StateName(FailoverState s)
{
    switch (s)
    {
        case FAILOVER_ON_MAIN:                 return "ON_MAIN";
        case FAILOVER_SWITCH_TO_GEN_ALL_OFF:   return "SW_TO_GEN_ALL_OFF";
        case FAILOVER_SWITCH_TO_GEN_GEN_ONLY:  return "SW_TO_GEN_GEN_ONLY";
        case FAILOVER_ON_GEN:                  return "ON_GEN";
        case FAILOVER_SWITCH_TO_MAIN_GEN_OFF:  return "SW_TO_MAIN_GEN_OFF";
        case FAILOVER_SWITCH_TO_MAIN_TRANSFER_OFF: return "SW_TO_MAIN_XFER_OFF";
        case FAILOVER_SWITCH_TO_MAIN_MAIN_ON:  return "SW_TO_MAIN_MAIN_ON";
        default:                               return "UNKNOWN";
    }
}

static void Failover_PrintStatus(FailoverState state, float vrms)
{
    uint8_t mainOn     = (HAL_GPIO_ReadPin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin)     == GPIO_PIN_SET);
    uint8_t genOn      = (HAL_GPIO_ReadPin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin)      == GPIO_PIN_SET);
    uint8_t transferOn = (HAL_GPIO_ReadPin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin) == GPIO_PIN_SET);

    const char* source = "NONE";

    if (mainOn && !transferOn)
    {
        source = "MAIN";
    }
    else if (genOn && transferOn)
    {
        source = "GEN";
    }

    uart_printf(
        "\r\n"
        "+-------------------------------------------+\r\n"
        "|        AUTO SWITCH DEMO: STATUS           |\r\n"
        "+-------------------------------------------+\r\n"
        "| Line Voltage: %6.1f V RMS                 |\r\n"
        "| State       : %-27s|\r\n"
        "| MAIN Relay  : %s\r\n"
        "| GEN Relay   : %s\r\n"
        "| TRANSFER    : %s\r\n"
        "| Source      : %s\r\n"
        "+-------------------------------------------+\r\n",
        vrms,
        Failover_StateName(state),
        mainOn     ? "ON " : "OFF",
        genOn      ? "ON " : "OFF",
        transferOn ? "ON " : "OFF",
        source
    );
}

/*
 * Demo_Failover:
 *
 * Combined voltage + relay demo.
 *
 * States:
 *  - ON_MAIN:
 *      MAIN ON, others OFF.
 *      If Vrms is outside [108..132] for 500 ms, start switch to GEN.
 *
 *  - SWITCH_TO_GEN_ALL_OFF:
 *      All relays OFF for 1 second.
 *
 *  - SWITCH_TO_GEN_GEN_ONLY:
 *      GEN ON alone for 1 second (sim: generator spin-up), TRANSFER OFF.
 *
 *  - ON_GEN:
 *      GEN+TRANSFER ON (load on generator).
 *      When Vrms is in [117..123] for 2 seconds, start switch back to MAIN.
 *
 *  - SWITCH_TO_MAIN_GEN_OFF:
 *      GEN OFF, TRANSFER still ON for 1 second (your requested sequence).
 *
 *  - SWITCH_TO_MAIN_TRANSFER_OFF:
 *      TRANSFER OFF (all OFF) for 1 second.
 *
 *  - SWITCH_TO_MAIN_MAIN_ON:
 *      MAIN ON, others OFF, then return to ON_MAIN.
 */

void Demo_Failover(void)
{
    static FailoverState state = FAILOVER_ON_MAIN;
    static uint32_t lastMeasureTick   = 0;
    static uint32_t lastPrintTick     = 0;
    static float    lastLineVrms      = 0.0f;
    static uint8_t  haveValidVoltage  = 0;

    static uint32_t switchStartTick    = 0;  // for staged switching
    static uint32_t mainsGoodStartTick = 0;  // for 2-second good mains timing
    static uint32_t mainsBadStartTick  = 0;  // for 0.5-second bad mains timing

    uint32_t now = HAL_GetTick();

    // --- 1) Voltage measurement (every 500 ms) ---
    if (now - lastMeasureTick >= 500)
    {
        lastMeasureTick = now;

        float v_ac_rms  = get_ac_rms(4000);      // module RMS
        float line_vrms = v_ac_rms * lineScaleFactor;

        if (line_vrms < 5.0f)    // ignore noise, treat as 0
            line_vrms = 0.0f;

        lastLineVrms = line_vrms;

        if (lastLineVrms > 5.0f)
        {
            haveValidVoltage = 1;   // we've seen real mains at least once
        }
    }

    // --- 2) Slow, readable UART printing (every 2000 ms) ---
    if (now - lastPrintTick >= 2000)
    {
        lastPrintTick = now;
        Failover_PrintStatus(state, lastLineVrms);
    }

    // Until we've seen some non-zero mains, don't bail off MAIN
    uint8_t mainsInWideRange = 1;
    uint8_t mainsInReturnRange = 0;

    if (haveValidVoltage)
    {
        mainsInWideRange =
            (lastLineVrms >= MAINS_MIN_VRMS) &&
            (lastLineVrms <= MAINS_MAX_VRMS);

        mainsInReturnRange =
            (lastLineVrms >= MAINS_RETURN_MIN_VRMS) &&
            (lastLineVrms <= MAINS_RETURN_MAX_VRMS);
    }

    // --- 3) State machine for relays ---
    switch (state)
    {
        case FAILOVER_ON_MAIN:
            // MAIN ON, others OFF
            Relay_MainOnly();

            mainsGoodStartTick = 0;

            if (haveValidVoltage && !mainsInWideRange)
            {
                if (mainsBadStartTick == 0)
                {
                    mainsBadStartTick = now;  // start timing bad mains
                }
                else if (now - mainsBadStartTick >= MAINS_BAD_TIME_MS)
                {
                    uart_printf("Failover: MAIN voltage out of range (%.1f V for %lu ms). Switching to GEN.\r\n",
                                lastLineVrms, (unsigned long)(now - mainsBadStartTick));

                    Relay_AllOff();
                    switchStartTick   = now;
                    mainsBadStartTick = 0;
                    state = FAILOVER_SWITCH_TO_GEN_ALL_OFF;
                }
            }
            else
            {
                mainsBadStartTick = 0;
            }
            break;

        case FAILOVER_SWITCH_TO_GEN_ALL_OFF:
            // All relays OFF for SWITCH_DELAY_MS
            Relay_AllOff();
            if (now - switchStartTick >= SWITCH_DELAY_MS)
            {
                // Turn GEN ON only
                Relay_GenOnly();
                switchStartTick = now;
                uart_printf("Failover: GEN ON (TRANSFER still OFF).\r\n");
                state = FAILOVER_SWITCH_TO_GEN_GEN_ONLY;
            }
            break;

        case FAILOVER_SWITCH_TO_GEN_GEN_ONLY:
            // GEN ON alone for SWITCH_DELAY_MS before we bring in TRANSFER
            Relay_GenOnly();
            if (now - switchStartTick >= SWITCH_DELAY_MS)
            {
                Relay_GenWithTransfer();
                uart_printf("Failover: GEN & TRANSFER ON (load on generator).\r\n");
                state = FAILOVER_ON_GEN;
            }
            break;

        case FAILOVER_ON_GEN:
            // GEN + TRANSFER ON, MAIN OFF
            Relay_GenWithTransfer();

            if (haveValidVoltage && mainsInReturnRange)
            {
                if (mainsGoodStartTick == 0)
                {
                    mainsGoodStartTick = now;  // start timing
                }
                else if (now - mainsGoodStartTick >= MAINS_RETURN_TIME_MS)
                {
                    uart_printf("Failover: MAIN voltage stable (%.1f V). Switching back to MAIN.\r\n",
                                lastLineVrms);

                    switchStartTick    = now;
                    mainsGoodStartTick = 0;
                    state = FAILOVER_SWITCH_TO_MAIN_GEN_OFF;
                }
            }
            else
            {
                mainsGoodStartTick = 0;
            }
            break;

        case FAILOVER_SWITCH_TO_MAIN_GEN_OFF:
            // GEN OFF first, TRANSFER still ON (your requested visual sequence)
            HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);

            if (now - switchStartTick >= SWITCH_DELAY_MS)
            {
                switchStartTick = now;
                state = FAILOVER_SWITCH_TO_MAIN_TRANSFER_OFF;
            }
            break;

        case FAILOVER_SWITCH_TO_MAIN_TRANSFER_OFF:
            // Now drop TRANSFER as well (all OFF) for a delay
            HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_RESET);

            if (now - switchStartTick >= SWITCH_DELAY_MS)
            {
                switchStartTick = now;
                state = FAILOVER_SWITCH_TO_MAIN_MAIN_ON;
            }
            break;

        case FAILOVER_SWITCH_TO_MAIN_MAIN_ON:
            // Bring MAIN back on, others off
            Relay_MainOnly();
            uart_printf("Failover: MAIN ON, load returned to mains.\r\n");
            state = FAILOVER_ON_MAIN;
            break;

        default:
            Relay_AllOff();
            state = FAILOVER_ON_MAIN;
            break;
    }
}

/*
 * RESERVED: Generator simulator state machine demo.
 */
void Demo_GenSim(void)
{
    // TODO: implement full GenSim state machine here using LED_S1/S2/S3
}

/*
 * Placeholder: FINAL / COMBINED PROJECT DEMO.
 */
void Demo_FinalCombined(void)
{
    // TODO: call GenSim + monitoring + any extra UI behavior here
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  uart_printf("System started.\r\n");
  uart_printf("Demo config: V=%d, R=%d, F=%d, G=%d, C=%d\r\n",
              DEMO_VOLTAGE_TEST,
              DEMO_RELAY_TEST,
              DEMO_FAILOVER,
              DEMO_GENSIM,
              DEMO_FINAL_COMBINED);

  lastButtonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

  // Power LED
  HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_SET);

  Status_AllOff();
  Relay_AllOff();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Demo_ButtonLed();

#if DEMO_VOLTAGE_TEST
    Demo_Voltage();
#endif

#if DEMO_RELAY_TEST
    Demo_RelayTest();
#endif

#if DEMO_FAILOVER
    Demo_Failover();
#endif

#if DEMO_GENSIM
    Demo_GenSim();
#endif

#if DEMO_FINAL_COMBINED
    Demo_FinalCombined();
#endif

  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_5;         // PA0 / IN5 for ZMPT
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RELAY_MAIN_Pin|RELAY_GEN_Pin|RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LED_PWR_Pin|LED_S1_Pin|LED_S2_Pin|LED_S3_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // polling mode, no interrupt
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin|RELAY_MAIN_Pin|RELAY_GEN_Pin|RELAY_TRANSFER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_PWR_Pin|LED_S1_Pin|LED_S2_Pin|LED_S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
