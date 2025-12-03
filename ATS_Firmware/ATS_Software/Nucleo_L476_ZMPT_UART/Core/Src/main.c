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
#define DEMO_RELAY_TEST        0   // Relay + GPIO-only demo (temp LEDs on RELAY_* pins)
#define DEMO_FAILOVER          0   // Voltage-based automatic MAIN/GEN switching (relays)
#define DEMO_GENSIM            1   // Generator simulator LED-only startup/shutdown demo
#define DEMO_FINAL_COMBINED    0   // Future: full project demo

// Hard-coded ADC DC offset (midpoint) for the ZMPT output.
// Based on initial no-AC measurement (approx. 3080).
const uint16_t ADC_OFFSET = 3080;

// From calibration: ~0.61 Vrms at module OUT when line is 120 Vrms.
// 120 / 0.61 ≈ 196.7  --> 185.5f more accurate after tuning
const float lineScaleFactor = 185.5f;      // converts module Vrms -> line Vrms

/* GenSim timing constants */
#define GENSIM_STARTUP_S1_TIME_MS        1000   // 1 s S1 solid at startup
#define GENSIM_STARTUP_CRANK_TIME_MS     3000   // 3 s S2 blinking at startup
#define GENSIM_SHUT_S1_SOLID_TIME_MS     1000   // 1 s S1 solid at shutdown
#define GENSIM_SHUT_S2_BLINK_TIME_MS     3000   // 3 s S2 blinking at shutdown
#define GENSIM_DEMO_DELAY_MS             5000   // 5 s delay before startup / shutdown
#define GENSIM_BLINK_PERIOD_MS            250   // 250 ms blink period

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
void Demo_Failover(void);         // Voltage-based failover demo (relays + UART)
void Demo_GenSim(void);           // Generator simulator LED-only demo
void Demo_FinalCombined(void);    // Future: final combined project demo

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart_printf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len > 0)
    {
        if (len > (int)sizeof(buffer))
            len = sizeof(buffer);

        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
    }
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
 * For the relay demo you hang temporary LEDs + resistors from these pins to GND.
 */

static void Relay_AllOff(void)
{
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
}

static void Relay_MainOn(void)
{
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
}

static void Relay_GenOn(void)
{
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
}

static void Relay_TransferOn(void)
{
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin,      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_SET);
}

/*
 * Optional: status LED helpers (reserved for GenSim / final demo).
 * These use LED_S1/S2/S3 and are NOT used in the simple relay demo.
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
 * Cycles:
 *   0: all OFF
 *   1: MAIN ON
 *   2: GEN ON
 *   3: TRANSFER ON
 *
 * You connect test LEDs + resistors from each RELAY_* pin → GND.
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
                Relay_MainOn();
                break;

            case 2:
                uart_printf("Relay demo: GEN ON\r\n");
                Relay_GenOn();
                break;

            case 3:
                uart_printf("Relay demo: TRANSFER ON\r\n");
                Relay_TransferOn();
                break;
        }
    }
}

/*
 * Simple failover demo:
 * - Reads line voltage via ZMPT.
 * - Switches from MAIN to GEN when line is outside [108, 132] V RMS
 *   for more than 500 ms.
 * - Switches back to MAIN when line is inside [117, 123] V RMS
 *   for more than 2000 ms.
 * - Drives RELAY_MAIN / RELAY_GEN / RELAY_TRANSFER accordingly.
 */
void Demo_Failover(void)
{
    typedef enum {
        SRC_MAIN = 0,
        SRC_GEN  = 1
    } SourceState;

    static SourceState src = SRC_MAIN;
    static uint32_t badStartTick  = 0;
    static uint32_t goodStartTick = 0;
    static uint32_t lastPrint     = 0;

    uint32_t now = HAL_GetTick();

    // Measure voltage every call (could be throttled if needed)
    float v_ac_rms  = get_ac_rms(4000);
    float line_vrms = v_ac_rms * lineScaleFactor;
    if (line_vrms < 5.0f)
        line_vrms = 0.0f;

    int mainOn     = 0;
    int genOn      = 0;
    int transferOn = 0;

    // MAIN vs GEN logic
    if (src == SRC_MAIN)
    {
        // MAIN considered "bad" if out of [108,132]
        if (line_vrms < 108.0f || line_vrms > 132.0f)
        {
            if (badStartTick == 0)
                badStartTick = now;
            if (now - badStartTick >= 500) // 0.5 s bad
            {
                uart_printf("Failover: MAIN voltage out of range (%.1f V). Switching to GEN.\r\n",
                            line_vrms);
                // GEN + TRANSFER ON
                src = SRC_GEN;
                badStartTick  = 0;
                goodStartTick = 0;
            }
        }
        else
        {
            badStartTick = 0;
        }
    }
    else // SRC_GEN
    {
        // MAIN considered "good" if within [117,123] for at least 2 seconds
        if (line_vrms >= 117.0f && line_vrms <= 123.0f)
        {
            if (goodStartTick == 0)
                goodStartTick = now;
            if (now - goodStartTick >= 2000) // 2 s good
            {
                uart_printf("Failover: MAIN voltage stable (%.1f V). Switching back to MAIN.\r\n",
                            line_vrms);
                src = SRC_MAIN;
                badStartTick  = 0;
                goodStartTick = 0;
            }
        }
        else
        {
            goodStartTick = 0;
        }
    }

    // Drive relays based on chosen source
    if (src == SRC_MAIN)
    {
        mainOn     = 1;
        genOn      = 0;
        transferOn = 0;
        Relay_MainOn();
    }
    else
    {
        mainOn     = 0;
        genOn      = 1;
        transferOn = 1;
        Relay_GenOn();
        Relay_TransferOn();
    }

    // Pretty status print every 1s
    if (now - lastPrint >= 1000)
    {
        lastPrint = now;

        uart_printf(
            "+-------------------------------------------+\r\n"
            "|        AUTO SWITCH DEMO: STATUS           |\r\n"
            "+-------------------------------------------+\r\n"
            "| Line Voltage: %6.1f V RMS                 |\r\n"
            "| Source: %s                                |\r\n"
            "| MAIN Relay:     %s                        |\r\n"
            "| GEN Relay:      %s                        |\r\n"
            "| TRANSFER Relay: %s                        |\r\n"
            "+-------------------------------------------+\r\n",
            line_vrms,
            (src == SRC_MAIN) ? "MAIN" : "GEN ",
            mainOn     ? "ON " : "OFF",
            genOn      ? "ON " : "OFF",
            transferOn ? "ON " : "OFF"
        );
    }
}

/*
 * GenSim: STARTUP sequence
 *
 * Returns:
 *   0 -> still running sequence
 *   1 -> sequence finished (S3 ON, generator "running")
 */
static uint8_t GenSim_RunStartup(void)
{
    typedef enum {
        GS_START_IDLE = 0,
        GS_START_S1_ACTIVE,
        GS_START_CRANK,
        GS_START_DONE
    } GenSimStartState;

    static GenSimStartState s = GS_START_IDLE;
    static uint32_t stateStartTick = 0;
    static uint32_t lastBlinkTick  = 0;
    static uint8_t  s2On           = 0;

    uint32_t now = HAL_GetTick();

    switch (s)
    {
        case GS_START_IDLE:
            // First entry: start the sequence
            Status_AllOff();
            Status_Set(1, 0, 0);   // S1 ON
            stateStartTick = now;
            uart_printf("GenSim: start sequence requested\r\n");
            s = GS_START_S1_ACTIVE;
            return 0;

        case GS_START_S1_ACTIVE:
            // S1 ON for 1 s
            Status_Set(1, 0, 0);
            if (now - stateStartTick >= GENSIM_STARTUP_S1_TIME_MS)
            {
                // Move to crank (S2 blinking)
                s2On = 0;
                lastBlinkTick = now;
                stateStartTick = now;
                uart_printf("GenSim: crank phase (S2 blink)\r\n");
                s = GS_START_CRANK;
            }
            return 0;

        case GS_START_CRANK:
            // S1 ON, S2 blinking, S3 OFF
            if (now - lastBlinkTick >= GENSIM_BLINK_PERIOD_MS)
            {
                s2On = !s2On;
                lastBlinkTick = now;
            }
            Status_Set(1, s2On, 0);

            // After crank time, generator is running
            if (now - stateStartTick >= GENSIM_STARTUP_CRANK_TIME_MS)
            {
                // S3 ON, others OFF
                Status_Set(0, 0, 1);
                uart_printf("GenSim: generator running (S3 ON)\r\n");
                s = GS_START_DONE;
            }
            return 0;

        case GS_START_DONE:
            // Final state: S3 ON only
            Status_Set(0, 0, 1);
            // Reset internal state for next time
            s = GS_START_IDLE;
            return 1;

        default:
            s = GS_START_IDLE;
            return 1;
    }
}

/*
 * GenSim: SHUTDOWN sequence
 *
 * Returns:
 *   0 -> still running sequence
 *   1 -> sequence finished (all S1/S2/S3 OFF)
 *
 * Behavior (your requested version):
 *  - S1 solid ON for 1 s (shutdown requested), S3 still ON.
 *  - S2 blinking for 3 s as cooldown indicator, S1 stays ON, S3 stays ON.
 *  - Then all OFF (generator stopped).
 */
static uint8_t GenSim_RunShutdown(void)
{
    typedef enum {
        GS_SHUT_IDLE = 0,
        GS_SHUT_S1_SOLID,
        GS_SHUT_S2_BLINK,
        GS_SHUT_DONE
    } GenSimShutState;

    static GenSimShutState s = GS_SHUT_IDLE;
    static uint32_t stateStartTick = 0;
    static uint32_t lastBlinkTick  = 0;
    static uint8_t  s2On           = 0;

    uint32_t now = HAL_GetTick();

    switch (s)
    {
        case GS_SHUT_IDLE:
            // Assume generator is running (S3 ON).
            // Begin shutdown: S1 solid ON, S2 off, S3 stays ON.
            Status_Set(1, 0, 1);
            uart_printf("GenSim: shutdown requested\r\n");
            stateStartTick = now;
            s = GS_SHUT_S1_SOLID;
            return 0;

        case GS_SHUT_S1_SOLID:
            // S1 solid for 1 s, S3 still ON
            Status_Set(1, 0, 1);
            if (now - stateStartTick >= GENSIM_SHUT_S1_SOLID_TIME_MS)
            {
                // Start cooldown: S2 blinking, S1 stays ON, S3 stays ON
                s2On = 0;
                lastBlinkTick = now;
                stateStartTick = now;
                uart_printf("GenSim: shutdown cooldown (S2 blink)\r\n");
                s = GS_SHUT_S2_BLINK;
            }
            return 0;

        case GS_SHUT_S2_BLINK:
            // S2 blinking, S1 stays ON, S3 stays ON
            if (now - lastBlinkTick >= GENSIM_BLINK_PERIOD_MS)
            {
                s2On = !s2On;
                lastBlinkTick = now;
            }
            Status_Set(1, s2On, 1);

            if (now - stateStartTick >= GENSIM_SHUT_S2_BLINK_TIME_MS)
            {
                // Cooldown complete — turn off everything.
                Status_AllOff();
                uart_printf("GenSim: generator stopped\r\n");
                s = GS_SHUT_DONE;
            }
            return 0;

        case GS_SHUT_DONE:
            Status_AllOff();
            // Reset for next use
            s = GS_SHUT_IDLE;
            return 1;

        default:
            s = GS_SHUT_IDLE;
            return 1;
    }
}

/*
 * Demo_GenSim:
 * - Waits 5 s
 * - Runs STARTUP sequence (S1 → S2 blink → S3)
 * - Holds RUNNING (S3 ON) for 5 s
 * - Runs SHUTDOWN sequence (S1 solid → S2 blink → all off)
 * - Loops forever
 */
void Demo_GenSim(void)
{
    typedef enum {
        GENSIM_DEMO_WAIT_STARTUP = 0,
        GENSIM_DEMO_DO_STARTUP,
        GENSIM_DEMO_RUNNING_HOLD,
        GENSIM_DEMO_DO_SHUTDOWN
    } GenSimDemoState;

    static GenSimDemoState demoState = GENSIM_DEMO_WAIT_STARTUP;
    static uint32_t phaseStartTick   = 0;

    uint32_t now = HAL_GetTick();

    switch (demoState)
    {
        case GENSIM_DEMO_WAIT_STARTUP:
            // Idle, all status LEDs off
            Status_AllOff();
            if (phaseStartTick == 0)
            {
                phaseStartTick = now;  // start delay timer
            }

            if (now - phaseStartTick >= GENSIM_DEMO_DELAY_MS)
            {
                uart_printf("GenSim demo: starting STARTUP sequence\r\n");
                phaseStartTick = 0;  // reset for next use
                demoState = GENSIM_DEMO_DO_STARTUP;
            }
            break;

        case GENSIM_DEMO_DO_STARTUP:
            if (GenSim_RunStartup())
            {
                // Startup finished: generator is running (S3 ON)
                uart_printf("GenSim demo: STARTUP complete, generator running\r\n");
                phaseStartTick = now;
                demoState = GENSIM_DEMO_RUNNING_HOLD;
            }
            break;

        case GENSIM_DEMO_RUNNING_HOLD:
            // Keep S3 ON to represent running
            Status_Set(0, 0, 1);

            if (now - phaseStartTick >= GENSIM_DEMO_DELAY_MS)
            {
                uart_printf("GenSim demo: starting SHUTDOWN sequence\r\n");
                phaseStartTick = 0;
                demoState = GENSIM_DEMO_DO_SHUTDOWN;
            }
            break;

        case GENSIM_DEMO_DO_SHUTDOWN:
            if (GenSim_RunShutdown())
            {
                uart_printf("GenSim demo: SHUTDOWN complete, generator stopped\r\n");
                phaseStartTick = now;
                // Loop back to the start so the demo repeats:
                demoState = GENSIM_DEMO_WAIT_STARTUP;
            }
            break;

        default:
            demoState = GENSIM_DEMO_WAIT_STARTUP;
            phaseStartTick = 0;
            break;
    }
}

/*
 * Placeholder: FINAL / COMBINED PROJECT DEMO.
 */
void Demo_FinalCombined(void)
{
    // TODO: call Failover + GenSim + monitoring + any final display behavior here
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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

  // Grab initial button state for edge detection demo
  lastButtonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

  // Turn on power LED so we know the board/PCB has power
  HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_SET);

  // Ensure status LEDs and relay drivers start in a known OFF state
  Status_AllOff();
  Relay_AllOff();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
     * CENTRAL DEMO DISPATCH
     *
     * - Demo_ButtonLed() can safely run in all modes
     *   (just watches the blue button and toggles LD2).
     * - Enable exactly ONE of the DEMO_* main modes at the top.
     */

    // Keep the button → LD2 demo active in all modes
    Demo_ButtonLed();

#if DEMO_VOLTAGE_TEST
    // ZMPT + RMS UART demo
    Demo_Voltage();
#endif

#if DEMO_RELAY_TEST
    // Relay-only pattern using LEDs on RELAY_* pins
    Demo_RelayTest();
#endif

#if DEMO_FAILOVER
    // Automatic failover demo (MAIN ↔ GEN) using relays and voltage
    Demo_Failover();
#endif

#if DEMO_GENSIM
    // Generator simulator LED-only startup/shutdown demo
    Demo_GenSim();
#endif

#if DEMO_FINAL_COMBINED
    // Final combined project behavior (future)
    Demo_FinalCombined();
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
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

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN_USART2_Init 2 */

  /* USER CODE END_USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RELAY_MAIN_Pin|RELAY_GEN_Pin|RELAY_TRANSFER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_PWR_Pin|LED_S1_Pin|LED_S2_Pin|LED_S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // polling mode, no interrupt
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RELAY_MAIN_Pin RELAY_GEN_Pin RELAY_TRANSFER_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RELAY_MAIN_Pin|RELAY_GEN_Pin|RELAY_TRANSFER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PWR_Pin LED_S1_Pin LED_S2_Pin LED_S3_Pin */
  GPIO_InitStruct.Pin = LED_PWR_Pin|LED_S1_Pin|LED_S2_Pin|LED_S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Additional GPIO user init (if needed) can go here.
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
