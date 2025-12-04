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

// High-level GenSim status for UART printing
typedef enum
{
    GSTAT_IDLE = 0,
    GSTAT_STARTUP_S1,
    GSTAT_STARTUP_CRANK,
    GSTAT_RUNNING,
    GSTAT_SHUT_S1,
    GSTAT_SHUT_COOLDOWN,
    GSTAT_STOPPED
} GenSimStatus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * Demo mode switches.
 * Enable only ONE main demo at a time (except ButtonLed, which is always safe).
 */
#define DEMO_VOLTAGE_TEST      0   // ZMPT + RMS + UART demo
#define DEMO_RELAY_TEST        0   // Simple relay pattern demo
#define DEMO_GENSIM_ONLY       0   // Standalone GenSim LED sequence
#define DEMO_FINAL_ATS         1   // Full ATS + GenSim + relays

// Hard-coded ADC DC offset (midpoint) for the ZMPT output.
// Based on initial no-AC measurement (approx. 3080).
const uint16_t ADC_OFFSET = 3080;

// From calibration: ~0.61 Vrms at module OUT when line is 120 Vrms.
// 120 / 0.61 ≈ 196.7  --> 185.5f more accurate after tuning
const float lineScaleFactor = 185.5f;      // converts module Vrms -> line Vrms

// GenSim timing constants (ms)
#define GENSIM_STARTUP_S1_TIME_MS       1000U   // S1 solid at start
#define GENSIM_STARTUP_CRANK_TIME_MS    3000U   // S2 blink (crank)
#define GENSIM_SHUT_S1_SOLID_TIME_MS    1000U   // S1+S3 solid at shutdown start
#define GENSIM_SHUT_S2_BLINK_TIME_MS    3000U   // S2 blink (cooldown)
#define GENSIM_BLINK_PERIOD_MS          200U    // S2 blink period

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

// Global GenSim status for UART printing
GenSimStatus_t g_GenSimStatus = GSTAT_IDLE;

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
void Demo_GenSimOnly(void);       // Standalone GenSim startup & shutdown loop
void Demo_FinalCombined(void);    // Final ATS + GenSim + relays

// GenSim update (non-blocking visual engine)
uint8_t GenSim_Update(uint8_t requestRun);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart_printf(const char *fmt, ...)
{
    char buffer[512];  // bigger buffer so ASCII boxes don't get chopped
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
 * These drive the actual relay outputs (or LEDs hung from RELAY_* pins).
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
    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_SET);
}

/*
 * Status LED helpers (GenSim visuals).
 * These use LED_S1/S2/S3 but are NOT used to represent the real relays.
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
 * GenSim_Update(requestRun)
 *
 * requestRun = 1 → we WANT the generator running
 *   - if currently idle/stopped, runs startup animation:
 *       S1 ON (1s), then S1+S2 crank blink (3s), then S3 ON = RUNNING
 *   - then stays in RUNNING S3 as long as requestRun stays 1
 *
 * requestRun = 0 → we WANT the generator stopped
 *   - if currently running, runs shutdown animation:
 *       S1+S3 ON (1s), then S1+S3 with S2 blink (3s), then all OFF = STOPPED
 *   - then stays in STOPPED as long as requestRun stays 0
 *
 * Returns:
 *   1 → GenSim is in RUNNING S3 (generator “up”)
 *   0 → any other state (idle, starting, shutting down, stopped)
 */
uint8_t GenSim_Update(uint8_t requestRun)
{
    typedef enum {
        GS_IDLE = 0,
        GS_START_S1,
        GS_START_CRANK,
        GS_RUNNING,
        GS_SHUT_S1,
        GS_SHUT_COOLDOWN,
        GS_STOPPED
    } GenSimState_t;

    static GenSimState_t s = GS_IDLE;
    static uint32_t stateStartTick = 0;
    static uint32_t lastBlinkTick  = 0;
    static uint8_t  s2On           = 0;

    uint32_t now = HAL_GetTick();

    switch (s)
    {
        case GS_IDLE:
            Status_AllOff();
            g_GenSimStatus = GSTAT_IDLE;

            if (requestRun)
            {
                Status_Set(1, 0, 0);               // S1 ON
                g_GenSimStatus = GSTAT_STARTUP_S1;
                stateStartTick = now;
                s = GS_START_S1;
                uart_printf("GenSim: STARTUP requested\r\n");
            }
            break;

        case GS_START_S1:
            Status_Set(1, 0, 0);
            g_GenSimStatus = GSTAT_STARTUP_S1;

            if (!requestRun)
            {
                Status_AllOff();
                g_GenSimStatus = GSTAT_STOPPED;
                s = GS_STOPPED;
                break;
            }

            if (now - stateStartTick >= GENSIM_STARTUP_S1_TIME_MS)
            {
                s2On = 0;
                lastBlinkTick  = now;
                stateStartTick = now;
                g_GenSimStatus = GSTAT_STARTUP_CRANK;
                s = GS_START_CRANK;
                uart_printf("GenSim: crank phase (S2 blink)\r\n");
            }
            break;

        case GS_START_CRANK:
            if (!requestRun)
            {
                Status_AllOff();
                g_GenSimStatus = GSTAT_STOPPED;
                s = GS_STOPPED;
                break;
            }

            if (now - lastBlinkTick >= GENSIM_BLINK_PERIOD_MS)
            {
                s2On = !s2On;
                lastBlinkTick = now;
            }
            Status_Set(1, s2On, 0);
            g_GenSimStatus = GSTAT_STARTUP_CRANK;

            if (now - stateStartTick >= GENSIM_STARTUP_CRANK_TIME_MS)
            {
                Status_Set(0, 0, 1);
                g_GenSimStatus = GSTAT_RUNNING;
                stateStartTick = now;
                s = GS_RUNNING;
                uart_printf("GenSim: generator RUNNING (S3 ON)\r\n");
            }
            break;

        case GS_RUNNING:
            Status_Set(0, 0, 1);
            g_GenSimStatus = GSTAT_RUNNING;

            if (!requestRun)
            {
                Status_Set(1, 0, 1);   // S1 + S3 ON
                g_GenSimStatus = GSTAT_SHUT_S1;
                stateStartTick = now;
                s = GS_SHUT_S1;
                uart_printf("GenSim: SHUTDOWN requested\r\n");
            }
            break;

        case GS_SHUT_S1:
            Status_Set(1, 0, 1);
            g_GenSimStatus = GSTAT_SHUT_S1;

            if (requestRun)
            {
                Status_Set(0, 0, 1);
                g_GenSimStatus = GSTAT_RUNNING;
                s = GS_RUNNING;
                break;
            }

            if (now - stateStartTick >= GENSIM_SHUT_S1_SOLID_TIME_MS)
            {
                s2On = 0;
                lastBlinkTick  = now;
                stateStartTick = now;
                g_GenSimStatus = GSTAT_SHUT_COOLDOWN;
                s = GS_SHUT_COOLDOWN;
                uart_printf("GenSim: cooldown (S2 blink)\r\n");
            }
            break;

        case GS_SHUT_COOLDOWN:
            if (requestRun)
            {
                Status_Set(0, 0, 1);
                g_GenSimStatus = GSTAT_RUNNING;
                s = GS_RUNNING;
                break;
            }

            if (now - lastBlinkTick >= GENSIM_BLINK_PERIOD_MS)
            {
                s2On = !s2On;
                lastBlinkTick = now;
            }
            Status_Set(1, s2On, 1);
            g_GenSimStatus = GSTAT_SHUT_COOLDOWN;

            if (now - stateStartTick >= GENSIM_SHUT_S2_BLINK_TIME_MS)
            {
                Status_AllOff();
                g_GenSimStatus = GSTAT_STOPPED;
                s = GS_STOPPED;
                uart_printf("GenSim: generator STOPPED\r\n");
            }
            break;

        case GS_STOPPED:
            Status_AllOff();
            g_GenSimStatus = GSTAT_STOPPED;

            if (requestRun)
            {
                Status_Set(1, 0, 0);               // S1 ON
                g_GenSimStatus = GSTAT_STARTUP_S1;
                stateStartTick = now;
                s = GS_START_S1;
                uart_printf("GenSim: STARTUP requested (from STOPPED)\r\n");
            }
            break;

        default:
            s = GS_IDLE;
            Status_AllOff();
            g_GenSimStatus = GSTAT_IDLE;
            break;
    }

    return (g_GenSimStatus == GSTAT_RUNNING);
}

/*
 * Standalone GenSim demo (just loops startup → hold → shutdown).
 */
void Demo_GenSimOnly(void)
{
    static uint8_t wantRun     = 0;
    static uint32_t lastToggle = 0;
    uint32_t now = HAL_GetTick();

    // Toggle desired state every 8 seconds for demo
    if (now - lastToggle >= 8000)
    {
        lastToggle = now;
        wantRun = !wantRun;
        uart_printf("Demo_GenSimOnly: wantRun = %d\r\n", wantRun);
    }

    GenSim_Update(wantRun);
}

/*
 * FINAL ATS DEMO:
 * - Watches line voltage and switches MAIN ↔ GEN with delays.
 * - Uses GenSim_Update() to animate generator start/stop on LED_S1/S2/S3.
 * - Drives RELAY_MAIN / RELAY_GEN / RELAY_TRANSFER.
 */
void Demo_FinalCombined(void)
{
    typedef enum {
        ATS_MAIN = 0,     // load on MAIN
        ATS_TO_GEN,       // transitioning MAIN → GEN
        ATS_GEN,          // load on GEN
        ATS_TO_MAIN       // transitioning GEN → MAIN
    } AtsState_t;

    static AtsState_t atsState       = ATS_MAIN;
    static uint32_t   badStartTick   = 0;
    static uint32_t   goodStartTick  = 0;
    static uint32_t   transStartTick = 0;
    static uint32_t   lastPrint      = 0;

    uint32_t now = HAL_GetTick();

    // Measure line voltage
    float v_ac_rms  = get_ac_rms(12000);
    float line_vrms = v_ac_rms * lineScaleFactor;
    if (line_vrms < 5.0f)
        line_vrms = 0.0f;

    // This flag tells GenSim whether gen "should" be running
    uint8_t requestGenRun = 0;

    switch (atsState)
    {
        case ATS_MAIN:
            Relay_MainOn();
            requestGenRun = 0;

            // Detect bad mains (<108 or >132) for >500 ms
            if (line_vrms < 108.0f || line_vrms > 132.0f)
            {
                if (badStartTick == 0)
                    badStartTick = now;

                if (now - badStartTick >= 500)
                {
                    uart_printf("ATS: MAIN voltage out of range (%.1f V).\r\n", line_vrms);
                    uart_printf("ATS: Transitioning to GEN.\r\n");

                    Relay_AllOff();
                    transStartTick = now;
                    atsState = ATS_TO_GEN;

                    badStartTick  = 0;
                    goodStartTick = 0;
                }
            }
            else
            {
                badStartTick = 0;
            }
            break;

        case ATS_TO_GEN:
            // We WANT generator running; GenSim will animate startup
            requestGenRun = 1;

            // 0–1000 ms: all OFF
            // 1000–2000 ms: GEN ON only
            // >=2000 ms: GEN + TRANSFER ON → load on GEN
            if (now - transStartTick < 1000)
            {
                Relay_AllOff();
            }
            else if (now - transStartTick < 2000)
            {
                Relay_GenOn();
                HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
            }
            else
            {
                Relay_GenOn();
                Relay_TransferOn();
                atsState = ATS_GEN;
                uart_printf("ATS: Load now on GEN.\r\n");
            }
            break;

        case ATS_GEN:
            Relay_GenOn();
            Relay_TransferOn();
            requestGenRun = 1;

            // Look for good mains in tight window [117,123] for 2 seconds
            if (line_vrms >= 115.0f && line_vrms <= 125.0f)
            {
                if (goodStartTick == 0)
                    goodStartTick = now;

                if (now - goodStartTick >= 2000)
                {
                    uart_printf("ATS: MAIN voltage stable (%.1f V).\r\n", line_vrms);
                    uart_printf("ATS: Transitioning back to MAIN.\r\n");

                    // Remove load from generator first
                    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);

                    transStartTick = now;
                    atsState = ATS_TO_MAIN;

                    badStartTick  = 0;
                    goodStartTick = 0;
                }
            }
            else
            {
                goodStartTick = 0;
            }
            break;

        case ATS_TO_MAIN:
            // During transition back, we no longer want gen running
            requestGenRun = 0;

            // 0–1000 ms: GEN ON (cooldown), TRANSFER OFF
            // 1000–2000 ms: GEN OFF, all open
            // >=2000 ms: MAIN ON
            if (now - transStartTick < 1000)
            {
                Relay_GenOn();
                HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin,     GPIO_PIN_RESET);
            }
            else if (now - transStartTick < 2000)
            {
                Relay_AllOff();
            }
            else
            {
                Relay_MainOn();
                atsState = ATS_MAIN;
                uart_printf("ATS: Load returned to MAIN.\r\n");
            }
            break;

        default:
            atsState = ATS_MAIN;
            Relay_AllOff();
            Relay_MainOn();
            requestGenRun = 0;
            badStartTick  = 0;
            goodStartTick = 0;
            break;
    }

    // Update GenSim LEDs (purely visual, never blocks ATS)
    GenSim_Update(requestGenRun);

    // Periodic UART status (once per 2 seconds)
    if (now - lastPrint >= 2000)
    {
        lastPrint = now;

        int mainOn     = (HAL_GPIO_ReadPin(RELAY_MAIN_GPIO_Port,     RELAY_MAIN_Pin)     == GPIO_PIN_SET);
        int genOn      = (HAL_GPIO_ReadPin(RELAY_GEN_GPIO_Port,      RELAY_GEN_Pin)      == GPIO_PIN_SET);
        int transferOn = (HAL_GPIO_ReadPin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin) == GPIO_PIN_SET);

        const char *srcStr;
        switch (atsState)
        {
            case ATS_MAIN:     srcStr = "MAIN"; break;
            case ATS_GEN:      srcStr = "GEN "; break;
            case ATS_TO_GEN:	srcStr = "TRAN"; break;
            case ATS_TO_MAIN:  srcStr = "TRAN"; break;
            default:           srcStr = "MIX?"; break;
        }

        const char *genSimStr;
        switch (g_GenSimStatus)
        {
            case GSTAT_IDLE:            genSimStr = "IDLE"; break;
            case GSTAT_STARTUP_S1:      genSimStr = "STARTUP S1"; break;
            case GSTAT_STARTUP_CRANK:   genSimStr = "CRANK S2"; break;
            case GSTAT_RUNNING:         genSimStr = "RUNNING S3"; break;
            case GSTAT_SHUT_S1:         genSimStr = "SHUTDOWN S1"; break;
            case GSTAT_SHUT_COOLDOWN:   genSimStr = "COOLDOWN S2"; break;
            case GSTAT_STOPPED:         genSimStr = "STOPPED"; break;
            default:                    genSimStr = "UNKNOWN"; break;
        }

        uart_printf(
            "+-------------------------------------------+\r\n"
            "|        FINAL ATS DEMO: STATUS             |\r\n"
            "+-------------------------------------------+\r\n"
            "| Line Voltage: %6.1f V RMS                 |\r\n"
            "| Source: %-4s                              |\r\n"
            "| MAIN Relay:     %s                        |\r\n"
            "| GEN Relay:      %s                        |\r\n"
            "| TRANSFER Relay: %s                        |\r\n"
            "| GenSim Status:  %-13s           |\r\n"
            "+-------------------------------------------+\r\n",
            line_vrms,
            srcStr,
            mainOn     ? "ON " : "OFF",
            genOn      ? "ON " : "OFF",
            transferOn ? "ON " : "OFF",
            genSimStr
        );
    }
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
  uart_printf("Demo config: V=%d, R=%d, G=%d, F=%d\r\n",
              DEMO_VOLTAGE_TEST,
              DEMO_RELAY_TEST,
              DEMO_GENSIM_ONLY,
              DEMO_FINAL_ATS);

  // Grab initial button state for edge detection demo
  lastButtonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

  // Turn on power LED so we know the board/PCB has power
  HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_SET);

  // Ensure status LEDs and relay drivers start in a known OFF state
  Status_AllOff();
  Relay_AllOff();
  Relay_MainOn();   // default: on mains

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
     * - Enable exactly ONE of the main DEMO_* modes at the top.
     */

    // Keep the button → LD2 demo active in all modes
    Demo_ButtonLed();

#if DEMO_VOLTAGE_TEST
    Demo_Voltage();
#endif

#if DEMO_RELAY_TEST
    Demo_RelayTest();
#endif

#if DEMO_GENSIM_ONLY
    Demo_GenSimOnly();
#endif

#if DEMO_FINAL_ATS
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
  sConfig.Channel      = ADC_CHANNEL_5;         // PA0 / IN5 for ZMPT
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
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
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // polling mode, no interrupt
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RELAY_MAIN_Pin RELAY_GEN_Pin RELAY_TRANSFER_Pin */
  GPIO_InitStruct.Pin   = LD2_Pin|RELAY_MAIN_Pin|RELAY_GEN_Pin|RELAY_TRANSFER_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PWR_Pin LED_S1_Pin LED_S2_Pin LED_S3_Pin */
  GPIO_InitStruct.Pin   = LED_PWR_Pin|LED_S1_Pin|LED_S2_Pin|LED_S3_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Additional GPIO user init (if needed) can go here.
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* extra helper functions if needed are already above */
/* USER CODE END 4 */

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
