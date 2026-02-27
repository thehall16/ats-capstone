#include "gensim.h"
#include "main.h"
#include "uart_display.h"

// GenSim timing constants (ms)
#define GENSIM_STARTUP_S1_TIME_MS       1000U   // S1 solid at start
#define GENSIM_STARTUP_CRANK_TIME_MS    3000U   // S2 blink (crank)
#define GENSIM_SHUT_S1_SOLID_TIME_MS    1000U   // S1+S3 solid at shutdown start
#define GENSIM_SHUT_S2_BLINK_TIME_MS    3000U   // S2 blink (cooldown)
#define GENSIM_BLINK_PERIOD_MS          200U    // S2 blink period

/* LED helpers (moved from main.c) */
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

/* Internal state */
typedef enum {
    GS_IDLE = 0,
    GS_START_S1,
    GS_START_CRANK,
    GS_RUNNING,
    GS_SHUT_S1,
    GS_SHUT_COOLDOWN,
    GS_STOPPED
} GenSimState_t;

static GenSimState_t   s_state = GS_IDLE;
static uint32_t        s_stateStartTick = 0;
static uint32_t        s_lastBlinkTick  = 0;
static uint8_t         s_s2On           = 0;

static GenSimStatus_t  s_status  = GSTAT_IDLE;
static uint8_t         s_running = 0;

void GenSim_Init(void)
{
    s_state = GS_IDLE;
    s_stateStartTick = 0;
    s_lastBlinkTick = 0;
    s_s2On = 0;

    s_status = GSTAT_IDLE;
    s_running = 0;

    Status_AllOff();
}

GenSimStatus_t GenSim_GetStatus(void)
{
    return s_status;
}

uint8_t GenSim_IsRunning(void)
{
    return s_running;
}


/*
 * GenSim_Update(requestRun)
 *
 * requestRun = 1 -> we WANT the generator running
 * requestRun = 0 -> we WANT the generator stopped
 *
 * Returns:
 *   1 -> GenSim is in RUNNING S3 (generator “up”)
 *   0 -> any other state
 */
uint8_t GenSim_Update(uint8_t requestRun)
{
    uint32_t now = HAL_GetTick();

    switch (s_state)
    {
        case GS_IDLE:
            Status_AllOff();
            s_status = GSTAT_IDLE;

            if (requestRun)
            {
                Status_Set(1, 0, 0);
                s_status = GSTAT_STARTUP_S1;
                s_stateStartTick = now;
                s_state = GS_START_S1;
                uart_printf("GenSim: STARTUP requested\r\n");
            }
            break;

        case GS_START_S1:
            Status_Set(1, 0, 0);
            s_status = GSTAT_STARTUP_S1;

            if (!requestRun)
            {
                Status_AllOff();
                s_status = GSTAT_STOPPED;
                s_state = GS_STOPPED;
                break;
            }

            if (now - s_stateStartTick >= GENSIM_STARTUP_S1_TIME_MS)
            {
                s_s2On = 0;
                s_lastBlinkTick  = now;
                s_stateStartTick = now;
                s_status = GSTAT_STARTUP_CRANK;
                s_state = GS_START_CRANK;
                uart_printf("GenSim: crank phase (S2 blink)\r\n");
            }
            break;

        case GS_START_CRANK:
            if (!requestRun)
            {
                Status_AllOff();
                s_status = GSTAT_STOPPED;
                s_state = GS_STOPPED;
                break;
            }

            if (now - s_lastBlinkTick >= GENSIM_BLINK_PERIOD_MS)
            {
                s_s2On = !s_s2On;
                s_lastBlinkTick = now;
            }
            Status_Set(1, s_s2On, 0);
            s_status = GSTAT_STARTUP_CRANK;

            if (now - s_stateStartTick >= GENSIM_STARTUP_CRANK_TIME_MS)
            {
                Status_Set(0, 0, 1);
                s_status = GSTAT_RUNNING;
                s_stateStartTick = now;
                s_state = GS_RUNNING;
                uart_printf("GenSim: generator RUNNING (S3 ON)\r\n");
            }
            break;

        case GS_RUNNING:
            Status_Set(0, 0, 1);
            s_status = GSTAT_RUNNING;

            if (!requestRun)
            {
                Status_Set(1, 0, 1);
                s_status = GSTAT_SHUT_S1;
                s_stateStartTick = now;
                s_state = GS_SHUT_S1;
                uart_printf("GenSim: SHUTDOWN requested\r\n");
            }
            break;

        case GS_SHUT_S1:
            Status_Set(1, 0, 1);
            s_status = GSTAT_SHUT_S1;

            if (requestRun)
            {
                Status_Set(0, 0, 1);
                s_status = GSTAT_RUNNING;
                s_state = GS_RUNNING;
                break;
            }

            if (now - s_stateStartTick >= GENSIM_SHUT_S1_SOLID_TIME_MS)
            {
                s_s2On = 0;
                s_lastBlinkTick  = now;
                s_stateStartTick = now;
                s_status = GSTAT_SHUT_COOLDOWN;
                s_state = GS_SHUT_COOLDOWN;
                uart_printf("GenSim: cooldown (S2 blink)\r\n");
            }
            break;

        case GS_SHUT_COOLDOWN:
            if (requestRun)
            {
                Status_Set(0, 0, 1);
                s_status = GSTAT_RUNNING;
                s_state = GS_RUNNING;
                break;
            }

            if (now - s_lastBlinkTick >= GENSIM_BLINK_PERIOD_MS)
            {
                s_s2On = !s_s2On;
                s_lastBlinkTick = now;
            }
            Status_Set(1, s_s2On, 1);
            s_status = GSTAT_SHUT_COOLDOWN;

            if (now - s_stateStartTick >= GENSIM_SHUT_S2_BLINK_TIME_MS)
            {
                Status_AllOff();
                s_status = GSTAT_STOPPED;
                s_state = GS_STOPPED;
                uart_printf("GenSim: generator STOPPED\r\n");
            }
            break;

        case GS_STOPPED:
            Status_AllOff();
            s_status = GSTAT_STOPPED;

            if (requestRun)
            {
                Status_Set(1, 0, 0);
                s_status = GSTAT_STARTUP_S1;
                s_stateStartTick = now;
                s_state = GS_START_S1;
                uart_printf("GenSim: STARTUP requested (from STOPPED)\r\n");
            }
            break;

        default:
            s_state = GS_IDLE;
            Status_AllOff();
            s_status = GSTAT_IDLE;
            break;
    }

    s_running = (s_status == GSTAT_RUNNING);
    return s_running;
}