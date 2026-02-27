#include "ats_ctrl.h"
#include "main.h"
#include "uart_display.h"
#include "voltage_read.h"
#include "gensim.h"

#include <stdint.h>

/* Relay helper functions (moved with ATS) */
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
 * ATS_Task:
 * - Watches line voltage and switches MAIN <-> GEN with delays.
 * - Uses GenSim_Update() to animate generator start/stop on LED_S1/S2/S3.
 * - Drives RELAY_MAIN / RELAY_GEN / RELAY_TRANSFER.
 * - Ensures GEN & TRANSFER are both blocked until GenSim reports RUNNING.
 */
void ATS_Task(void)
{
    typedef enum {
        ATS_MAIN = 0,
        ATS_TO_GEN,
        ATS_GEN,
        ATS_TO_MAIN
    } AtsState_t;

    static AtsState_t atsState       = ATS_MAIN;
    static uint32_t   badStartTick   = 0;
    static uint32_t   goodStartTick  = 0;
    static uint32_t   transStartTick = 0;
    static uint32_t   lastPrint      = 0;
    static uint32_t   genOnTick      = 0;

    uint32_t now = HAL_GetTick();

    /* Line voltage */
    float line_vrms = voltage_get_line_vrms(12000);

    /* Tell GenSim what we want */
    uint8_t requestGenRun = 0;

    switch (atsState)
    {
        case ATS_MAIN:
            Relay_MainOn();
            requestGenRun = 0;

            if (line_vrms < 108.0f || line_vrms > 132.0f)
            {
                if (badStartTick == 0) badStartTick = now;

                if (now - badStartTick >= 500)
                {
                    uart_printf("ATS: MAIN voltage out of range (%.1f V).\r\n", line_vrms);
                    uart_printf("ATS: Transitioning to GEN.\r\n");

                    Relay_AllOff();
                    transStartTick = now;
                    atsState = ATS_TO_GEN;

                    badStartTick  = 0;
                    goodStartTick = 0;
                    genOnTick     = 0;
                }
            }
            else
            {
                badStartTick = 0;
            }
            break;

        case ATS_TO_GEN:
            requestGenRun = 1;

            /* Until GenSim says RUNNING, keep relays open */
            if (!GenSim_IsRunning())
            {
                Relay_AllOff();
                genOnTick = 0;
                break;
            }

            /* Close GEN relay first, then TRANSFER after 1s */
            if (genOnTick == 0)
            {
                genOnTick = now;
                Relay_GenOn();
                HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
                break;
            }

            if (now - genOnTick < 1000)
            {
                Relay_GenOn();
                HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);
            }
            else
            {
                Relay_GenOn();
                Relay_TransferOn();
                atsState = ATS_GEN;
                uart_printf("ATS: Load now on GEN (GenSim RUNNING & GEN relay ON).\r\n");
            }
            break;

        case ATS_GEN:
            Relay_GenOn();
            Relay_TransferOn();
            requestGenRun = 1;

            if (line_vrms >= 115.0f && line_vrms <= 125.0f)
            {
                if (goodStartTick == 0) goodStartTick = now;

                if (now - goodStartTick >= 2000)
                {
                    uart_printf("ATS: MAIN voltage stable (%.1f V).\r\n", line_vrms);
                    uart_printf("ATS: Transitioning back to MAIN.\r\n");

                    HAL_GPIO_WritePin(RELAY_TRANSFER_GPIO_Port, RELAY_TRANSFER_Pin, GPIO_PIN_RESET);

                    transStartTick = now;
                    atsState = ATS_TO_MAIN;

                    badStartTick  = 0;
                    goodStartTick = 0;
                    genOnTick     = 0;
                }
            }
            else
            {
                goodStartTick = 0;
            }
            break;

        case ATS_TO_MAIN:
            requestGenRun = 0;

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
            genOnTick     = 0;
            break;
    }

    /* Update GenSim (non-blocking) */
    GenSim_Update(requestGenRun);

    /* Periodic UART status */
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
            case ATS_TO_GEN:   srcStr = "TRAN"; break;
            case ATS_TO_MAIN:  srcStr = "TRAN"; break;
            default:           srcStr = "MIX?"; break;
        }

        const char *genSimStr;
        switch (GenSim_GetStatus())
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
            "| Line Voltage: %6.1f V RMS                |\r\n"
            "| Source: %-4s                              |\r\n"
            "| MAIN Relay:     %s                       |\r\n"
            "| GEN Relay:      %s                       |\r\n"
            "| TRANSFER Relay: %s                       |\r\n"
            "| GenSim Status:  %-13s             |\r\n"
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