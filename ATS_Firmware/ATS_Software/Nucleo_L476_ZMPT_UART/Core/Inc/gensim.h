#ifndef GENSIM_H
#define GENSIM_H

#include <stdint.h>

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

void GenSim_Init(void);

/* requestRun = 1 -> want generator running, 0 -> want stopped
   returns 1 when generator is in RUNNING state, else 0 */
uint8_t GenSim_Update(uint8_t requestRun);

/* accessors so ATS can print status / check running */
GenSimStatus_t GenSim_GetStatus(void);
uint8_t GenSim_IsRunning(void);

#endif /* GENSIM_H */