#ifndef VOLTAGE_READ_H
#define VOLTAGE_READ_H

#include <stdint.h>

/* Returns RMS of the AC component at the ZMPT module output (volts RMS at module) */
float voltage_get_module_rms(uint16_t samples);

/* Returns estimated line voltage RMS (volts RMS line)*/
float voltage_get_line_vrms(uint16_t samples);

#endif /* VOLTAGE_READ_H */