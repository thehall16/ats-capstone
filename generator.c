#include "generator.h"
#include <stdio.h>
#include "stm32c0xx_hal.h" // HAL functions

//Pin assignments
GPIO_TypeDef* GEN_PORT = GPIOA;      // GPIO port for generator pins
uint16_t GEN_CHOKE_PIN = GPIO_PIN_4; // Choke control
uint16_t GEN_START_PIN = GPIO_PIN_3; // Start control
uint16_t GEN_RUN_PIN   = GPIO_PIN_5; // Run signal

//Delay variables
#define CHOKE_DELAY    2000  // Choke duration in ms
#define START_DELAY    4000  // Start/crank duration in ms
#define RUN_DELAY      1000  // Stabilization before SSR

//Functions
void generator_startup_sequence(void){

    //Choke Sim

    //turn on choke indicator LED light
    wait(CHOKE_DELAY);
    
    //Startup Sim

    //turn off choke indicator
    wait(START_DELAY);

    //Running

    //turn on running light
    wait(RUN_DELAY);
    


}

void generator_stop(void){

    //stop indicator LED

}