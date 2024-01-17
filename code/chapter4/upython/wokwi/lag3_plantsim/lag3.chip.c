// lag3.chip.c  
// Custom chip for LAG3 plant simulation
// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
// Copyright 2023 Dew Toochinda

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
  pin_t pin_yout;
  pin_t pin_x1out;
  pin_t pin_x2out;
  pin_t pin_uin;
  pin_t pin_reset;
} chip_state_t;

// --------------- plant simulation variables ----------------------
float T = 0.08;       // sampling period
uint32_t T_us;  // sampling period in microseconds
struct PlantSim
{
  float a11, a21, a31, b11,b21,b31;   // coefficients
  float x1_1,x1_0, x2_1, x2_0, x3_1, x3_0;       // states
  float u0, u1;           // input (from controller output)
};

struct PlantSim PSim;   // create instance

void PSim_init(void)  {  // initialize lag3 coefficients
  T_us = T*1e6; // in microseconds
  float a = (2+T);
  float b = (2-T);    
  PSim.a11 = PSim.a21 = PSim.a31 = b/a;
  PSim.b11 = PSim.b21 = PSim.b31 = T/a; 
  PSim.x3_0 = 0;
  PSim.x2_0 = 0;
  PSim.x1_0 = 0;
  PSim.x3_1 = 0;
  PSim.x2_1 = 0;
  PSim.x1_1 = 0;
}

static void chip_timer_event(void *user_data);

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  chip->pin_uin = pin_init("U", ANALOG);
  chip->pin_yout = pin_init("Y", ANALOG);
  chip->pin_x1out = pin_init("X1", ANALOG);
  chip->pin_x2out = pin_init("X2", ANALOG);
  chip->pin_reset = pin_init("RESET",INPUT_PULLDOWN);
  PSim_init();
  const timer_config_t timer_config = {
    .callback = chip_timer_event,
    .user_data = chip,
  };
  timer_t timer_id = timer_init(&timer_config);
  timer_start(timer_id, T_us, true);
}

void chip_timer_event(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  uint32_t reset = pin_read(chip->pin_reset);
  if (reset) PSim_init();
  float r = pin_adc_read(chip->pin_uin);
  PSim.u1 = PSim.u0;
  PSim.u0 = r;
  // update plant states
  PSim.x3_1 = PSim.x3_0;
  PSim.x2_1 = PSim.x2_0;
  PSim.x1_1 = PSim.x1_0;
  PSim.x1_0 = PSim.a11*PSim.x1_1 + PSim.b11*(PSim.u0 + PSim.u1);
  PSim.x2_0 = PSim.a21*PSim.x2_1 + PSim.b21*(PSim.x1_0 + PSim.x1_1);
  PSim.x3_0 = PSim.a31*PSim.x3_1 + PSim.b31*(PSim.x2_0 + PSim.x2_1);
  pin_dac_write(chip->pin_x1out, PSim.x1_0);
  pin_dac_write(chip->pin_x2out, PSim.x2_0);
  pin_dac_write(chip->pin_yout, PSim.x3_0);
}
