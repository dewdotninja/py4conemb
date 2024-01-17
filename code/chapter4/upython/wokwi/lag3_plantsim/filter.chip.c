// filter.chip.c
// Moving average or Exponentially weighted average
// implementation to convert PWM to analog signal.
// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
// Copyright 2023 Dew Ninja

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

//float v_0 = 0, v_1 = 0;
// int k = 0;  // index
// int N = 4;  // number of average4
// float buf[4];
// float sum_x = 0;  

typedef struct {
  pin_t pin_in;
  pin_t pin_out;
  uint32_t beta_attr;  
  uint32_t filtype_attr;
  int k;
  int N;
  float buf[10];
  float sum_x;
  float v_0;
  float v_1;
} chip_state_t;

static void chip_timer_event(void *user_data);

// static void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
//   chip_state_t *chip = (chip_state_t*)user_data;
//   //printf("Pin change: %d %d\n", pin, value);
//   pin_write(chip->pin_out, !value);
// }

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  chip->pin_in = pin_init("IN", INPUT);
 // chip->pin_ain = pin_init("ADCIN", ANALOG);

  chip->pin_out = pin_init("OUT", ANALOG);
//  chip->pin_aout = pin_init("DACOUT", ANALOG);
  chip->beta_attr = attr_init_float("beta", 0.9);
  chip->filtype_attr = attr_init("filter_type", 0);

  chip->k = 0;
  chip->N = 10;
  chip->sum_x = 0;
  chip->v_0 = 0;
  chip->v_1 = 0;
  for (int i=0;i<10;i++)
    chip->buf[i]=0.0;

  //pin_write(chip->pin_out, !pin_read(chip->pin_in));
  const timer_config_t timer_config = {
    .callback = chip_timer_event,
    .user_data = chip,
  };
  timer_t timer_id = timer_init(&timer_config);
  timer_start(timer_id, 100, true);
  // const pin_watch_config_t config = {
  //   .edge = BOTH,
  //   .pin_change = chip_pin_change,
  //   .user_data = chip,
  // };
  // pin_watch(chip->pin_in, &config);
}

void chip_timer_event(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  float beta = attr_read_float(chip->beta_attr); // EWA parameter
  uint32_t ftype = attr_read(chip->filtype_attr); // filter type  
  float x = pin_read(chip->pin_in);
  if (ftype == 0)  {  // MA
    chip->sum_x -= chip->buf[chip->k];
    chip->buf[chip->k] = x;
    chip->sum_x += x;
    float ma_val = chip->sum_x/chip->N;
    chip->k++;
    if (chip->k==chip->N) chip->k = 0;  
    pin_dac_write(chip->pin_out, 4.5*ma_val); 
  }
  else   {  // EWA
    chip->v_0 = beta*chip->v_1 + (1-beta)*x;
    chip->v_1 = chip->v_0;
    pin_dac_write(chip->pin_out, 4.5*chip->v_0); 
  }
}
