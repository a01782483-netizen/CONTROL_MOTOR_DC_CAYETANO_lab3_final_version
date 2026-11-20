#ifndef __DEFINITION_H__
#define __DEFINITION_H__

#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <SimpleUART.h>
#include "H_bridge.h"
#include "QuadratureEncoder.h"
#include "PID.h"


SimpleUART uart(115200);
H_bridge motor;
QuadratureEncoder encoder;
PID controler;

int64_t prev_time, current_time;
int64_t dt_us = 20000;
int64_t send_dt_us = 20000;
int64_t prev_send_time = 0;

//MOTOR
uint8_t in[] = {25,26};
uint8_t ch[] = {0,1};
ledc_timer_t timer = LEDC_TIMER_0;
float threshold = 56.0f;
//float duty = 0.0f;

//ENCODER
uint8_t pins[] = {32,33};
float PPR = 495.45;
float degrees_per_edge = 360/(2.0f * PPR);
//float speed = 0.0f;

//PID 
char mode = 'P';
float gains[] = {1.3f, 0.4f, 0.0f};
float reference = 0.0f;
int64_t position_dt_us = 20000;

float rpm_filtrada = 0.0f;
const float tau_filtro = 0.02f; // Constante de tiempo: 20 ms.

//UART
char message[64];
int pos = 0;

//State_machine
float rpm = 0.0f;
float error = 0.0f;
float duty = 0.0f;




#endif // __DEFINITION_H__