/**
 * File : hardwareconfig.h
 * Version : 8.2.0
 * Date    : 17/06/2020
 * Author  : AnDX
 * Description :
 * 
 *******************************************************/

#ifndef __HARDWARECONFIG_H
#define __HARDWARECONFIG_H

#include <Arduino.h>
#include "HardwareSerial.h"
#include "CAN_config.h"

#define DEBUG_SERIAL_ENABLE

#ifdef DEBUG_SERIAL_ENABLE
    #define DebugSrial Serial 
#endif

#define ID_OC   0x04
#define ID_RTC  0x01

#define CAN_BAUD_SPEED  CAN_SPEED_125KBPS
#define CAN_TX          GPIO_NUM_5
#define CAN_RX          GPIO_NUM_4
#define CAN_FRAME       CAN_frame_std
#define CAN_ID          ID_OC
#define CAN_SEND_SIZE   8
#define CAN_TIMER       60
#define TIME_OUT        50000

#define SENSOR_1        13 // 1
#define SENSOR_2        12 // 1
#define SENSOR_3        15 // 1
#define SENSOR_4        18 // 1
#define SENSOR_5        21 // 1
#define SENSOR_6        22 // 1
#define SENSOR_7        23 // 1

// ---------------------
#define EMERGENCY_OUT   19 //1

#define revert1 0
#define revert2 0
// -- CONVAYER No.1
#define CTR_ENABLE1     14 //1

#if revert1
    #define CTR_PWM_H1      27 // -
    #define CTR_PWM_L1      26 // -
#else
    #define CTR_PWM_H1      26 // - origin
    #define CTR_PWM_L1      27 // -
#endif

#define SS_LIMMIT_AHEAD1     SENSOR_1
#define SS_LIMMIT_BEHIND1    SENSOR_3
#define SS_OBJECT_DETECTION1 SENSOR_4

// -- CONVAYER No.2
#define CTR_ENABLE2     25 //1

#if revert2
    #define CTR_PWM_H2      33 // -
    #define CTR_PWM_L2      32 // -
#else
    #define CTR_PWM_H2      32 // - origin
    #define CTR_PWM_L2      33 // -
#endif

#define SS_LIMMIT_AHEAD2     SENSOR_5
#define SS_LIMMIT_BEHIND2    SENSOR_6
#define SS_OBJECT_DETECTION2 SENSOR_7

#endif
