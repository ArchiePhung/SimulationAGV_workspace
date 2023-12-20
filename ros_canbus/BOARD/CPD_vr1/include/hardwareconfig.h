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

#define ID_CPD  0x07
#define ID_RTC  0x01

#define CAN_BAUD_SPEED  CAN_SPEED_125KBPS
#define CAN_TX          GPIO_NUM_5
#define CAN_RX          GPIO_NUM_4
#define CAN_FRAME       CAN_frame_std
#define CAN_ID          ID_CPD
#define CAN_SEND_SIZE   8
#define CAN_TIMER       60
#define TIME_OUT        50000

// --
#define ADDRESS_PCF8575 0x20 // 0x20
// -----------
#define ONBOARD_INPUT1   33 //
#define ONBOARD_INPUT2   32 //
#define ONBOARD_INPUT3   35 //
#define ONBOARD_INPUT4   34 //

#define ONBOARD_OUTPUT1   18 //
#define ONBOARD_OUTPUT2   19 //
#define ONBOARD_OUTPUT3   23 //
#define ONBOARD_OUTPUT4   25 //

// ---------
#define ONBOARD_LED1   13 //
#define ONBOARD_LED2   26 //
#define ONBOARD_LED3   27 //
#define ONBOARD_LED4   14 //

#define LED_CAN_SEND  27 //
#define LED_CAN_REC   26 // 26
#define LED_ROS_SEND  13 //
#define LED_ROS_REC   14 //
// ---------

#define PCF_INPUT5   P15 //
#define PCF_INPUT6   P14 //
#define PCF_INPUT7   P13 //
#define PCF_INPUT8   P12 //
#define PCF_INPUT9   P11 //
#define PCF_INPUT10  P10 //
#define PCF_INPUT11  P9 //
#define PCF_INPUT12  P8 //

#define PCF_OUTPUT5   P3// 3 //
#define PCF_OUTPUT6   P2// 2 //
#define PCF_OUTPUT7   P1 // 1 //
#define PCF_OUTPUT8   P0 // 0 //
#define PCF_OUTPUT9   P7 // 7 //
#define PCF_OUTPUT10  P6 // 6 // 10
#define PCF_OUTPUT11  P5 // 5 //
#define PCF_OUTPUT12  P4 // 4 //

#endif