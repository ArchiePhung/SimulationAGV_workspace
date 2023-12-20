#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "CAN_manager.h"
#include "CPD_controller.h"
#include "WiFi.h"
#include "ros.h"
#include "ros/time.h"

TaskHandle_t Task0;

CAN_manager* CPD_CAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
CPD_controller* CPD_Ctrl = new CPD_controller(CPD_CAN);

void Task0_code( void * pvParameters ){
  for(;;){
   // CPD_Ctrl->CAN_receive();
   // vTaskDelay(2);
    
   CPD_Ctrl->CAN_send();
   vTaskDelay(2);

  }
}

void create_task() {
  xTaskCreatePinnedToCore(
                    Task0_code,   /* Task function. */
                    "Task0",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task0,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */
}

void setup() {
  WiFi.mode(WIFI_OFF);
  delay(100);
  CPD_CAN->CAN_prepare();
  delay(50);
  CPD_Ctrl->setupBegin();
  btStop();
  CPD_Ctrl->led_start();
  // delay(60);
  create_task();
  delay(100);
}

void loop() {
  CPD_Ctrl->CAN_receive();
  
  // CPD_Ctrl->CAN_send();
  
  CPD_Ctrl->loop_run();
    
   // vTaskDelay(10);
  // CPD_Ctrl->debug();
  // CPD_Ctrl->loop_run();
}
