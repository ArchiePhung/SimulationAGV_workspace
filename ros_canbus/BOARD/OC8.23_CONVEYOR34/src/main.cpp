#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "CAN_manager.h"
#include "OC_controller.h"
#include "WiFi.h"
#include "ros.h"
#include "ros/time.h"

TaskHandle_t Task0;

CAN_manager* OC_CAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
OC_controller* OC_Ctrl = new OC_controller(OC_CAN);

void Task0_code( void * pvParameters ){
  for(;;){
    OC_Ctrl->OC_CAN_Receive();
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
  OC_CAN->CAN_prepare();
  delay(50);
  OC_Ctrl->setupBegin();
  btStop();
  delay(50);
  create_task();
  delay(100);
}

void loop() {
  OC_Ctrl->OC_loop();
  // OC_Ctrl->tryRun();

  // OC_Ctrl->debug();
}