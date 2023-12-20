#include <Arduino.h>
#include "CAN_manager.h"
#include "Main_controller.h"


void create_task();

TaskHandle_t Task_Receive_CAN; // Task_Receive_CAN
CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
Main_controller* MainCtrl = new Main_controller(MainCAN);

void CanHandle_Receive( void* pvParameters ){
    for(;;){
        MainCtrl->CANReceiveHandle();
        vTaskDelay(2);
    }
}

void setup() {
    MainCtrl->init_main();
    delay(500);
    // -
    MainCAN->CAN_prepare();
    delay(1000);
    create_task();
    MainCtrl->saveTime_checkShutdown = millis();
    // -
    MainCtrl->saveTime_checkLowVoltage = millis();

    MainCtrl->setCoefficient_voltage(0.653, 340.74);
    MainCtrl->setCoefficient_current(1.315, -4060);

}

void loop() {
    MainCtrl->loopMainCtr();
}

void create_task() {   
    xTaskCreatePinnedToCore(
                CanHandle_Receive, /* Task function. */
                "Task_Receive_CAN",    /* name of task. */
                10000,        /* Stack size of task */
                NULL,         /* parameter of the task */
                1,            /* priority of the task */
                &Task_Receive_CAN,     /* Task handle to keep track of created task */
                1);           /* pin task to core 0 */              
}
