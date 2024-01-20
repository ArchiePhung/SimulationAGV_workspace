#include <Arduino.h>
#include "CAN_manager.h"
#include "Main_controller.h"

#include "ros.h"
#include "ros/time.h"
#include "sti_msgs/POWER_info.h"
#include "sti_msgs/POWER_request.h"

#include "sti_msgs/Lift_status.h"
#include "sti_msgs/Lift_control.h"

#include "sti_msgs/HC_info.h"
#include "sti_msgs/HC_request.h"

#define FREQUENCY_PUB_POWER_INFO 4
#define FREQUENCY_PUB_OC_INFO    4
#define FREQUENCY_PUB_HC_INFO    24
#define ENB_DEBUG 1

void create_task();
void debuge();
void POWER_controlCallback(const sti_msgs::POWER_request &data);
void OC_controlCallback(const sti_msgs::Lift_control &data);
void HC_Callback(const sti_msgs::HC_request &data);
void HC_fieldCallback(const std_msgs::Int8 &data);

ros::NodeHandle nodeHandle;
sti_msgs::POWER_info power_info;
sti_msgs::POWER_request power_request;

sti_msgs::Lift_status lift_status;
sti_msgs::Lift_control lift_control;

sti_msgs::HC_info hc_info;
sti_msgs::HC_request hc_request;   
std_msgs::Int8 hc_fieldRequest;

ros::Publisher power_pub("POWER_info", &power_info);         	
ros::Subscriber<sti_msgs::POWER_request> power_sub("POWER_request", POWER_controlCallback);

ros::Publisher lift_pub("lift_status", &lift_status);
ros::Subscriber<sti_msgs::Lift_control> lift_sub("lift_control", OC_controlCallback);

ros::Publisher hc_pub("HC_info", &hc_info);       	
ros::Subscriber<sti_msgs::HC_request> hc_sub("HC_request", HC_Callback);
ros::Subscriber<std_msgs::Int8> hc_subSelectField("HC_fieldRequest", HC_fieldCallback);

unsigned long saveTime_pub_main = 0;
unsigned long saveTime_pub_oc = 0;
unsigned long saveTime_pub_hc = 0;
unsigned long debugTime = 0;

int sts_OC;
int sts_HC;
int sts_CAN_receive;

TaskHandle_t Task_Receive_CAN; // Task_Receive_CAN
CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
Main_controller* MainCtrl = new Main_controller(MainCAN);

void POWER_controlCallback(const sti_msgs::POWER_request &data)
{
    power_request = data; 
} 

void OC_controlCallback(const sti_msgs::Lift_control &data)
{
    lift_control = data;
} 

void CanHandle_Receive( void* pvParameters ){
    for(;;){
        sts_CAN_receive = MainCtrl->CANReceiveHandle();
        vTaskDelay(2);
    }
}

void HC_Callback(const sti_msgs::HC_request &data)
{
    hc_request = data; 
}

void HC_fieldCallback(const std_msgs::Int8 &data)
{
    hc_fieldRequest = data;
}

void setup() {
    MainCtrl->set_enbDebug(ENB_DEBUG);
    MainCtrl->init_main();
    delay(500);
    // - ROS
    nodeHandle.initNode();
    nodeHandle.getHardware()->setBaud(57600); 
    delay(10);
    nodeHandle.advertise(power_pub);
    nodeHandle.subscribe(power_sub);
    delay(10);
    nodeHandle.advertise(lift_pub);
	nodeHandle.subscribe(lift_sub);
    delay(10);
    nodeHandle.advertise(hc_pub);
    nodeHandle.subscribe(hc_sub);
    nodeHandle.subscribe(hc_subSelectField);
    delay(10);
    // -
    MainCAN->CAN_prepare();
    delay(6000);

    create_task();

    // MainCtrl->setCoefficient_voltage(0.726, 193.568); // 0.620376, 496.7085  1.031209, 286.21583
    // MainCtrl->setCoefficient_current(2.1, -6022.38805); // 0.666666, -1933.33

    // -- 29/06/2022
    MainCtrl->setCoefficient_voltage(0.6421, 452.5988); // 0.633798, 425.328
    MainCtrl->setCoefficient_current(3.7037, -10911.1113); // 0.666666, -1933.33

    MainCtrl->saveTime_checkShutdown = millis();
    // - ROS
    while (!nodeHandle.connected())
    {   
        MainCtrl->readButton_Shutdown();
        MainCtrl->resetEMG();
		MainCtrl->read_voltage();
		MainCtrl->shutdown_voltageLow();
        MainCtrl->warning_battery();
        delay(10);
        nodeHandle.spinOnce();
    }
    // -
    MainCtrl->saveTime_checkLowVoltage = millis();
}

void loop() {
    // - status main
    power_info.voltages.data = (MainCtrl->main_status->voltage)/100.;
    power_info.voltages_analog.data = MainCtrl->main_status->voltage_analog;
    power_info.charge_current.data = (MainCtrl->main_status->current)/100.;
    power_info.charge_analog.data = MainCtrl->main_status->current_analog;
    power_info.stsButton_reset.data = MainCtrl->main_status->stsButton_reset;
    power_info.stsButton_power.data = MainCtrl->main_status->stsButton_power;
    power_info.EMC_status.data = MainCtrl->main_status->EMG_status;
    power_info.CAN_status.data = MainCtrl->main_status->CAN_status;
    // - control main
    MainCtrl->main_control->sound_enb = power_request.sound_on.data;
    MainCtrl->main_control->sound_type = power_request.sound_type.data;
    MainCtrl->main_control->charge_write = power_request.charge.data;
    MainCtrl->main_control->EMG_write = power_request.EMC_write.data;
    MainCtrl->main_control->EMG_reset = power_request.EMC_reset.data;
    // - 
    MainCtrl->loopMainCtr();

    // -- OC
    // if (sts_CAN_receive == -1 || sts_CAN_receive == -3){ // - LOST OC
    //     sts_OC = -2;
    // }else{ // - OK
    //     if (MainCtrl->OC_status->error == 1) // - ERROR OC
    //         sts_OC = -1;
    //     else
    //         sts_OC = MainCtrl->OC_status->commandStatus;
    // }

    // lift_status.status.data = sts_OC;

    // lift_status.sensorLift.data = MainCtrl->getBit_fromInt(MainCtrl->OC_status->sensorBit_status, BIT_SENSOR_Lift);
    // lift_status.sensorUp.data = MainCtrl->getBit_fromInt(MainCtrl->OC_status->sensorBit_status, BIT_SENSOR_UP);
    // lift_status.sensorDown.data = MainCtrl->getBit_fromInt(MainCtrl->OC_status->sensorBit_status, BIT_SENSOR_DOWN);
    // // -- 
    // MainCtrl->OC_comd->command = lift_control.control.data;
    // MainCtrl->OC_comd->resetError = lift_control.reset.data;

    // // -- -- -- -- HC
    // if (sts_CAN_receive == -2 || sts_CAN_receive == -3){ // - LOST CAN HC
    //     sts_HC = -1;
    // }else{ // - OK
    //     sts_HC = 0;
    // }
    // hc_info.status.data = sts_HC;
    // hc_info.zone_sick_ahead.data = MainCtrl->HC_status->zone_sick_ahead;
    // hc_info.zone_sick_behind.data = MainCtrl->HC_status->zone_sick_behind;
    // hc_info.vacham.data = MainCtrl->HC_status->vacham;
    // // --
    // MainCtrl->HC_comd->RGB1 = hc_request.RBG1.data;
    // MainCtrl->HC_comd->RGB2 = hc_request.RBG2.data;
    // MainCtrl->HC_comd->sickSelectFeild = hc_fieldRequest.data;
    // - ROS
    if ((millis() - saveTime_pub_main) > (1000 / FREQUENCY_PUB_POWER_INFO))
    {   
        saveTime_pub_main = millis();
        power_pub.publish(&power_info); 
    }

    // if ((millis() - saveTime_pub_oc) > (1000 / FREQUENCY_PUB_OC_INFO))
    // {   
    //     saveTime_pub_oc = millis();
    //     lift_pub.publish(&lift_status);
    // }

    // if ((millis() - saveTime_pub_hc) > (1000 / FREQUENCY_PUB_HC_INFO))
    // {   
    //     saveTime_pub_hc = millis();
    //     hc_pub.publish(&hc_info);
    // }

	nodeHandle.spinOnce();
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
