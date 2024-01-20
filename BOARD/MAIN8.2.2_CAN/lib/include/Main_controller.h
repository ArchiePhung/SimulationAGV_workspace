#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include "hardwareconfig.h"
#include "controll_config.h"
#include "CAN_manager.h"
#include <WiFi.h>  

struct controllBoard_OC{
    uint8_t address;
    uint8_t command = DO_NOTHING;
    uint8_t status = have_sent;
    bool resetError = false;

    controllBoard_OC(int adr) {address = adr;}
    ~controllBoard_OC();
};

struct statusBoard_OC{ // - OC
    uint8_t CAN_address;
    uint8_t command = DO_NOTHING;
    uint8_t commandStatus = COMMAND_DONE;
    uint8_t sensorBit_status = 0;
    uint8_t error = ISOK;

    statusBoard_OC(uint8_t adr) {CAN_address = adr;}
    ~statusBoard_OC(){}
};

struct controllBoard_HC{
    uint8_t address;
    uint8_t RGB1 = 0;
    uint8_t RGB2 = 0;
    uint8_t sickSelectFeild = 0;

    controllBoard_HC(int adr) {address = adr;}
    ~controllBoard_HC();
};

struct statusBoard_HC{ // - HC
    uint8_t CAN_address;
    uint8_t status = 0;
    uint8_t zone_sick_ahead = 0;
    uint8_t zone_sick_behind = 0;
    uint8_t vacham = 0;

    statusBoard_HC(uint8_t adr) {CAN_address = adr;}
    ~statusBoard_HC(){}
};


struct MainControl{ // Main
    bool sound_enb = 0;
    uint8_t sound_type = 0;
    bool charge_write = 0;
    bool EMG_reset = 0;
    bool EMG_write = 0;
    ~MainControl(){}
};

struct MainStatus {
    uint32_t voltage = 250;
    uint32_t voltage_analog = 0;
    uint32_t current = 0;
    uint32_t current_analog = 0;
    bool EMG_status = 0;
    bool stsButton_reset = 0;
    bool stsButton_power = 0;
    bool CAN_status = 0;
};

class Main_controller
{
    private:
        
        CAN_manager* CAN_main = NULL;

        bool enable_debug = 0;
        // -
        unsigned long saveTime_checkHC = 0;
        unsigned long saveTime_checkOC = 0;
        unsigned long saveTime_checkCAN = 0;
        unsigned long saveTime_checkReset = 0;

        float av_coefficient = 0;
        float bv_coefficient = 0;
        float ac_coefficient = 0;
        float bc_coefficient = 0;

        // - for reset EMG
        int flag_reseting = 0;
        unsigned long saveTime_checkReseted = 0;
        unsigned long saveTime_sendCAN_OC = 0;
        // -
        uint16_t volRead_time0;
        uint16_t volRead_time1;
        uint16_t volRead_time2;
        uint16_t volRead_time3;
        // --
        bool switch_sendCAN = 0;
    public:
        Main_controller(CAN_manager*);
        ~Main_controller();
        
        MainStatus *main_status = new MainStatus;
        MainControl* main_control = new MainControl;

        controllBoard_OC* OC_comd = new controllBoard_OC(ID_OC);
        statusBoard_OC* OC_status = new statusBoard_OC(ID_OC);

        statusBoard_HC* HC_status = new statusBoard_HC(ID_HC);
        controllBoard_HC* HC_comd = new controllBoard_HC(ID_HC);

        unsigned long saveTime_checkShutdown = 0;
        unsigned long saveTime_checkLowVoltage = 0;
        bool stsSend_CAN = 0;
        bool statusBitSensor[8];

        int CANReceiveHandle();
        bool CANTransmitHandle();

        void setCommandFromController(uint8_t);
        void setNewCommand(controllBoard_OC*, uint8_t);

        statusBoard_OC getStatus_OC(uint8_t ID_board) {
            statusBoard_OC noValueHere(99);
            if(ID_board == ID_OC){
                return *OC_status;
            }else 
                return noValueHere;
        }

        MainStatus  getMainStatus(){
            return *main_status;
        }

        void set_enbDebug(bool enb){
            enable_debug = enb;
        }

        void setCoefficient_voltage(float av, float bv){
            av_coefficient = av;
            bv_coefficient = bv;
        }

        void setCoefficient_current(float ac, float bc){
            ac_coefficient = ac;
            bc_coefficient = bc;
        }

        void EMG_detect();

        void soundControl(uint8_t, uint8_t);

        void command_OC();

        void init_main();
        void loopMainCtr();
        void readButton_Shutdown();
        void resetEMG();
        void shutdown_voltageLow();

        void read_voltage();
        void read_currentCharge();
        void warning_battery();
        
        void SetEMG(bool tog){ // true = enable
            if (tog) {
                digitalWrite(PIN_EMC_WRITE, LOW);
            }
            else {
                digitalWrite(PIN_EMC_WRITE, HIGH);
            }
        }
        bool getBit_fromInt(int, int);

        uint16_t filter_analogVoltages(uint16_t vol_in){
            uint16_t vol_out;
            volRead_time3 = volRead_time2;
            volRead_time2 = volRead_time1;
            volRead_time1 = volRead_time0;
            volRead_time0 = vol_in;
            vol_out = volRead_time0 + volRead_time1 + volRead_time2 + volRead_time3;

            return vol_out/4.;
        }
};
#endif