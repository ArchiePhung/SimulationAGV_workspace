#ifndef CPD_CONTROLLER_H
#define CPD_CONTROLLER_H

#include "CAN_manager.h"
#include "hardwareconfig.h"
#include "controll_config.h"


struct CAN_SEND{ // - OC
    uint8_t status1 = 0;                    // - Byte_0
    uint8_t status2 = 0;                    // - Byte_1
    uint8_t statusSensor_limitAhead1 = 0;   // - Byte_2
    uint8_t statusSensor_limitBehind1 = 0;  // - Byte_3
    uint8_t statusSensor_checkRack1 = 0;    // - Byte_4
    uint8_t statusSensor_limitAhead2 = 0;   // - Byte_5
    uint8_t statusSensor_limitBehind2 = 0;  // - Byte_6
    uint8_t statusSensor_checkRack2 = 0;    // - Byte_7

    CAN_SEND() {}
    ~CAN_SEND(){}
};

struct CAN_RECEIVED{ // - OC
    uint8_t mission1 = 0;
    uint8_t speed1 = 0;
    uint8_t mission2 = 0;
    uint8_t speed2 = 0;

    CAN_RECEIVED() {}
    ~CAN_RECEIVED(){}
};

class CPD_controller
{
    private:
        unsigned long time_gui;
        /**
        * @brief          : luu gia tri nhan/gui cua mang giao tiep CAN
        * @details        : can_id: id can cua mach
                            cmd,cmd_status: function va trang thai (0:lo lung || 1:o tren || 2:o duoi)
                            status_r: hoan thanh lenh cmd
                            err_lx: bao loi ban nang
        * @note           : 
        */
        unsigned char   cmd_request = commands::DO_NOTHING,
                        cmd_resetError = 0,
                        cmd_status = processStatus::COMMAND_DONE,
                        pre_cmd_request = commands::DO_NOTHING,
                        err_lx = ISOK;        
        CAN_manager* CANctr = NULL;

        // -
        int FREQUENCY_sendCAN = 4;
        unsigned long preTime_sendCAN = 0;
        uint8_t statusRev_CAN = 0;
        unsigned long saveTime_checkCAN_Rev = 0;
      
        // ----------------------------------------------
        unsigned long timeSave_LED_CANreceived; // Lưu thời gian
        unsigned long timeSave_LED_CANsend;     // Lưu thời gian
        unsigned long timeSave_writePCF;     // Lưu thời gian

        // --------------------------------------
        unsigned char valueInput = 0;
        unsigned char valueOutput = 0;

        uint8_t valueReceived_byte0 = 0;
        uint8_t valueReceived_byte1 = 0;

        uint16_t valueWrite = 0;
        bool arrBits_write[16];
        bool pre_arrBits_write[16];

        uint16_t valueTransmit_byte0 = 0;
        uint16_t valueTransmit_byte1 = 0;
        bool status_out = 1;

    public:
        CPD_controller(CAN_manager*);
        ~CPD_controller();

        CAN_SEND* CAN_sendData = new CAN_SEND();
        CAN_RECEIVED* CAN_receivedData = new CAN_RECEIVED();
        CAN_RECEIVED* CAN_command = new CAN_RECEIVED();


        void setCommand(unsigned char _cmd) {cmd_request = _cmd;}
        void setCommandStatus(unsigned char _cmd_status) {cmd_status = _cmd_status;}
        void setError(unsigned char _err_lx) {err_lx = _err_lx;}

        unsigned char getError() {return err_lx;}
        unsigned char getCommandStatus() {return cmd_status;}
        unsigned char getCommand() {return cmd_request;}
        bool getSensor(int _pin) {return digitalRead(_pin);}
        
        bool the_three_timer(unsigned int timer_num, unsigned int set_value){
            static unsigned int timer_value[3]={0,0,0};
            int t = millis();
            if (t- timer_value[timer_num] >= set_value){
                timer_value[timer_num] = t;
                return true;
            }
            else{
                return false;
            }
        }
        // -- 
        void setupBegin();

        void readInput();
        void writeOutput();

        void CAN_transmit();
        void CAN_receive();
        void CAN_send();

        void loop_run();
        void led_start();
        void debuge();

        uint8_t setBit_toByte(bool bit0, bool bit1, bool bit2, bool bit3, bool bit4, bool bit5, bool bit6, bool bit7);
        bool getBit_from2Byte(int pos, uint16_t valByte);
        uint8_t int_to2Bytes(uint16_t valueIn, int pos_out);
};


#endif