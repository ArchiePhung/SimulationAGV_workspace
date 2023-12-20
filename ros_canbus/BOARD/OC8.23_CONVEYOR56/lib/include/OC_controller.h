#ifndef OC_CONTROLLER_H
#define OC_CONTROLLER_H

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

class OC_controller
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
        int FREQUENCY_sendCAN = 2;
        unsigned long preTime_sendCAN = 0;
        uint8_t statusRev_CAN = 0;
        unsigned long saveTime_checkCAN_Rev = 0;
      
        // ----------------------------------------------
        int timeCheckErrorRun = 40000; // ms - Do thoi gian loi.
        int timeCheckSensor = 150; // ms - Do thoi gian cam bien tac dong.
        
        // ------------
    	int channelPWM_H1 = 0;
    	int channelPWM_L1 = 4;
        // -
    	int step_transmitRack1 = 0;	 // Lưu bước nâng bàn nâng.
    	int step_receivedRack1 = 0;	 // Lưu bước hạ bàn nâng.
    	int speed_transmitRack1 = 0; // Lưu tốc độ nâng bàn nâng.	
    	int speed_receivedRack1 = 0; // Lưu tốc độ nâng hạ nâng.

        // ------------
    	int channelPWM_H2 = 8;
    	int channelPWM_L2 = 12;
        // --
    	int step_transmitRack2 = 0;	 // Lưu bước nâng bàn nâng.
    	int step_receivedRack2 = 0;	 // Lưu bước hạ bàn nâng.
    	int speed_transmitRack2 = 0; // Lưu tốc độ nâng bàn nâng.	
    	int speed_receivedRack2 = 0; // Lưu tốc độ nâng hạ nâng.

        // -- --
        unsigned long timeStart_sensorAhead1;   // Lưu thời gian tác động cảm biến giới hạn trước.
        unsigned long timeStart_sensorBehind1;  // Lưu thời gian tác động cảm biến giới hạn sau.
        unsigned long timeStart_sensorRack1;    // Lưu thời gian tác động cảm biến phát hiện thùng hàng.

    	unsigned long timeStart_transmitRack1;	// Lưu thời gian bắt đầu hạ.
        unsigned long timeStart_receivedRack1;    // Lưu thời gian tác động cảm biến phát hiện thùng hàng.
        
        // -
        unsigned long timeStart_sensorAhead2;   // Lưu thời gian tác động cảm biến giới hạn trước.
        unsigned long timeStart_sensorBehind2;  // Lưu thời gian tác động cảm biến giới hạn sau.
        unsigned long timeStart_sensorRack2;    // Lưu thời gian tác động cảm biến phát hiện thùng hàng.

    	unsigned long timeStart_transmitRack2;  // Lưu thời gian bắt đầu nâng.
    	unsigned long timeStart_receivedRack2;	// Lưu thời gian bắt đầu hạ.
        // --------------------------------------

    public:
        OC_controller(CAN_manager*);
        ~OC_controller();

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

        bool readSensorAhead1(bool statusRight);
        bool readSensorBehind1(bool statusRight);
        bool readSensorRack1(bool statusRight);

        bool readSensorAhead2(bool statusRight);
        bool readSensorBehind2(bool statusRight);
        bool readSensorRack2(bool statusRight);

        void transmitRack1(int percentSpeed);
        void receivedRack1(int percentSpeed);

        void transmitRack2(int percentSpeed);
        void receivedRack2(int percentSpeed);

        void stopAndReset1();
        void stopAndReset2();
        // --     
        void OC_CAN_Transmit();
        void OC_CAN_Receive();
        void OC_CAN_send();

        void tryRun_conveyor1(int percentSpeed);
        void tryRun_conveyor2(int percentSpeed);
        void tryRun();
        
        void forward_rotation1(int percentSpeed);
        void forward_rotation2(int percentSpeed);

        void reverse_rotation1(int percentSpeed);
        void reverse_rotation2(int percentSpeed);

        void OC_loop();
        void debug();
};


#endif
