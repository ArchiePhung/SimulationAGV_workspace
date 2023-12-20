#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#define receive_all true
#define receive_one false

#include "Arduino.h"
#include "ESP32CAN.h"
#include "CAN_config.h"
#include "hardwareconfig.h"

#define CAN_CHECK_LOST_FREQ 200

#define USING_MAIN_BOARD

#ifdef USING_MOTOR_CONTROLLER_BOARD
    enum CAN_TransmitDataType{
        //ID_send = 0,
        CommandReceived = 0,
        commandStatus = 1,
        velocity = 2,
        error_check = 7
    };

    enum CAN_ReceiveDataType{
        ID_received = 0,
        IncommingCommand = 1,
        velocity = 2,
        SickSignal = 3,
    };
#endif

#ifdef USING_MAIN_BOARD
    enum sendCommandStatus{
        pending = 0,
        have_sent = 1,
        sendAccepted = 3,
    };

    enum CAN_TransmitData{
        to_ID = ID_RTC,

        ID_require = 0,
        CommandRequire = 1,
        CommandReset = 2,
        SpeedRequire = 2,
        SickSignal = 3,
        SelectHead = 2,
        SelectSick = 3,
        cam_led =4
    };
    enum CAN_ReceiveData{
        FeedbackCommand = 0,
        CommandStatus = 1,
        velocity = 2,
        LineTrakingSensor = 3,
        oneBitSignal = 4,
        error_check = 7,

        RFIDstatus4 = 4,
        RFIDstatus3 = 3,
        RFIDstatus2 = 2,
        RFIDstatus1 = 1,
        RFIDstatus0 = 0,
        SICKstatus = 5,

        sensorsData = 2 
    };

#endif

class CAN_manager
{
private:
    CAN_speed_t _baud_speed;
    gpio_num_t _tx;
    gpio_num_t _rx;
    CAN_frame_format_t _frame;
    uint32_t _can_id;
    uint8_t _send_size ;

public:

    CAN_manager(CAN_speed_t ,gpio_num_t ,gpio_num_t , CAN_frame_format_t ,uint32_t , uint8_t );
    void CAN_prepare(void);
    bool CAN_ReceiveFrom(int);
    // uint32_t CAN_ReceiveFrom(void); 
    uint8_t GetByteReceived(unsigned char);


    bool CAN_Send();
    void SetByteTransmit(uint8_t,unsigned char);

    ~CAN_manager();
};

#endif
