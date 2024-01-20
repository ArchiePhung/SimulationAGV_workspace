#include "Main_controller.h"

Main_controller::Main_controller(CAN_manager* Main)
{
    CAN_main = Main;
}

Main_controller::~Main_controller()
{
    delete OC_comd;
    delete OC_status;
    delete main_status;
}

bool Main_controller::CANTransmitHandle() {
    if (switch_sendCAN == 1){
        CAN_main->SetByteTransmit(HC_comd->address, ID_require);
        CAN_main->SetByteTransmit(HC_comd->RGB1, 1);
        CAN_main->SetByteTransmit(HC_comd->RGB2, 2);
        CAN_main->SetByteTransmit(HC_comd->sickSelectFeild, 3);
        switch_sendCAN = 0;
    }else{
        CAN_main->SetByteTransmit(OC_comd->address, ID_require);
        CAN_main->SetByteTransmit(OC_comd->command, CommandRequire);
        CAN_main->SetByteTransmit(OC_comd->resetError, CommandReset);
        switch_sendCAN = 1;
    }

    if (CAN_main->CAN_Send()) {
        return 1;
    }else{
        return 0;
    }
}

int Main_controller::CANReceiveHandle() {
    uint32_t addRevFromCanBus;
    uint32_t timeOut = 0;
    uint8_t bit_sensor = 0;
    // -------
    addRevFromCanBus = CAN_main->CAN_ReceiveFrom();
    if(addRevFromCanBus != 0x55) // 0x55 | 85
    {
        saveTime_checkCAN = millis();
    }

    if ((millis() - saveTime_checkCAN) > 2000 )
    {   
        if (enable_debug == 1){
        }
        return -3;
    }

    // -------
    if (addRevFromCanBus == ID_OC){
        OC_status->command          = CAN_main->GetByteReceived(FeedbackCommand);

        OC_status->commandStatus    = CAN_main->GetByteReceived(CommandStatus);
        OC_status->sensorBit_status = CAN_main->GetByteReceived(sensorsData);
        OC_status->error            = CAN_main->GetByteReceived(error_check);
        
        if (enable_debug == 1){
        }
        saveTime_checkOC = millis();
    }

    // -------
    if (addRevFromCanBus == ID_HC){
        HC_status->status          = CAN_main->GetByteReceived(7);
        HC_status->zone_sick_ahead    = CAN_main->GetByteReceived(4);
        HC_status->zone_sick_behind = CAN_main->GetByteReceived(5);
        HC_status->vacham            = CAN_main->GetByteReceived(6);
        
        if (enable_debug == 1){
        }
        saveTime_checkHC = millis();
    }

    // --
    if ((millis() - saveTime_checkOC) > 2000 )
    {   
        if (enable_debug == 1){
        }
        return -1;
    }

    if ((millis() - saveTime_checkHC) > 320 )
    {   
        if (enable_debug == 1){
        }
        return -2;
    }

    return 1;
}

void Main_controller::setNewCommand(controllBoard_OC* slave, uint8_t id_comd){
    slave->command = id_comd;
    slave->status = sendAccepted;
}

void Main_controller::soundControl(uint8_t enb, uint8_t comd){
    //  debugSerial.println((String) "[SND] "+ comd);
    if (enb == 0){
        digitalWrite(PIN_SOUND_SP, HIGH);
        digitalWrite(PIN_SOUND_1, HIGH);
        digitalWrite(PIN_SOUND_2, HIGH);
        digitalWrite(PIN_SOUND_3, HIGH);
        digitalWrite(PIN_SOUND_4, HIGH);
    }else{
        digitalWrite(PIN_SOUND_SP , LOW);
        switch(comd){
            case commands::SOUND_OE_OE:
                digitalWrite(PIN_SOUND_1,HIGH);
                digitalWrite(PIN_SOUND_2,LOW);
                digitalWrite(PIN_SOUND_3,HIGH);
                digitalWrite(PIN_SOUND_4,HIGH);
                break;
            case commands::SOUND_FUR_ELISE:
                digitalWrite(PIN_SOUND_4,LOW);
                digitalWrite(PIN_SOUND_2,HIGH);
                digitalWrite(PIN_SOUND_3,HIGH);
                digitalWrite(PIN_SOUND_1,HIGH);
                break;
            case commands::SOUND_3: /* Ambulance */
                digitalWrite(PIN_SOUND_4,HIGH);
                digitalWrite(PIN_SOUND_2,HIGH);
                digitalWrite(PIN_SOUND_3,HIGH);
                digitalWrite(PIN_SOUND_1,LOW);
                break;
            case commands::SOUND_EMC:
                digitalWrite(PIN_SOUND_1,HIGH);
                digitalWrite(PIN_SOUND_2,HIGH);
                digitalWrite(PIN_SOUND_3,LOW);
                digitalWrite(PIN_SOUND_4,HIGH);
                break;
            default:
                digitalWrite(PIN_SOUND_1,HIGH);
                digitalWrite(PIN_SOUND_2,HIGH);
                digitalWrite(PIN_SOUND_3,HIGH);
                digitalWrite(PIN_SOUND_4,HIGH);
                break;
        }
    }
}

void Main_controller::init_main(){
    WiFi.mode(WIFI_OFF);
    btStop();
    //-------------------------------------------- OUT ------------------ 
    pinMode(PIN_WRITE_POWER, OUTPUT);
    digitalWrite(PIN_WRITE_POWER, LOW); 

    pinMode(PIN_EMC_READ, INPUT_PULLUP);
    pinMode(PIN_EMC_WRITE, OUTPUT);

    digitalWrite(PIN_EMC_WRITE, HIGH);

    pinMode(PIN_EMC_RESET, OUTPUT);
    digitalWrite(PIN_EMC_RESET, LOW);

    pinMode(PIN_ADC_VOLTAGE, INPUT);
    pinMode(PIN_ADC_CURRENT, INPUT);

    pinMode(PIN_SOUND_SP, OUTPUT);
    digitalWrite(PIN_SOUND_SP, LOW);

    pinMode(PIN_SOUND_1, OUTPUT);
    pinMode(PIN_SOUND_2, OUTPUT);
    pinMode(PIN_SOUND_3, OUTPUT);
    pinMode(PIN_SOUND_4, OUTPUT);

    digitalWrite(PIN_SOUND_1, HIGH);
    digitalWrite(PIN_SOUND_2, HIGH);
    digitalWrite(PIN_SOUND_3, HIGH);
    digitalWrite(PIN_SOUND_4, HIGH);

    pinMode(PIN_READ_PC, INPUT);
    pinMode(PIN_CHARGE, OUTPUT);

    pinMode(ENABLE_5v, OUTPUT);
    pinMode(ENABLE_22v, OUTPUT);

    pinMode(BUTTON_SHUTDOWN, INPUT_PULLUP);
    pinMode(BUTTON_EMC_RES, INPUT_PULLUP);

    // -- Giu nguon
    digitalWrite(ENABLE_5v, LOW);
    digitalWrite(ENABLE_22v, LOW);

    pinMode(PIN_L1, OUTPUT);
    pinMode(PIN_L2, OUTPUT);
    digitalWrite(PIN_L1, HIGH);
    digitalWrite(PIN_L2, HIGH);

    main_status->voltage = 2500;
}

void Main_controller::warning_battery(){
    if ((main_status->voltage/100.) < 23.){
        digitalWrite(PIN_L1, LOW); // - Red LED
        digitalWrite(PIN_L2, HIGH); // - Blue LED
    }else{
        digitalWrite(PIN_L1, HIGH); // - Red LED
        digitalWrite(PIN_L2, LOW); // - Blue LED
    }
}

void Main_controller::EMG_detect(){
    if (digitalRead(PIN_EMC_READ) == LOW) {
        // SetEMG(true);
        main_status->EMG_status = 1;
    }else{
        main_status->EMG_status = 0;
    }
}

void Main_controller::command_OC() {
    if(OC_comd->status != have_sent) {
        if(OC_status->commandStatus == IN_PROCESSING) {OC_comd->status = pending;}
        else {OC_comd->status = sendAccepted;}
    }
}

void Main_controller::read_voltage(){
    // - read voltage pin 
    uint16_t voltage_analog;
    uint16_t voltage_analog_filter;
    voltage_analog = analogRead(PIN_ADC_VOLTAGE);
    voltage_analog_filter = filter_analogVoltages(voltage_analog);

    main_status->voltage_analog = voltage_analog_filter;
    // - read voltage
    if (voltage_analog_filter == 0) {
        main_status->voltage = 0;
    }
    else{
        main_status->voltage = av_coefficient*voltage_analog_filter + bv_coefficient;
    }
}

void Main_controller::read_currentCharge(){
    // - read current pin 
    uint16_t current_analog;
    int current;
    current_analog = analogRead(PIN_ADC_CURRENT);
    main_status->current_analog = current_analog;
    // - read voltage
    if (current_analog == 0) {
        main_status->current = 0;
    }else{
        current = ac_coefficient*current_analog + bc_coefficient;
        if (current < 0){
            main_status->current = 0;
        }else{
            main_status->current = current;
        }
    }
}

void Main_controller::readButton_Shutdown(){
    // B2 in board
    if (digitalRead(BUTTON_SHUTDOWN) == 0){
        saveTime_checkShutdown = millis();
        main_status->stsButton_power = 1;
    }else{
        main_status->stsButton_power = 0;
    }

    if (millis() - saveTime_checkShutdown > 6000){
        digitalWrite(ENABLE_5v, HIGH);
        digitalWrite(ENABLE_22v, HIGH);
    }
}

void Main_controller::resetEMG(){
    bool sts, sts1;
    // - B1 in board || control EMG reset
    sts = digitalRead(BUTTON_EMC_RES);

    if (sts == 1){ // - Do tin hieu bi nguoc.
        sts1 = 1;
    }else{
        sts1 = 0;
    }

    if (sts1 == false && main_control->EMG_reset == false){
        saveTime_checkReset = millis();
    }

    if (sts1 == false){ // false
        main_status->stsButton_reset = false;
    }else{
        main_status->stsButton_reset = true;
    }
    
    if (flag_reseting == 0){
        if (millis() - saveTime_checkReset > 250){
            SetEMG(false);
            digitalWrite(PIN_EMC_RESET, HIGH);
            flag_reseting = 1;
            saveTime_checkReseted = millis();
        }
    }else if (flag_reseting == 1){
        if (millis() - saveTime_checkReseted > 150){
            digitalWrite(PIN_EMC_RESET, LOW);
            flag_reseting = 2;
            saveTime_checkReset = millis();
        }
    }else if (flag_reseting == 2){
        if (sts1 == false && main_control->EMG_reset == false){
            flag_reseting = 0;
            saveTime_checkReset = millis();
        }
    }
}

void Main_controller::shutdown_voltageLow(){
    if ((main_status->voltage/100.) > 21.){
    // if (22 > 21.){
        saveTime_checkLowVoltage = millis();
    }

    if ((millis() - saveTime_checkLowVoltage) > 20000){
        SetEMG(true);
        digitalWrite(ENABLE_5v  , HIGH);
        digitalWrite(ENABLE_22v , HIGH);
    }
}

void Main_controller::loopMainCtr(){
    // - send CAN
    // if ( (millis() - saveTime_sendCAN_OC) > 100){
    //     saveTime_sendCAN_OC = millis();
    //    main_status->CAN_status = CANTransmitHandle();
    // }

    // - add: 10/08/2022
    warning_battery();

    // - EMG
    EMG_detect();

    // - Voltage
    read_voltage();

    // - Current charge
    read_currentCharge();

    // - Read button shutdown
    readButton_Shutdown();

    // - Reset EMG - Button reset - Command reset
    resetEMG();

    // - Check low voltages
    shutdown_voltageLow();

    // --------------------------------------
    // - Speaker
    soundControl(main_control->sound_enb, main_control->sound_type);

    // - Charge
    if (main_control->charge_write == true){
        // - charge ON
        digitalWrite(PIN_CHARGE , HIGH);
    }else{
        // - charge OFF
        digitalWrite(PIN_CHARGE , LOW);
    }
    
    // - EMG write
    if (main_control->EMG_write == true){
        SetEMG(true);
    }
    else{
        SetEMG(false);
    }
    
}

bool Main_controller::getBit_fromInt(int val, int pos){
    bool bit;
    int val_now = val;
    for (int i = 0; i < 8; i++){
        bit = val_now%2;
        val_now /= 2;

        if (i == pos){
            return bit;
        }

        if (val_now < 1)
            return 0;
    }
    return 0;
}
