#include "CPD_controller.h"
#include "PCF8575.h"

// Set i2c address
// PCF8575 pcf8575(ADDRESS_PCF8575);

// Set i2c address
PCF8575 pcf8575(ADDRESS_PCF8575);

CPD_controller::CPD_controller(CAN_manager* _CANctr)
{
    CANctr = _CANctr;
}

CPD_controller::~CPD_controller()
{
  delete CAN_sendData;
  delete CAN_receivedData;
  delete CAN_command;
}

void CPD_controller::setupBegin() 
{
  Serial.begin(57600);
  pinMode(ONBOARD_INPUT1, INPUT); // INPUT_PULLUP
  pinMode(ONBOARD_INPUT2, INPUT); // 
  pinMode(ONBOARD_INPUT3, INPUT); // 
  pinMode(ONBOARD_INPUT4, INPUT); // 

  pinMode(ONBOARD_OUTPUT1, OUTPUT);
  pinMode(ONBOARD_OUTPUT2, OUTPUT);
  pinMode(ONBOARD_OUTPUT3, OUTPUT);
  pinMode(ONBOARD_OUTPUT4, OUTPUT);

  // ----------
  pinMode(LED_CAN_SEND, OUTPUT);
  pinMode(LED_CAN_REC,  OUTPUT);
  pinMode(LED_ROS_SEND, OUTPUT);
  pinMode(LED_ROS_REC,  OUTPUT);
  // ----------
	// Set pinMode to OUTPUT
	pcf8575.pinMode(PCF_OUTPUT5, OUTPUT);
  pcf8575.pinMode(PCF_OUTPUT6, OUTPUT);
  pcf8575.pinMode(PCF_OUTPUT7, OUTPUT);
  pcf8575.pinMode(PCF_OUTPUT8, OUTPUT);
  pcf8575.pinMode(PCF_OUTPUT9, OUTPUT);
  pcf8575.pinMode(PCF_OUTPUT10, OUTPUT);
  pcf8575.pinMode(PCF_OUTPUT11, OUTPUT);
  pcf8575.pinMode(PCF_OUTPUT12, OUTPUT);

	pcf8575.pinMode(PCF_INPUT5, INPUT);
	pcf8575.pinMode(PCF_INPUT6, INPUT);
	pcf8575.pinMode(PCF_INPUT7, INPUT);
	pcf8575.pinMode(PCF_INPUT8, INPUT);
	pcf8575.pinMode(PCF_INPUT9, INPUT);
	pcf8575.pinMode(PCF_INPUT10, INPUT);
	pcf8575.pinMode(PCF_INPUT11, INPUT);
	pcf8575.pinMode(PCF_INPUT12, INPUT);

  pcf8575.begin();

  digitalWrite(ONBOARD_OUTPUT1, 1);
  digitalWrite(ONBOARD_OUTPUT2, 1);
  digitalWrite(ONBOARD_OUTPUT3, 1);
  digitalWrite(ONBOARD_OUTPUT4, 1);

  // pcf8575.write(PCF_OUTPUT5,    1);
  // pcf8575.write(PCF_OUTPUT6,    1);
  // pcf8575.write(PCF_OUTPUT7,    1);
  // pcf8575.write(PCF_OUTPUT8,    1);
  // pcf8575.write(PCF_OUTPUT9,    1);
  // pcf8575.write(PCF_OUTPUT10,   1);
  // pcf8575.write(PCF_OUTPUT11,   1);
  // pcf8575.write(PCF_OUTPUT12,   1);

}

void CPD_controller::readInput()
{
  uint16_t bitPCF, intSensor;
  bool arrBits[16];
  // - Read
  // bitPCF = pcf8575.read16();
  PCF8575::DigitalInput digi = pcf8575.digitalReadAll();
  intSensor = 0;
  // - 
  arrBits[0] = digitalRead(ONBOARD_INPUT1);
  arrBits[1] = digitalRead(ONBOARD_INPUT2);
  arrBits[2] = digitalRead(ONBOARD_INPUT3);
  arrBits[3] = digitalRead(ONBOARD_INPUT4);
  // arrBits[4] = getBit_from2Byte(PCF_INPUT5, bitPCF);
  // arrBits[5] = getBit_from2Byte(PCF_INPUT6, bitPCF);
  // arrBits[6] = getBit_from2Byte(PCF_INPUT7,  bitPCF);
  // arrBits[7] = getBit_from2Byte(PCF_INPUT8,  bitPCF);
  // arrBits[8] = getBit_from2Byte(PCF_INPUT9,  bitPCF);
  // arrBits[9] = getBit_from2Byte(PCF_INPUT10, bitPCF);
  // arrBits[10] = getBit_from2Byte(PCF_INPUT11, bitPCF);
  // arrBits[11] = getBit_from2Byte(PCF_INPUT12, bitPCF);
  // arrBits[12] = 0;
  // arrBits[13] = 0;
  // arrBits[14] = 0;
  // arrBits[15] = 0;

  arrBits[4]  = digi.p15;
  arrBits[5]  = digi.p14;
  arrBits[6]  = digi.p13;
  arrBits[7]  = digi.p12;
  arrBits[8]  = digi.p11;
  arrBits[9]  = digi.p10;
  arrBits[10] = digi.p9;
  arrBits[11] = digi.p8;
  arrBits[12] = 0;
  arrBits[13] = 0;
  arrBits[14] = 0;
  arrBits[15] = 0;
  
  // -
  for (int i = 0; i < 16; i++){
    intSensor += arrBits[i]*pow(2, i);
  }

  valueTransmit_byte0 = int_to2Bytes(intSensor, 0);
  valueTransmit_byte1 = int_to2Bytes(intSensor, 1);
}

void CPD_controller::writeOutput()
{
  char arrBits[16];
  valueWrite = valueReceived_byte0 + valueReceived_byte1*256;
  // - 
  for (int i = 0; i < 16; i++){
    arrBits_write[i] = getBit_from2Byte(i, valueWrite);
  }

  if ((millis() - timeSave_writePCF) > (200))
  {
    timeSave_writePCF = millis();
    digitalWrite(ONBOARD_OUTPUT1, 1 - arrBits_write[0]);
    digitalWrite(ONBOARD_OUTPUT2, 1 - arrBits_write[1]);
    digitalWrite(ONBOARD_OUTPUT3, 1 - arrBits_write[2]);
    digitalWrite(ONBOARD_OUTPUT4, 1 - arrBits_write[3]);

    pcf8575.digitalWrite(PCF_OUTPUT5,    1 - arrBits_write[4]);
    pcf8575.digitalWrite(PCF_OUTPUT6,    1 - arrBits_write[5]);
    pcf8575.digitalWrite(PCF_OUTPUT7,    1 - arrBits_write[6]);
    pcf8575.digitalWrite(PCF_OUTPUT8,    1 - arrBits_write[7]);
    pcf8575.digitalWrite(PCF_OUTPUT9,    1 - arrBits_write[8]);
    pcf8575.digitalWrite(PCF_OUTPUT10,   1 - arrBits_write[9]);
    pcf8575.digitalWrite(PCF_OUTPUT11,   1 - arrBits_write[10]);
    pcf8575.digitalWrite(PCF_OUTPUT12,   1 - arrBits_write[11]);

  }
}

void CPD_controller::CAN_receive(){
  if (CANctr->CAN_ReceiveFrom(ID_RTC) )
  {
    if (CANctr->GetByteReceived(0) == ID_CPD)
    {
      timeSave_LED_CANreceived = millis();
      saveTime_checkCAN_Rev = millis();
      valueReceived_byte0  = CANctr->GetByteReceived(1);
      valueReceived_byte1 = CANctr->GetByteReceived(2);
      // Serial.printf("Received: Byte1_6 - %d | Byte7_12 - %d\n", valueReceived_byte0, valueReceived_byte1);
    }
  }
  if ((millis() - saveTime_checkCAN_Rev) > 1000)
  {   
    statusRev_CAN = 0;
  }else{
    statusRev_CAN = 1;
  }
}

void CPD_controller::CAN_transmit(){
  // -
  // - 
  int status;
  status = 0;
  CANctr->SetByteTransmit(status, 0);
  CANctr->SetByteTransmit(valueTransmit_byte0, 1);
  CANctr->SetByteTransmit(valueTransmit_byte1, 2);

  // Serial.println((String) "sensorBits: " + sensorBits);
  if (CANctr->CAN_Send()){
    timeSave_LED_CANsend = millis();
  }else{
    Serial.println("CPD CAN_Send - ERROR");
  }
}

void CPD_controller::CAN_send(){
  if ((millis() - preTime_sendCAN) > (1000 / FREQUENCY_sendCAN))
  {   
    preTime_sendCAN = millis();
    CAN_transmit();
  }
}

void CPD_controller::loop_run()
{
  if ((millis() - timeSave_LED_CANsend) > (1000 / 10))
  {
    digitalWrite(LED_CAN_SEND, 0);
  }else{
    digitalWrite(LED_CAN_SEND, 1);
  }
  // -
  if ((millis() - timeSave_LED_CANreceived) > (1000 / 10))
  {
    digitalWrite(LED_CAN_REC, 0);
  }else{
    digitalWrite(LED_CAN_REC, 1);
  }
  // --
  readInput();
  writeOutput();
  // debuge();
}

void CPD_controller::debuge()
{
  if ((millis() - timeSave_writePCF) > (250))
  {
    timeSave_writePCF = millis();
    digitalWrite(ONBOARD_OUTPUT1, status_out);
    digitalWrite(ONBOARD_OUTPUT2, status_out);
    digitalWrite(ONBOARD_OUTPUT3, status_out);
    digitalWrite(ONBOARD_OUTPUT4, status_out);
    pcf8575.digitalWrite(PCF_OUTPUT5, status_out);
    pcf8575.digitalWrite(PCF_OUTPUT6, status_out);
    pcf8575.digitalWrite(PCF_OUTPUT7, status_out);
    pcf8575.digitalWrite(PCF_OUTPUT8, status_out);
    pcf8575.digitalWrite(PCF_OUTPUT9, status_out);
    pcf8575.digitalWrite(PCF_OUTPUT10, status_out);
    pcf8575.digitalWrite(PCF_OUTPUT11, status_out);
    pcf8575.digitalWrite(PCF_OUTPUT12, status_out);
    if (status_out == 1){
      status_out = 0;
    }else{
      status_out = 1;
    }
  }
}

void CPD_controller::led_start(){
  digitalWrite(LED_CAN_REC,  0);
  digitalWrite(LED_CAN_SEND, 0);
  digitalWrite(LED_ROS_REC,  0);
  digitalWrite(LED_ROS_SEND, 0);
  delay(10);
  digitalWrite(LED_CAN_REC,  1);
  delay(60);
  digitalWrite(LED_CAN_REC,  0);
  digitalWrite(LED_CAN_SEND, 1);
  delay(60);
  digitalWrite(LED_CAN_SEND, 0);
  digitalWrite(LED_ROS_REC,  1);
  delay(60);
  digitalWrite(LED_ROS_REC,  0);
  digitalWrite(LED_ROS_SEND, 1);
  delay(60);
  digitalWrite(LED_ROS_SEND, 0);
  delay(100);
  digitalWrite(LED_CAN_REC,  0);
  digitalWrite(LED_CAN_SEND, 0);
  digitalWrite(LED_ROS_REC,  0);
  digitalWrite(LED_ROS_SEND, 0);
}

bool CPD_controller::getBit_from2Byte(int pos, uint16_t valByte)
{
  bool outValue;
  outValue = valByte & (1 << pos);
  return outValue;
}

uint8_t CPD_controller::setBit_toByte(bool bit0, bool bit1, bool bit2, bool bit3, bool bit4, bool bit5, bool bit6, bool bit7)
{
  uint8_t outValue;
  outValue = bit0*pow(2, 0) + bit1*pow(2, 1) + bit2*pow(2, 2) + bit3*pow(2, 3) + bit4*pow(2, 4) + bit5*pow(2, 5) + bit6*pow(2, 6) + bit7*pow(2, 7);
  return outValue;
}

uint8_t CPD_controller::int_to2Bytes(uint16_t valueIn, int pos_out)
{
  int byte0, byte1;
  byte1 = valueIn/256;
  byte0 = valueIn - byte1*256;
  if (pos_out == 0){
    return byte0;
  }else{
    return byte1;
  }
}

// uint8_t CPD_controller::2Bytes_toInt(uint16_t byte0, int byte1)
// {
//   int byte0, byte1;
//   byte1 = valueIn/256;
//   byte0 = valueIn - byte1*256;
//   if (pos_out == 0){
//     return byte0;
//   }else{
//     return byte1;
//   }
// }
