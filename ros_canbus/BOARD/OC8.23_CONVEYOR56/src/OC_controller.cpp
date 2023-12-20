#include "OC_controller.h"

OC_controller::OC_controller(CAN_manager* _CANctr)
{
    CANctr = _CANctr;
}

OC_controller::~OC_controller()
{
  delete CAN_sendData;
  delete CAN_receivedData;
  delete CAN_command;
}

void OC_controller::setupBegin() // - OK
{
  Serial.begin(57600);
  pinMode(SENSOR_1, INPUT_PULLUP);
  pinMode(SENSOR_2, INPUT_PULLUP);
  pinMode(SENSOR_3, INPUT_PULLUP);
  pinMode(SENSOR_4, INPUT_PULLUP);
  pinMode(SENSOR_5, INPUT_PULLUP);
  pinMode(SENSOR_6, INPUT_PULLUP);
  pinMode(SENSOR_7, INPUT_PULLUP);
  // -
  pinMode(EMERGENCY_OUT, OUTPUT);
  // -
  pinMode(CTR_ENABLE1, OUTPUT);
  pinMode(CTR_PWM_H1,  OUTPUT);
  pinMode(CTR_PWM_L1,  OUTPUT);
  // -
  pinMode(CTR_ENABLE2, OUTPUT);
  pinMode(CTR_PWM_H2,  OUTPUT);
  pinMode(CTR_PWM_L2,  OUTPUT);

  // -------------------------------
  digitalWrite(EMERGENCY_OUT, LOW);
  // -
  digitalWrite(CTR_ENABLE1, LOW);
  digitalWrite(CTR_PWM_H1,  LOW);
  digitalWrite(CTR_PWM_L1,  LOW);
  // -
  digitalWrite(CTR_ENABLE2, LOW);
  digitalWrite(CTR_PWM_H2,  LOW);
  digitalWrite(CTR_PWM_L2,  LOW);

  // ----------- PWM SETUP --------------------
  // - ledcSetup(ledChannel, freq, resolution);
  ledcSetup(channelPWM_H1, 1000, 10);
  ledcSetup(channelPWM_L1, 1000, 10);
  ledcSetup(channelPWM_H2, 1000, 10);
  ledcSetup(channelPWM_L2, 1000, 10);   

  // - ledcAttachPin(ledPin, ledChannel);
  ledcAttachPin(CTR_PWM_H1, channelPWM_H1);
  ledcAttachPin(CTR_PWM_L1, channelPWM_L1);
  ledcAttachPin(CTR_PWM_H2, channelPWM_H2);
  ledcAttachPin(CTR_PWM_L2, channelPWM_L2);
}

bool OC_controller::readSensorAhead1(bool statusRight){
  if (digitalRead(SS_LIMMIT_AHEAD1) == statusRight){

  }else{
    timeStart_sensorAhead1 = millis();
  }

  if (millis() - timeStart_sensorAhead1 >= timeCheckSensor){
    return 1;
  } else{
    return 0;
  }
}

bool OC_controller::readSensorBehind1(bool statusRight){
  if (digitalRead(SS_LIMMIT_BEHIND1) == statusRight){

  }else{
    timeStart_sensorBehind1 = millis();
  }

  if (millis() - timeStart_sensorBehind1 >= timeCheckSensor){
    return 1;
  } else{
    return 0;
  }
}

bool OC_controller::readSensorRack1(bool statusRight){
  if (digitalRead(SS_OBJECT_DETECTION1) == statusRight){

  }else{
    timeStart_sensorRack1 = millis();
  }

  if (millis() - timeStart_sensorRack1 >= timeCheckSensor){
    return 1;
  } else{
    return 0;
  }
}

bool OC_controller::readSensorAhead2(bool statusRight){
  if (digitalRead(SS_LIMMIT_AHEAD2) == statusRight){

  }else{
    timeStart_sensorAhead2 = millis();
  }

  if (millis() - timeStart_sensorAhead2 >= timeCheckSensor){
    return 1;
  } else{
    return 0;
  }
}

bool OC_controller::readSensorBehind2(bool statusRight){
  if (digitalRead(SS_LIMMIT_BEHIND2) == statusRight){

  }else{
    timeStart_sensorBehind2 = millis();
  }

  if (millis() - timeStart_sensorBehind2 >= timeCheckSensor){
    return 1;
  } else{
    return 0;
  }
}

bool OC_controller::readSensorRack2(bool statusRight){
  if (digitalRead(SS_OBJECT_DETECTION2) == statusRight){

  }else{
    timeStart_sensorRack2 = millis();
  }

  if (millis() - timeStart_sensorRack2 >= timeCheckSensor){
    return 1;
  } else{
    return 0;
  }
}

void OC_controller::transmitRack1(int percentSpeed) //- OK
{
  int speedMax = 0;
  if (percentSpeed >= 100)
    speedMax = 1024; 
  else if (percentSpeed < 60)
    speedMax = 600;
  else
    speedMax = 1024*(percentSpeed/100.);

  speedMax = 900;
  // - Quay thuan
	if (step_transmitRack1 == 0){ // điều khiển chiều.
		timeStart_transmitRack1 = millis();
    speed_transmitRack1 = 0;
    ledcWrite(channelPWM_H1, speed_transmitRack1);
    ledcWrite(channelPWM_L1, speed_transmitRack1);
		digitalWrite(CTR_PWM_H1, LOW);
    digitalWrite(CTR_PWM_L1, LOW);
    digitalWrite(CTR_ENABLE1, HIGH);
		step_transmitRack1 = 1;

	}else if (step_transmitRack1 == 1){ // - Tăng dần tốc độ.		
		if (readSensorAhead1(LOW) == 1 && readSensorBehind1(LOW) == 1){ // - Thoát được cảm biến giới hạn Trước và Sau.
      if (speed_transmitRack1 == 0){
        step_transmitRack1 = 5;
      }else{
  			step_transmitRack1 = 4;
      }
		}
    // -
    speed_transmitRack1 += 6;
		if (speed_transmitRack1 >= speedMax){ // - Đạt tốc độ giới hạn.
			step_transmitRack1 = 2;
			speed_transmitRack1 = speedMax;
		}
    // ledcWrite(channelPWM_H1, speed_transmitRack1);

	}else if (step_transmitRack1 == 2){ // Đợi bắt được cảm biến.
		if (millis() - timeStart_transmitRack1 > timeCheckErrorRun){ // - Kiểm tra lỗi theo thời gian vận hành.
      step_transmitRack1 = 6;
		}

		if (readSensorAhead1(LOW) == 1 && readSensorBehind1(LOW) == 1){ // - Thoát được cảm biến giới hạn Trước và Sau.
			step_transmitRack1 = 3;
      timeStart_transmitRack1 = millis();
		}

  }else if (step_transmitRack1 == 3){ // Chay them 1 thoi gian.
		if (millis() - timeStart_transmitRack1 > 1000){ // -
      step_transmitRack1 = 4;
		}

	}else if (step_transmitRack1 == 4){ // Giảm dần tốc độ về 0.
    speed_transmitRack1 -= 6;
		if(speed_transmitRack1 <= 0){
			speed_transmitRack1 = 0;
      step_transmitRack1 = 5;
    }
    // ledcWrite(channelPWM_H1, speed_transmitRack1);

	}else if (step_transmitRack1 == 5){ // - Hoàn thành.	
    speed_transmitRack1 = 0;
    ledcWrite(channelPWM_H1, speed_transmitRack1);
    digitalWrite(CTR_PWM_H1, LOW);
		digitalWrite(CTR_ENABLE1, LOW);

	}else if (step_transmitRack1 == 6){ // - ERROR - Giam dan toc do.
    speed_transmitRack1 -= 6;
		if(speed_transmitRack1 <= 0){
			speed_transmitRack1 = 0;
      step_transmitRack1 = 7;
    }
    // ledcWrite(channelPWM_H1, speed_transmitRack1);

	}else if (step_transmitRack1 == 7){ // - ERROR - Stop
    speed_transmitRack1 = 0;
    ledcWrite(channelPWM_H1, speed_transmitRack1);
    digitalWrite(CTR_PWM_H1, LOW);
		digitalWrite(CTR_ENABLE1, LOW);
	}
  ledcWrite(channelPWM_H1, speed_transmitRack1);
}

void OC_controller::receivedRack1(int percentSpeed)
{
  int speedMax = 0;
  if (percentSpeed >= 100)
    speedMax = 1024; 
  else if (percentSpeed < 60)
    speedMax = 600;
  else
    speedMax = 1024*(percentSpeed/100.);

  speedMax = 900;
  // - Quay thuan
	if (step_receivedRack1 == 0){ // điều khiển chiều.
		timeStart_receivedRack1 = millis();
		speed_receivedRack1 = 0;
		ledcWrite(channelPWM_L1, speed_receivedRack1);		
		digitalWrite(CTR_PWM_H1, LOW);
    digitalWrite(CTR_PWM_L1, LOW);
    digitalWrite(CTR_ENABLE1, HIGH);
		step_receivedRack1 = 1;

	}else if (step_receivedRack1 == 1){ // - Tăng dần tốc độ.		
		if (readSensorAhead1(HIGH) == 1 && readSensorBehind1(HIGH) == 1){ // - Bắt được cảm biến giới hạn Truoc + Sau.
      if (speed_receivedRack1 == 0){
        step_receivedRack1 = 5;
      }else{
        step_receivedRack1 = 4;
      }
		}
    // -
    speed_receivedRack1 += 6;
		if (speed_receivedRack1 >= speedMax){ // - Đạt tốc độ giới hạn.
			step_receivedRack1 = 2;
			speed_receivedRack1 = speedMax;
		}
    // ledcWrite(channelPWM_L1, speed_receivedRack1);

	}else if (step_receivedRack1 == 2){ // Đợi bắt được cảm biến.
		if(millis() - timeStart_receivedRack1 > timeCheckErrorRun){ // - Kiểm tra lỗi theo thời gian vận hành.
      step_receivedRack1 = 6;
		}

		if (readSensorAhead1(HIGH) == 1 && readSensorBehind1(HIGH) == 1){ // - Bắt được cảm biến giới hạn Truoc + Sau.
			step_receivedRack1 = 3;
      timeStart_receivedRack1 = millis();
		}

  }else if (step_receivedRack1 == 3){ // Chay them 1 thoi gian.
		if(millis() - timeStart_receivedRack1 > 10){ // -
      step_receivedRack1 = 4;
		}

	}else if (step_receivedRack1 == 4){ // Giảm dần tốc độ về 0.
		speed_receivedRack1 -= 6;
		if(speed_receivedRack1 <= 0){
      speed_receivedRack1 = 0;
      step_receivedRack1 = 5;
		}
    // ledcWrite(channelPWM_L1, speed_receivedRack1);

	}else if (step_receivedRack1 == 5){ // - Hoàn thành.	
		speed_receivedRack1 = 0;
    ledcWrite(channelPWM_L1, speed_receivedRack1);
    digitalWrite(CTR_PWM_L1, LOW);
		digitalWrite(CTR_ENABLE1, LOW);

	}else if (step_receivedRack1 == 6){ // - ERROR - Giam dan toc do.
    speed_receivedRack1 -= 6;
    if(speed_receivedRack1 <= 0){
      speed_receivedRack1 = 0;
      step_receivedRack1 = 7;
    }
    // ledcWrite(channelPWM_L1, speed_receivedRack1);

	}else if (step_receivedRack1 == 7){ // - ERROR - Stop.
		speed_receivedRack1 = 0;
    ledcWrite(channelPWM_L1, speed_receivedRack1);
    digitalWrite(CTR_PWM_L1, LOW);
		digitalWrite(CTR_ENABLE1, LOW);
  }
  ledcWrite(channelPWM_L1, speed_receivedRack1);		
}

void OC_controller::transmitRack2(int percentSpeed) //- OK
{
  int speedMax = 0;
  if (percentSpeed >= 100)
    speedMax = 1024; 
  else if (percentSpeed < 60)
    speedMax = 600;
  else
    speedMax = 1024*(percentSpeed/100.);

  speedMax = 900;
  // - Quay thuan
	if (step_transmitRack2 == 0){ // điều khiển chiều.
		timeStart_transmitRack2 = millis();
    speed_transmitRack2 = 0;
    ledcWrite(channelPWM_H2, speed_transmitRack2);
    ledcWrite(channelPWM_L2, speed_transmitRack2);
		digitalWrite(CTR_PWM_H2, LOW);
    digitalWrite(CTR_PWM_L2, LOW);
    digitalWrite(CTR_ENABLE2, HIGH);
		step_transmitRack2 = 1;

	}else if (step_transmitRack2 == 1){ // - Tăng dần tốc độ.		
		if (readSensorAhead2(LOW) == 1 && readSensorBehind2(LOW) == 1){ // - Thoát được cảm biến giới hạn Trước và Sau.
      if (speed_transmitRack2 == 0){
        step_transmitRack2 = 5;
      }else{
  			step_transmitRack2 = 4;
      }
		}
    // -
    speed_transmitRack2 += 6;
		if (speed_transmitRack2 >= speedMax){ // - Đạt tốc độ giới hạn.
			step_transmitRack2 = 2;
			speed_transmitRack2 = speedMax;
		}
    // ledcWrite(channelPWM_H2, speed_transmitRack2);

	}else if (step_transmitRack2 == 2){ // Đợi bắt được cảm biến.
		if (millis() - timeStart_transmitRack2 > timeCheckErrorRun){ // - Kiểm tra lỗi theo thời gian vận hành.
      step_transmitRack2 = 6;
		}

		if (readSensorAhead2(LOW) == 1 && readSensorBehind2(LOW) == 1){ // - Thoát được cảm biến giới hạn Trước và Sau.
			step_transmitRack2 = 3;
      timeStart_transmitRack2 = millis();
		}

  }else if (step_transmitRack2 == 3){ // Chay them 1 thoi gian.
		if (millis() - timeStart_transmitRack2 > 1000){ // -
      step_transmitRack2 = 4;
		}

	}else if (step_transmitRack2 == 4){ // Giảm dần tốc độ về 0.
    speed_transmitRack2 -= 6;
		if (speed_transmitRack2 <= 0){
			speed_transmitRack2 = 0;
      step_transmitRack2 = 5;
    }
    // ledcWrite(channelPWM_H2, speed_transmitRack2);

	}else if (step_transmitRack2 == 5){ // - Hoàn thành.	
    speed_transmitRack2 = 0;
    ledcWrite(channelPWM_H2, speed_transmitRack2);
    digitalWrite(CTR_PWM_H2, LOW);
		digitalWrite(CTR_ENABLE2, LOW);

	}else if (step_transmitRack2 == 6){ // - ERROR - Giam dan toc do.
    speed_transmitRack2 -= 6;
		if(speed_transmitRack2 <= 0){
			speed_transmitRack2 = 0;
      step_transmitRack2 = 7;
    }
    // ledcWrite(channelPWM_H2, speed_transmitRack2);

	}else if (step_transmitRack2 == 7){ // - ERROR - Stop
    speed_transmitRack2 = 0;
    ledcWrite(channelPWM_H2, speed_transmitRack2);
    digitalWrite(CTR_PWM_H2, LOW);
		digitalWrite(CTR_ENABLE2, LOW);
	}
  ledcWrite(channelPWM_H2, speed_transmitRack2);
}

void OC_controller::receivedRack2(int percentSpeed)
{
  int speedMax = 0;
  if (percentSpeed >= 100)
    speedMax = 1024; 
  else if (percentSpeed < 60)
    speedMax = 600;
  else
    speedMax = 1024*(percentSpeed/100.);

  speedMax = 900;
  // - Quay thuan
	if (step_receivedRack2 == 0){ // điều khiển chiều.
		timeStart_receivedRack2 = millis();
		speed_receivedRack2 = 0;
		ledcWrite(channelPWM_L2, speed_receivedRack2);		
		digitalWrite(CTR_PWM_H2, LOW);
    digitalWrite(CTR_PWM_L2, LOW);
    digitalWrite(CTR_ENABLE2, HIGH);
		step_receivedRack2 = 1;

	}else if (step_receivedRack2 == 1){ // - Tăng dần tốc độ.		
		if (readSensorAhead2(HIGH) == 1 && readSensorBehind2(HIGH) == 1){ // - Bắt được cảm biến giới hạn Truoc + Sau.
      if (speed_receivedRack2 == 0){
        step_receivedRack2 = 5;
      }else{
        step_receivedRack2 = 4;
      }
		}
    // -
    speed_receivedRack2 += 6;
		if (speed_receivedRack2 >= speedMax){ // - Đạt tốc độ giới hạn.
			step_receivedRack2 = 2;
			speed_receivedRack2 = speedMax;
		}
    // ledcWrite(channelPWM_L2, speed_receivedRack2);

	}else if (step_receivedRack2 == 2){ // Đợi bắt được cảm biến.
		if(millis() - timeStart_receivedRack2 > timeCheckErrorRun){ // - Kiểm tra lỗi theo thời gian vận hành.
      step_receivedRack2 = 6;
		}

		if (readSensorAhead2(HIGH) == 1 && readSensorBehind2(HIGH) == 1){ // - Bắt được cảm biến giới hạn Truoc + Sau.
			step_receivedRack2 = 3;
      timeStart_receivedRack2 = millis();
		}

  }else if (step_receivedRack2 == 3){ // Chay them 1 thoi gian.
		if(millis() - timeStart_receivedRack2 > 10){ // -
      step_receivedRack2 = 4;
		}

	}else if (step_receivedRack2 == 4){ // Giảm dần tốc độ về 0.
		speed_receivedRack2 -= 6;
		if(speed_receivedRack2 <= 0){
      speed_receivedRack2 = 0;
      step_receivedRack2 = 5;
		}
    // ledcWrite(channelPWM_L2, speed_receivedRack2);

	}else if (step_receivedRack2 == 5){ // - Hoàn thành.	
		speed_receivedRack2 = 0;
    ledcWrite(channelPWM_L2, speed_receivedRack2);
    digitalWrite(CTR_PWM_L2, LOW);
		digitalWrite(CTR_ENABLE2, LOW);

	}else if (step_receivedRack2 == 6){ // - ERROR - Giam dan toc do.
    speed_receivedRack2 -= 6;
    if(speed_receivedRack2 <= 0){
      speed_receivedRack2 = 0;
      step_receivedRack2 = 7;
    }
    // ledcWrite(channelPWM_L2, speed_receivedRack2);

	}else if (step_receivedRack2 == 7){ // - ERROR - Stop.
		speed_receivedRack2 = 0;
    ledcWrite(channelPWM_L2, speed_receivedRack2);
    digitalWrite(CTR_PWM_L2, LOW);
		digitalWrite(CTR_ENABLE2, LOW);
  }
  ledcWrite(channelPWM_L2, speed_receivedRack2);
}

void OC_controller::stopAndReset1(){
  
  digitalWrite(CTR_PWM_L1, LOW);
  digitalWrite(CTR_PWM_H1, LOW);

	speed_transmitRack1 = 0;
	speed_receivedRack1 = 0;
  ledcWrite(channelPWM_L1, speed_transmitRack1);
  ledcWrite(channelPWM_H1, speed_receivedRack1);

  digitalWrite(CTR_ENABLE1, LOW);
	step_transmitRack1 = 0;
	step_receivedRack1 = 0;
}

void OC_controller::stopAndReset2(){
  
  digitalWrite(CTR_PWM_L2, LOW);
  digitalWrite(CTR_PWM_H2, LOW);

	speed_transmitRack2 = 0;
	speed_receivedRack2 = 0;
  ledcWrite(channelPWM_L2, speed_transmitRack2);
  ledcWrite(channelPWM_H2, speed_receivedRack2);

  digitalWrite(CTR_ENABLE2, LOW);
	step_transmitRack2 = 0;
	step_receivedRack2 = 0;
}

void OC_controller::tryRun_conveyor1(int percentSpeed){
  int speedMax = 0;
  if (percentSpeed >= 100)
    speedMax = 1024; 
  else if (percentSpeed < 50)
    speedMax = 1024/2;
  else
    speedMax = 1024*(percentSpeed/100.);

  // Serial.println(readSensorBehind1(HIGH)); // digitalRead(SS_LIMMIT_BEHIND1) readSensorBehind1(HIGH)

  if (step_transmitRack1 == 0){ // - 
		speed_transmitRack1 = 0;
    ledcWrite(channelPWM_H1, speed_transmitRack1);
    ledcWrite(channelPWM_L1, speed_transmitRack1);
		digitalWrite(CTR_PWM_H1, LOW);
    digitalWrite(CTR_PWM_L1, LOW);    
		digitalWrite(CTR_ENABLE1, LOW);

  }else if (step_transmitRack1 == 1){ // - Tăng dần tốc độ.		
		if (speed_transmitRack1 >= speedMax){ // - Đạt tốc độ giới hạn.
			step_transmitRack1 = 2;
			speed_transmitRack1 = speedMax;
      timeStart_transmitRack1 = millis();
		}else{						                    // - Tăng dần tốc độ.
			speed_transmitRack1 += 10;
		}
    ledcWrite(channelPWM_H1, speed_transmitRack1);
      
	}else if (step_transmitRack1 == 2){ // - 
		if(millis() - timeStart_transmitRack1 > 60000){ // - Theo thời gian vận hành.
      step_transmitRack1 = 3;
		}    

	}else if (step_transmitRack1 == 3){ // - Giam dần tốc độ.		
		if (speed_transmitRack1 <= 40){ // - Đạt tốc độ giới hạn.
			step_transmitRack1 = 4;
			speed_transmitRack1 = 0;
      ledcWrite(channelPWM_H1, speed_transmitRack1);
      timeStart_transmitRack1 = millis();
  		digitalWrite(CTR_PWM_H1, LOW);
      digitalWrite(CTR_PWM_L1, LOW);
		}else{						                    // - Tăng dần tốc độ.
			speed_transmitRack1 -= 10;
		}
    ledcWrite(channelPWM_H1, speed_transmitRack1);

	}else if (step_transmitRack1 == 4){ // - 
		if(millis() - timeStart_transmitRack1 > 3000){ // - Dung -> Dao chieu.
      step_transmitRack1 = 5;
  		digitalWrite(CTR_PWM_H1, LOW);
      digitalWrite(CTR_PWM_L1, LOW);
      speed_transmitRack1 = 0;
      ledcWrite(channelPWM_H1, speed_transmitRack1);
      ledcWrite(channelPWM_L1, speed_transmitRack1);
      speed_transmitRack1 = 60;
		}    

	}else if (step_transmitRack1 == 5){ // - Tăng dần tốc độ.		
		if (speed_transmitRack1 >= speedMax){ // - Đạt tốc độ giới hạn.
			step_transmitRack1 = 6;
			speed_transmitRack1 = speedMax;
      timeStart_transmitRack1 = millis();
		}else{						                    // - Tăng dần tốc độ.
			speed_transmitRack1 += 10;
		}
    ledcWrite(channelPWM_L1, speed_transmitRack1);
      
	}else if (step_transmitRack1 == 6){ // - 
		if(millis() - timeStart_transmitRack1 > 60000){ // - Chay theo thời gian vận hành.
      step_transmitRack1 = 7;
		}    

	}else if (step_transmitRack1 == 7){ // - Giam dần tốc độ.		
		if (speed_transmitRack1 <= 40){ // - Đạt tốc độ giới hạn.
			step_transmitRack1 = 8;
			speed_transmitRack1 = 0;
      timeStart_transmitRack1 = millis();
		}else{						                    // - Tăng dần tốc độ.
			speed_transmitRack1 -= 10;
		}
    ledcWrite(channelPWM_L1, speed_transmitRack1);

	}else if (step_transmitRack1 == 8){ // - 
		if(millis() - timeStart_transmitRack1 > 3000){ // - Dung -> Dao chieu.
      step_transmitRack1 = 1;
  		digitalWrite(CTR_PWM_H1, LOW);
      digitalWrite(CTR_PWM_L1, LOW);
      speed_transmitRack1 = 0;
      ledcWrite(channelPWM_H1, speed_transmitRack1);
      ledcWrite(channelPWM_L1, speed_transmitRack1);
      speed_transmitRack1 = 60;
		}
  }
}

void OC_controller::tryRun_conveyor2(int percentSpeed){
  int speedMax = 0;
  if (percentSpeed >= 100)
    speedMax = 1024; 
  else if (percentSpeed < 50)
    speedMax = 1024/2;
  else
    speedMax = 1024*(percentSpeed/100.);

  
  Serial.print( digitalRead(SENSOR_1) ); // digitalRead(SS_LIMMIT_BEHIND1) readSensorBehind1(HIGH)
  Serial.print( digitalRead(SENSOR_2) );
  Serial.print( digitalRead(SENSOR_3) );
  Serial.print( digitalRead(SENSOR_4) );
  Serial.print( digitalRead(SENSOR_5) );
  Serial.print( digitalRead(SENSOR_6) );
  Serial.println( digitalRead(SENSOR_7) );
  
  if (step_transmitRack2 == 0){ // - 
		speed_transmitRack2 = 0;
    ledcWrite(channelPWM_H2, speed_transmitRack2);
    ledcWrite(channelPWM_L2, speed_transmitRack2);
		
		digitalWrite(CTR_PWM_H2, LOW);
    digitalWrite(CTR_PWM_L2, LOW);
    digitalWrite(CTR_ENABLE2, LOW);

  }else if (step_transmitRack2 == 1){ // - Tăng dần tốc độ.		
		if (speed_transmitRack2 >= speedMax){ // - Đạt tốc độ giới hạn.
			step_transmitRack2 = 2;
			speed_transmitRack2 = speedMax;
      timeStart_transmitRack2 = millis();
		}else{						                    // - Tăng dần tốc độ.
			speed_transmitRack2 += 10;
		}
    ledcWrite(channelPWM_H2, speed_transmitRack2);
      
	}else if (step_transmitRack2 == 2){ // - 
		if(millis() - timeStart_transmitRack2 > 60000){ // - Theo thời gian vận hành.
      step_transmitRack2 = 3;
		}    

	}else if (step_transmitRack2 == 3){ // - Giam dần tốc độ.		
		if (speed_transmitRack2 <= 40){ // - Đạt tốc độ giới hạn.
			step_transmitRack2 = 4;
			speed_transmitRack2 = 0;
      ledcWrite(channelPWM_H2, speed_transmitRack2);
      timeStart_transmitRack2 = millis();
  		digitalWrite(CTR_PWM_H2, LOW);
      digitalWrite(CTR_PWM_L2, LOW);
		}else{						                    // - Tăng dần tốc độ.
			speed_transmitRack2 -= 10;
		}
    ledcWrite(channelPWM_H2, speed_transmitRack2);

	}else if (step_transmitRack2 == 4){ // - 
		if(millis() - timeStart_transmitRack2 > 3000){ // - Dung -> Dao chieu.
      step_transmitRack2 = 5;
  		digitalWrite(CTR_PWM_H2, LOW);
      digitalWrite(CTR_PWM_L2, LOW);
      speed_transmitRack2 = 0;
      ledcWrite(channelPWM_H2, speed_transmitRack2);
      ledcWrite(channelPWM_L2, speed_transmitRack2);
      speed_transmitRack2 = 60;
		}    

	}else if (step_transmitRack2 == 5){ // - Tăng dần tốc độ.		
		if (speed_transmitRack2 >= speedMax){ // - Đạt tốc độ giới hạn.
			step_transmitRack2 = 6;
			speed_transmitRack2 = speedMax;
      timeStart_transmitRack2 = millis();
		}else{						                    // - Tăng dần tốc độ.
			speed_transmitRack2 += 10;
		}
    ledcWrite(channelPWM_L2, speed_transmitRack2);
      
	}else if (step_transmitRack2 == 6){ // - 
		if(millis() - timeStart_transmitRack2 > 60000){ // - Chay theo thời gian vận hành.
      step_transmitRack2 = 7;
		}    

	}else if (step_transmitRack2 == 7){ // - Giam dần tốc độ.		
		if (speed_transmitRack2 <= 40){ // - Đạt tốc độ giới hạn.
			step_transmitRack2 = 8;
			speed_transmitRack2 = 0;
      timeStart_transmitRack2 = millis();
		}else{						                    // - Tăng dần tốc độ.
			speed_transmitRack2 -= 10;
		}
    ledcWrite(channelPWM_L2, speed_transmitRack2);

	}else if (step_transmitRack2 == 8){ // - 
		if(millis() - timeStart_transmitRack2 > 3000){ // - Dung -> Dao chieu.
      step_transmitRack2 = 1;
  		digitalWrite(CTR_PWM_H2, LOW);
      digitalWrite(CTR_PWM_L2, LOW);
      speed_transmitRack2 = 0;
      ledcWrite(channelPWM_H2, speed_transmitRack2);
      ledcWrite(channelPWM_L2, speed_transmitRack2);
      speed_transmitRack2 = 60;
		}
  }
}

void OC_controller::tryRun(){ 
  Serial.print(readSensorBehind1(HIGH)); // digitalRead(SS_LIMMIT_BEHIND1) readSensorBehind1(HIGH)
  Serial.println(readSensorRack1(HIGH)); // digitalRead(SS_LIMMIT_BEHIND1) readSensorBehind1(HIGH)

	if (readSensorBehind1(HIGH) == 1){ // - Stop. SS3
    stopAndReset1();
		stopAndReset2();
  }

	if (readSensorRack1(HIGH) == 1){ // - Kich hoat. SS4
		step_transmitRack1 = 1;
    digitalWrite(CTR_ENABLE1, HIGH);

		step_transmitRack2 = 1;
    digitalWrite(CTR_ENABLE2, HIGH);
  }
  
  tryRun_conveyor1(98);
  tryRun_conveyor2(98);
}

void OC_controller::OC_CAN_Receive(){ // - OK
  if (CANctr->CAN_ReceiveFrom(ID_RTC)){
    if (CANctr->GetByteReceived(POS_ID) == ID_OC){
      CAN_receivedData->mission1 = CANctr->GetByteReceived(POS_MISSION1);
      CAN_receivedData->speed1   = CANctr->GetByteReceived(POS_SPEED1);

      CAN_receivedData->mission2 = CANctr->GetByteReceived(POS_MISSION2);
      CAN_receivedData->speed2   = CANctr->GetByteReceived(POS_SPEED2);
    }
  }
}

void OC_controller::OC_CAN_Transmit(){ // - OK
  // - SetByteTransmit(data, position)
  CANctr->SetByteTransmit(CAN_sendData->status1, POS_status1);
  CANctr->SetByteTransmit(CAN_sendData->statusSensor_limitAhead1,  POS_limitAhead1);
  CANctr->SetByteTransmit(CAN_sendData->statusSensor_limitBehind1, POS_limitBehind1);
  CANctr->SetByteTransmit(CAN_sendData->statusSensor_checkRack1,   POS_checkRack1);
  
  CANctr->SetByteTransmit(CAN_sendData->status2, POS_status2);
  CANctr->SetByteTransmit(CAN_sendData->statusSensor_limitAhead2,  POS_limitAhead2);
  CANctr->SetByteTransmit(CAN_sendData->statusSensor_limitBehind2, POS_limitBehind2);
  CANctr->SetByteTransmit(CAN_sendData->statusSensor_checkRack2,   POS_checkRack2);

  if (CANctr->CAN_Send()){

  }else{
    Serial.println("OC CAN_Send - ERROR");
  }
}

void OC_controller::OC_CAN_send(){ // - OK
  if ((millis() - preTime_sendCAN) > (1000 / FREQUENCY_sendCAN))
  {   
    preTime_sendCAN = millis();
    OC_CAN_Transmit();
  }
}

void OC_controller::OC_loop(){
  // ------ READ ALL------
  if (digitalRead(SS_LIMMIT_AHEAD1) == LOW)
    CAN_sendData->statusSensor_limitAhead1 = 0;
  else
    CAN_sendData->statusSensor_limitAhead1 = 1;

  if (digitalRead(SS_LIMMIT_BEHIND1) == LOW)
    CAN_sendData->statusSensor_limitBehind1 = 0;
  else
    CAN_sendData->statusSensor_limitBehind1 = 1;

  if (digitalRead(SS_OBJECT_DETECTION1) == LOW)
    CAN_sendData->statusSensor_checkRack1 = 0;
  else
    CAN_sendData->statusSensor_checkRack1 = 1;

  // --
  if (digitalRead(SS_LIMMIT_AHEAD2) == LOW)
    CAN_sendData->statusSensor_limitAhead2 = 0;
  else
    CAN_sendData->statusSensor_limitAhead2 = 1;

  if (digitalRead(SS_LIMMIT_BEHIND2) == LOW)
    CAN_sendData->statusSensor_limitBehind2 = 0;
  else
    CAN_sendData->statusSensor_limitBehind2 = 1;

  if (digitalRead(SS_OBJECT_DETECTION2) == LOW)
    CAN_sendData->statusSensor_checkRack2 = 0;
  else
    CAN_sendData->statusSensor_checkRack2 = 1;

  // ------ WRITE CONVEYOR1------
  if (CAN_command->mission1 != CAN_receivedData->mission1){
    CAN_command->mission1 = CAN_receivedData->mission1;
    CAN_command->speed1 = CAN_receivedData->speed1;  
    stopAndReset1();
  }

  if (CAN_command->mission2 != CAN_receivedData->mission2){
    CAN_command->mission2 = CAN_receivedData->mission2;
    CAN_command->speed2 = CAN_receivedData->speed2;
    stopAndReset2();
  }

  // ------ RUN CONVEYOR1------
  if (CAN_command->mission1 == MISSION_RESET){
    // Serial.println("CY1: MISSION_RESET");
    stopAndReset1();
    CAN_sendData->status1 = 0;

  }else if (CAN_command->mission1 == MISSION_RECEIVE){
    // Serial.println("CY1: MISSION_RECEIVE");
    receivedRack1(CAN_command->speed1);
    // - Status
    if (step_receivedRack1 == 0){
      CAN_sendData->status1 = 0;

    }else if (step_receivedRack1 == 1 || step_receivedRack1 == 2 || step_receivedRack1 == 3 || step_receivedRack1 == 4 || step_receivedRack1 == 6){
      CAN_sendData->status1 = 1;

    }else if (step_receivedRack1 == 5){
      CAN_sendData->status1 = 3;

    }else if (step_receivedRack1 == 7){
      CAN_sendData->status1 = -1;

    }else{
      CAN_sendData->status1 = -3;
    }

  }else if (CAN_command->mission1 == MISSION_TRANSMISSION){
    // Serial.println("CY1: MISSION_TRANSMISSION");
    transmitRack1(CAN_command->speed1);
    // - Status
    if (step_transmitRack1 == 0){
      CAN_sendData->status1 = 0;

    }else if (step_transmitRack1 == 1 || step_transmitRack1 == 2 || step_transmitRack1 == 3 || step_transmitRack1 == 4 || step_transmitRack1 == 6){
      CAN_sendData->status1 = 2;

    }else if (step_transmitRack1 == 5){
      CAN_sendData->status1 = 4;

    }else if (step_transmitRack1 == 7){
      CAN_sendData->status1 = -2;

    }else{
      CAN_sendData->status1 = -4;
    }

  }else{
    // Serial.println("CY1: UNK");
    stopAndReset1();
    CAN_sendData->status1 = 0;
  }

  // ------ RUN CONVEYOR2------
  if (CAN_command->mission2 == MISSION_RESET){
    stopAndReset2();
    CAN_sendData->status2 = 0;

  }else if (CAN_command->mission2 == MISSION_RECEIVE){
    receivedRack2(CAN_command->speed2);
    // - Status
    if (step_receivedRack2 == 0){
      CAN_sendData->status2 = 0;

    }else if (step_receivedRack2 == 1 || step_receivedRack2 == 2 || step_receivedRack2 == 3 || step_receivedRack2 == 4 || step_receivedRack2 == 6){
      CAN_sendData->status2 = 1;

    }else if (step_receivedRack2 == 5){
      CAN_sendData->status2 = 3;

    }else if (step_receivedRack2 == 7){
      CAN_sendData->status2 = -1;

    }else{
      CAN_sendData->status2 = -3;
    }

  }else if (CAN_command->mission2 == MISSION_TRANSMISSION){
    transmitRack2(CAN_command->speed2);
    // - Status
    if (step_transmitRack2 == 0){
      CAN_sendData->status2 = 0;

    }else if (step_transmitRack2 == 1 || step_transmitRack2 == 2 || step_transmitRack2 == 3 || step_transmitRack2 == 4 || step_transmitRack2 == 6){
      CAN_sendData->status2 = 2;

    }else if (step_transmitRack2 == 5){
      CAN_sendData->status2 = 4;

    }else if (step_transmitRack2 == 7){
      CAN_sendData->status2 = -2;

    }else{
      CAN_sendData->status2 = -4;
    }

  }else{
    stopAndReset2();
    CAN_sendData->status2 = 0;
  }

  // -- 
  OC_CAN_send();
  
}

void OC_controller::debug(){

}