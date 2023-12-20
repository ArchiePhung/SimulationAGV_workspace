// 18,19,23,25 out
// in 32 - 35

#include <StopWatch.h>
#include "PCF8575.h"
// Set i2c address
PCF8575 pcf8575(0x20);

StopWatch sw_millis;

#define INPUT1      33
#define INPUT2      32
#define INPUT3      35
#define INPUT4      34


#define OUTPUT1      18
#define OUTPUT2      19
#define OUTPUT3      23
#define OUTPUT4      25



#define OUT_LED1      13
#define OUT_LED2      26
#define OUT_LED3      27
#define OUT_LED4      14


void setup() {
  Serial.begin(115200);
  Serial.println("hello co ba ");
  // put your setup code here, to run once:
  pinMode(OUTPUT1, OUTPUT);
  pinMode(OUTPUT2, OUTPUT);
  pinMode(OUTPUT3, OUTPUT);
  pinMode(OUTPUT4, OUTPUT);

  pinMode(OUT_LED1, OUTPUT);
  pinMode(OUT_LED2, OUTPUT);
  pinMode(OUT_LED3, OUTPUT);
  pinMode(OUT_LED4, OUTPUT);


  pcf8575.pinMode(P0, OUTPUT);
  pcf8575.pinMode(P1, OUTPUT);
  pcf8575.pinMode(P2, OUTPUT);
  pcf8575.pinMode(P3, OUTPUT);
  pcf8575.pinMode(P4, OUTPUT);
  pcf8575.pinMode(P5, OUTPUT);
  pcf8575.pinMode(P6, OUTPUT);
  pcf8575.pinMode(P7, OUTPUT);

  pinMode(INPUT1, INPUT); // CẤM PULLUP
  pinMode(INPUT2, INPUT);
  pinMode(INPUT3, INPUT);
  pinMode(INPUT4, INPUT);

  pcf8575.pinMode(P8, INPUT);
  pcf8575.pinMode(P9, INPUT);
  pcf8575.pinMode(P10, INPUT);
  pcf8575.pinMode(P11, INPUT);
  pcf8575.pinMode(P12, INPUT);
  pcf8575.pinMode(P13, INPUT);
  pcf8575.pinMode(P14, INPUT);
  pcf8575.pinMode(P15, INPUT);
  pcf8575.begin();
  sw_millis.start();// khởi tạo timer
}

void loop() {

  if (sw_millis.elapsed() >= 500) // TIMER 1ms
  {

    PCF8575::DigitalInput di = pcf8575.digitalReadAll();
    digitalWrite(OUTPUT1, 1 - digitalRead(OUTPUT1));
    digitalWrite(OUTPUT2, 1 - digitalRead(OUTPUT2));
    digitalWrite(OUTPUT3, 1 - digitalRead(OUTPUT3));
    digitalWrite(OUTPUT4, 1 - digitalRead(OUTPUT4));
    digitalWrite(OUT_LED1, 1 - digitalRead(OUT_LED1));
    digitalWrite(OUT_LED2, 1 - digitalRead(OUT_LED2));
    digitalWrite(OUT_LED3, 1 - digitalRead(OUT_LED3));
    digitalWrite(OUT_LED4, 1 - digitalRead(OUT_LED4));
    pcf8575.digitalWrite(P1, 1 - pcf8575.digitalRead(P1));
    pcf8575.digitalWrite(P0, 1 - pcf8575.digitalRead(P0));
    pcf8575.digitalWrite(P2, 1 - pcf8575.digitalRead(P2));
    pcf8575.digitalWrite(P3, 1 - pcf8575.digitalRead(P3));
    pcf8575.digitalWrite(P4, 1 - pcf8575.digitalRead(P4));
    pcf8575.digitalWrite(P5, 1 - pcf8575.digitalRead(P5));
    pcf8575.digitalWrite(P6, 1 - pcf8575.digitalRead(P6));
    pcf8575.digitalWrite(P7, 1 - pcf8575.digitalRead(P7));

    Serial.print("INPUT1 : "); Serial.print(digitalRead(INPUT1)); Serial.print(" - ");
    Serial.print("INPUT2 : "); Serial.print(digitalRead(INPUT2)); Serial.print(" - ");
    Serial.print("INPUT3 : "); Serial.print(digitalRead(INPUT3)); Serial.print(" - ");
    Serial.print("INPUT4 : "); Serial.println(digitalRead(INPUT4));
    Serial.print("READ VALUE FROM PCF P1: ");
    Serial.print(di.p8);
    Serial.print(" - ");
    Serial.print(di.p9);
    Serial.print(" - ");
    Serial.print(di.p10);
    Serial.print(" - ");
    Serial.print(di.p11);
    Serial.print(" - ");
    Serial.print(di.p12);
    Serial.print(" - ");
    Serial.print(di.p13);
    Serial.print(" - ");
    Serial.print(di.p14);
    Serial.print(" - ");
    Serial.println(di.p15);




    sw_millis.reset();
    sw_millis.start();
  }

}
