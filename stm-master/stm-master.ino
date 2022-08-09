#include <SPI.h> 
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>
//#include "SoftReset.h"
RF24 radio(9, 10); // in SPI bus to set nRF24L01 radio 

const uint64_t address = 0xE8E8F0F0E1LL;
struct data{
  int rpm,lin_vel;
  float motor_torq,motor_torque_demand;
  uint16_t motor_temp,motor_vol,batt_vol,motor_curr,batt_curr,throttle_input_vol;
  uint8_t heatsink_temp;
};

struct data a;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);//you can set RF24_PA_MIN / RF24_PA_LOW / RF24_PA_HIGH / RF24_PA_MAX
  radio.setAutoAck(false);
  radio.setPayloadSize(20); // setting the payload size to the needed value
  radio.setDataRate(RF24_2MBPS); 
  radio.setChannel(80);
  radio.openReadingPipe(1,address);
  radio.startListening(); //Setting Module to Listener

  
}
void loop() {//Read ECU Data and print to use Node.js
  if (radio.available()){ 
//  radio.read((void*)(&a),sizeof(a));
//  Serial.print(a.rpm);
//  Serial.print(" ");
//  Serial.print(a.lin_vel);
//  Serial.print(" ");
//  Serial.print(a.motor_temp);
//  Serial.print(" ");
//  Serial.print(a.heatsink_temp);
//  Serial.print(" ");
//  Serial.print(a.motor_torq);
//  Serial.print(" ");
//  Serial.print(a.motor_torque_demand);
//  Serial.print(" ");
//  Serial.print(a.motor_vol);
//  Serial.print(" ");
//  Serial.print(a.motor_curr);
//  Serial.print(" ");
//  Serial.print(a.batt_vol);
//  Serial.print(" ");
//  Serial.print(a.batt_curr);
//  Serial.print(" ");
//  Serial.println(a.throttle_input_vol);
  int aa;
  radio.read(&a,sizeof(a));
  Serial.println(aa);
}
Serial.println("asdf");
delay(100);
}
