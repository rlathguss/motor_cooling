#include <SPI.h> 
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>
RF24 radio(9, 6); // in SPI bus to set nRF24L01 radio 

const uint64_t address = 0xE8E8F0F0E1LL;// Pipeline
struct data{
	int rpm;
	float motor_torq,motor_torque_demand;
	uint16_t motor_temp,motor_vol,batt_vol,motor_curr,batt_curr,throttle_input_vol;
  uint8_t heatsink_temp;
};
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX); // To set about voltage level. If module is close you should set at least
  radio.setAutoAck(false);
  radio.openReadingPipe(1,address);//you can set RF24_PA_MIN / RF24_PA_LOW / RF24_PA_HIGH / RF24_PA_MAX
  //for your module distance
  //For MAX level to operate you should use by-pass capacitor between 3.3V and GND
  radio.startListening(); //Setting Module to Listener
  Serial.println("RPM | Motor_temp | heatsink_temp | Motor_torque | Motor_torque_demand | Motor_vol | Motor_curr | Batt_vol | batt_curr | throttle_input_vol");
}
void loop() {//Read ECU Data and print to use Node.js
  if (radio.available()){ 
  radio.read(data,30);
  Serial.print(data.RPM);
  Serial.print(" ");
  Serial.print(data.motor_temp);
  Serial.print(" ");
  Serial.print(data.heatsink_temp);
  Serial.print(" ");
  Serial.print(data.motor_torq);
  Serial.print(" ");
  Serial.print(data.motor_torque_demand);
  Serial.print(" ");
  Serial.print(data.motor_vol);
  Serial.print(" ");
  Serial.print(data.motor_curr);
  Serial.print(" ");
  Serial.print(data.batt_vol);
  Serial.print(" ");
  Serial.print(data.batt_curr);
  Serial.print(" ");
  Serial.print(data.throttle_input_vol);
  Serial.println(" ");
}
delay(100);
}
