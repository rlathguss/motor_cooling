#include <SPI.h> 
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>
RF24 radio(9, 6); // in SPI bus to set nRF24L01 radio 

const uint64_t address = 0xE8E8F0F0E1LL;// Pipeline
struct data{
	int rpm,lin_vel;
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

  String p1;
  Serial.println("Start slv?");
  while(1){
    p1 = Serial.readString();
    if(p1.indexOf("yes") != -1){
      for(int i=0;i<30;i++){
        radio.stopListening(); //송신모드
        int i1 = 1;
        radio.write(&i1,sizeof(i1));
      }
      break;
    }
    if(p1.indexOf("reset") != -1){
    Serial.println("Restart the slv board ");
      for(int i=0;i<30;i++){
        int i3 = 40;
        radio.stopListening();  
        radio.write(&i3,sizeof(i3));
        delay(10);
      }
    soft_restart();
    }
  }

  while(1){
    if(radio.begin()){
      int c1;
      radio.startListening();
      radio.read(&c1,sizeof(c1));
      //Serial.println(c1);
      delay(10);
        if (c1==257){ 
          Serial.println("radio_connected");  
          delay(500);              // ----1111-------디스플레이 출력 양호
          break;
        }
    }
    else{   
      Serial.println("ERR_Radio");            
      delay(500);
      break;
    }
  }

  while(1){
    if(radio.begin()){
      int c2;
      radio.startListening();
      radio.read(&c2,sizeof(c2));
      //Serial.println(c2);
      delay(10);
        if (c2==771){   
          Serial.println("Analogsensor_connected");            //----2222-------아날로그 신호 양호
          delay(500);
          break;
        }
        if (c2==1028){   
          Serial.println("ERR_Analogsensor");            
          delay(500);
          break;
        }
    }
  }

  String p2;
  Serial.println("Start recording? ");
  while(1){
    p2 = Serial.readString();
    if(p2.indexOf("yes") != -1){
      for(int i=0;i<30;i++){
        radio.stopListening(); //송신모드
        int i2 = 10;
        radio.write(&i2,sizeof(i2));
      }
      Serial.println("RPM | lin_vel | Motor_temp | heatsink_temp | Motor_torque | Motor_torque_demand | Motor_vol | Motor_curr | Batt_vol | batt_curr | throttle_input_vol"); // legend
      break;
    }
    if(p1.indexOf("reset") != -1){
      Serial.println("Restart the slv board ");
      for(int i=0;i<30;i++){
        int i3 = 40;
        radio.stopListening();  
        radio.write(&i3,sizeof(i3));
        delay(10);
      }
    soft_restart();
    }
  }

  
}
void loop() {//Read ECU Data and print to use Node.js
  if (radio.available()){ 
  radio.read(data,sizeof(data));
  Serial.print(data.RPM);
  Serial.print(" ");
  Serial.print(data.lin_vel);
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
