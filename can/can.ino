#include <Arduino.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <mcp2515.h>
#include <stdio.h>
#include <string.h>
#include "RF24.h"
#include "SoftReset.h"

#define slave1 0x205
#define slave2 0x114
#define slave3 0x112
#define coolantsensorDivider 4700
const float steinconstA = 0.001092768344245138;       
const float steinconstB = 0.000181450723833218;      
const float steinconstC = 0.000000222855858126706000; 

RF24 radio(9, 10); //CE, SS
MCP2515 mcp2515(8);   //cs핀 번호 설정
const byte CLK = 3;   // define CLK pin (any digital pin)
const byte DIO = 4;   // define DIO pin (any digital pin)
uint8_t address[6] = "04171";
byte data[8] = {0x00 ,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
const float Diameter = 325.0;  //mm단위
const uint8_t SEG_DONE[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};
const uint8_t SEG_ERR1[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_E | SEG_G,                                    // R
  SEG_E | SEG_G,                                    // R
  SEG_B | SEG_C                                     // 1
};
const uint8_t SEG_ERR2[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_E | SEG_G,                                    // R
  SEG_E | SEG_G,                                    // R
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G             // 2
};
const uint8_t SEG_ERR3[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_E | SEG_G,                                    // R
  SEG_E | SEG_G,                                    // R
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_G             // 3
};
const uint8_t SEG_RECD[] = {
  SEG_E | SEG_G,                                    // R
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_A | SEG_D | SEG_E | SEG_F,                    // C
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G             // d
};
TM1637Display display(CLK, DIO);// define dispaly object
long RPM=0;
int lin_vel=0;
float sen11,sen22,sen33;
uint16_t RPM_1,RPM_2,torque_buff,temp_buff,motor_vol,motor_curr, batt_vol,batt_curr,torque_demand,throttle_input_vol;
uint8_t heatsink_temp;

void setup() {
  display.setBrightness(0x0f);
  display.showNumberDec(9999);
  Serial.begin(9600);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  radio.begin(); //아두이노-RF모듈간 통신라인
  radio.setPALevel(RF24_PA_MAX); 
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);
  radio.stopListening(); //기본 :  송신모드

  /////////////////////////////////////////Slv Start 0000////////////////////////////////////////
  while(1){
    if(radio.begin()){
      int i1;
      radio.startListening();  
      radio.read(&i1,sizeof(i1));
      //Serial.println(i1);
      delay(10);
        if(i1 == 257){
          Serial.println("Slv started");
          display.showNumberDec(000);
          delay(300);
          break;
        }
        if (i1 == 10280){ 
          soft_restart();
        }
     }
  }

  /////////////////////////////////////////Radio check 1111//////////////////////////////////////
  for(int i=0;i<30;i++){
    if (radio.begin()){                        
      display.showNumberDec(1111);
      int c1 = 1;
      radio.stopListening();
      radio.write(&c1,sizeof(c1));
      //Serial.println(c1);
      delay(10);
    }
    else{
      display.setSegments(SEG_ERR1);
      radio.stopListening();
      int c2 = 2;
      radio.write(&c2,sizeof(c2));
      //Serial.println(c2);
      delay(10);
    }
  }

  ///////////////////////////////////analogpin input check 2222//////////////////////////////////
  for(int i=0;i<30;i++){
    if(analogRead(A0)<1000&&analogRead(A1)<1000&&analogRead(A2)<1000){ 
      display.showNumberDec(2222);
      int c3 = 3;
      radio.stopListening();
      radio.write(&c3,sizeof(c3));
      //Serial.println(c3);
      delay(10);
    }
    else{
      display.setSegments(SEG_ERR2);
      int c4 = 4;
      radio.stopListening();
      radio.write(&c4,sizeof(c4));
      //Serial.println(c4);
      delay(10);
    }
  }

  /////////////////////////////////////////Can Trans check 3333//////////////////////////////////
  for(int i=0;i<30;i++){
    if(digitalRead(2)==LOW||digitalRead(2)==HIGH){ 
      display.showNumberDec(3333);
      int c5= 5;
      radio.stopListening();
      radio.write(&c5,sizeof(c5));
      //Serial.println(c5);
      delay(10);
    }
    else{
      display.setSegments(SEG_ERR3);
      int c6 = 6;
      radio.stopListening();
      radio.write(&c6,sizeof(c6));
      //Serial.println(c6);
      delay(10);
    }
  }

  /////////////////////////////////////////Record OR Reset///////////////////////////////////////
  while(1){
    if(radio.available()){
      int i2;
      radio.startListening();  
      radio.read(&i2,sizeof(i2));
      delay(10);
        if(i2 == 2570){
          Serial.println("Start recording");
          display.setSegments(SEG_RECD);
          delay(500);
          break;
        }
        if (i2 == 10280){ 
          soft_restart();
        }
     }
  } 
}



void loop() {
  temp(analogRead(A0),analogRead(A1),analogRead(A2));

  /////////////////////////////////////////Request slave1//////////////////////////////////////
  mcp2515_send(slave1,data);

  //Response slave1
  byte recv[8];
  unsigned int id = mcp2515_receive(recv);
  if(id == -1){
    Serial.println("슬레이브1 오프라인!");
  }else{
    RPM_1 = ((uint16_t)recv[1] << 8) | recv[0];
	  RPM_2 = ((uint16_t)recv[3]<< 8) | recv[2];
	  RPM = ((long)RPM_2 << 16) | RPM_1; // Motor RPM data 4bytes
	  torque_buff = ((uint16_t)recv[5] << 8) | recv[4];// Motor Torque data 2bytes
	  temp_buff = ((uint16_t)recv[7] << 8) | recv[6];// Motor Temperature 2bytes
    lin_vel = (Diameter) * RPM * PI * 60/1000000;   //KM/H
    display.showNumberDec(lin_vel);
  }
  
  /////////////////////////////////////////Request slave2//////////////////////////////////////
  mcp2515_send(slave2,data);

  //Response slave2
  id = mcp2515_receive(recv);
  if(id == -1){
    Serial.println("슬레이브2 오프라인!");
  }else{
    motor_vol= ((uint16_t)recv[1] << 8) | recv[0]; //Motor_vol data 2bytes
	  motor_curr= ((uint16_t)recv[3] << 8) | recv[2]; //Motor_curr data 2bytes
	  batt_vol= ((uint16_t)recv[5] << 8) | recv[4]; //Batt vol data 2bytes
	  batt_curr= ((uint16_t)recv[7] << 8) | recv[6]; //Batt_curr data 2bytes
  }

  /////////////////////////////////////////Request slave3//////////////////////////////////////
  mcp2515_send(slave3,data);

  //Response slave3
  id = mcp2515_receive(recv);
  if(id == -1){
    Serial.println("슬레이브3 오프라인!");
  }else{
		torque_demand= ((uint16_t)recv[1]<< 8) | recv[0]; //torque_demand data 2bytes
		throttle_input_vol= ((uint16_t)recv[3] << 8) | recv[2]; //throttle_input_vol data 2bytes
		heatsink_temp= (uint8_t)recv[4]; // heatsink_temp data 1bytes
  }


  /////////////////////////////////////////Datapack RF Trans//////////////////////////////////////
  float datapack[14];
  datapack[0]=sen11; datapack[1]=sen22; datapack[2]=sen33; datapack[3]=RPM;
  datapack[4]=lin_vel; datapack[5]=temp_buff; datapack[6]=heatsink_temp; datapack[7]= torque_buff;
  datapack[8]= torque_demand; datapack[9]= motor_vol;datapack[10]= motor_curr; 
  datapack[11]= batt_vol; datapack[12]= batt_curr; datapack[13]= throttle_input_vol;
  radio.stopListening(); //송신모드
  radio.write(datapack,sizeof(datapack));


  ///////////////////////////////////////////////Reset/////////////////////////////////////////////
  radio.startListening();
  if(radio.available()){
  int i3;
    radio.read(&i3,sizeof(i3));
    if (i3 == 40){ 
      soft_restart();
    }
  }
  delay(200);
}


void mcp2515_send(unsigned int id, byte data[]){
  struct can_frame canMsg;
  canMsg.can_id  = id; //슬레이브의 ID
  canMsg.can_dlc = 8;
  for(int i =0;i<8;i++){
    canMsg.data[i] = data[i];
  }
  mcp2515.sendMessage(&canMsg);
  // Serial.println("[모터 컨트롤러에 보낸 메시지]");
  // Serial.println(id,HEX);
  // for (int i = 0; i<canMsg.can_dlc; i++)  {
  //     Serial.print(canMsg.data[i],HEX);
  //     Serial.print(" ");
  // }
  // Serial.println();
}

unsigned int mcp2515_receive(byte recv[]){
  struct can_frame canMsg;
  unsigned long t = millis();
  while(1){
    if(millis() - t > 3000){
      return -1;
      break;
    }
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      for(int i =0;i<8;i++){
        recv[i] = canMsg.data[i];
      }
      return canMsg.can_id;
      break;
    }
  }
}

void temp(float sensor_value1,float sensor_value2,float sensor_value3){
  sensor_value1 = (coolantsensorDivider*sensor_value1)/(1023-sensor_value1);
  sensor_value2 = (coolantsensorDivider*sensor_value2)/(1023-sensor_value2);
  sensor_value3 = (coolantsensorDivider*sensor_value3)/(1023-sensor_value3);  
  float sen1,sen2,sen3;                         
  sen1 = log(sensor_value1);                   
  sen1 = pow(sen1,3);                 
  sen1 *= steinconstC;                     
  sen1 += (steinconstB*(log(sensor_value1)));    
  sen1 += steinconstA;                     
  sen1 = 1.0/sen1;                  
  sen1 -= 273.15;   

  sen2 = log(sensor_value2);                   
  sen2 = pow(sen2,3);                 
  sen2 *= steinconstC;                     
  sen2 += (steinconstB*(log(sensor_value2)));    
  sen2 += steinconstA;                     
  sen2 = 1.0/sen2;                  
  sen2 -= 273.15; 

  sen3 = log(sensor_value3);                   
  sen3 = pow(sen3,3);                 
  sen3 *= steinconstC;                     
  sen3 += (steinconstB*(log(sensor_value3)));    
  sen3 += steinconstA;                     
  sen3 = 1.0/sen3;                  
  sen3 -= 273.15;
  sen11=sen1;
  sen22=sen2;
  sen33=sen3;
}
