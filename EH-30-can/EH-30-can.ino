#include <Arduino.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <mcp2515.h>
#include <stdio.h>
#include <string.h>
#include "RF24.h"

#define slave1 0x205
#define slave2 0x114
#define slave3 0x112
#define coolantsensorDivider 4700
const float steinconstA = 0.001092768344245138;       
const float steinconstB = 0.000181450723833218;      
const float steinconstC = 0.000000222855858126706000; 


RF24 radio(9, 8); //CE, CS
uint8_t address[6] = "41715";

MCP2515 mcp2515(10);   //cs핀 번호 설정
byte data[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte recv1[8], recv2[8], recv3[8];
const float Diameter = 525.0;  //mm단위
long RPM=0;
int lin_vel=0;
uint16_t RPM_1,RPM_2,motor_temp,motor_vol,batt_vol,throttle_input_vol;
int16_t torque_buff,motor_curr,batt_curr,torque_demand;
uint8_t heatsink_temp;
float sen11,sen22,sen33;
void temp(float sensor_value1,float sensor_value2,float sensor_value3);
void mcp2515_send(unsigned int id, byte data[]);
unsigned int mcp2515_receive1(byte recv[]);
unsigned int mcp2515_receive2(byte recv[]);
unsigned int mcp2515_receive3(byte recv[]);


volatile int flow_frequency; // Measures flow sensor pulses
unsigned int l_min; // Calculated litres/min
unsigned char flowsensor = 2; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;


const int aena= 5;
const int adir1=6;
const int adir2=7;

// const byte CLK = 3;   // define CLK pin (any digital pin)
// const byte DIO = 4;   // define DIO pin (any digital pin)
// TM1637Display display(CLK, DIO);// define dispaly object



void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
  
  radio.begin(); //아두이노-RF모듈간 통신라인
  radio.setPALevel(RF24_PA_MAX); 
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.stopListening(); //기본 :  송신모드
  
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  attachInterrupt(0, flow, RISING); // Setup Interrupt
  sei(); // Enable interrupts
  currentTime = millis();
  cloopTime = currentTime;


  
  pinMode(adir1,OUTPUT);    //l298n 모터 a 방향 결정 
  pinMode(adir2,OUTPUT);    //l298n 모터 a 방향 결정 
  delay(10);
}


void loop() {
  currentTime = millis(); // Every 0.5 second, calculate and print litres/min
  if(currentTime >= (cloopTime + 500)){
    cloopTime = currentTime; // Updates cloopTime
    l_min = (2*flow_frequency / 11); // Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
    flow_frequency = 0; // Reset Counter
    Serial.print(l_min, DEC); // Print litres/min
    Serial.println(" L/Min");
  }


  digitalWrite(adir1,1);      // 디지털 6번 전진
  digitalWrite(adir2,0);      // 디지털 7번 전진
  analogWrite(aena,255);     //디지털 5번 핀이 a 모터 속도 조절 핀  0~255


  //temp(analogRead(A0),analogRead(A1),analogRead(A2));
  digitalWrite(8,HIGH);     //RF24 비활성화
  digitalWrite(10,LOW);     //CAN 통신 활성화
  delay(10);

  
  mcp2515_send(slave1,data); //Request slave1
  unsigned int id = mcp2515_receive1(recv1); //Response slave1
  if(id == -1){
    Serial.println("슬레이브1 오프라인!");
  }else{
    RPM_1 = ((uint16_t)recv1[1] << 8) | recv1[0];
    RPM_2 = ((uint16_t)recv1[3]<< 8) | recv1[2];
    RPM = ((long)RPM_2 << 16) | RPM_1; // Motor RPM data 4bytes
    torque_buff = ((int16_t)recv1[5] << 8) | recv1[4];// Motor Torque data 2bytes
    motor_temp = ((uint16_t)recv1[7] << 8) | recv1[6];// Motor Temperature 2bytes
    lin_vel = (Diameter) * RPM * PI * 60/1000000/5.5  //KM/H  기어비 5.5 
    //display.showNumberDec(RPM);
  }delay(10);
  
  
  mcp2515_send(slave2,data); //Request slave2
  id = mcp2515_receive2(recv2);//Response slave2
  if(id == -1){
    Serial.println("슬레이브2 오프라인!");
  }else{
    motor_vol= ((uint16_t)recv2[1] << 8) | recv2[0]; //Motor_vol data 2bytes
    motor_curr= ((int16_t)recv2[3] << 8) | recv2[2]; //Motor_curr data 2bytes
    batt_vol= ((uint16_t)recv2[5] << 8) | recv2[4]; //Batt vol data 2bytes
    batt_curr= ((int16_t)recv2[7] << 8) | recv2[6]; //Batt_curr data 2bytes
  }delay(10);


  mcp2515_send(slave3,data); //Request slave3
  id = mcp2515_receive3(recv3);  //Response slave3
  if(id == -1){
    Serial.println("슬레이브3 오프라인!");
  }else{
      torque_demand= ((int16_t)recv3[1]<< 8) | recv3[0]; //torque_demand data 2bytes
      throttle_input_vol= ((uint16_t)recv3[3] << 8) | recv3[2]; //throttle_input_vol data 2bytes
      heatsink_temp= (uint8_t)recv3[4]; // heatsink_temp data 1bytes
  }delay(10);

  digitalWrite(8,LOW);     //RF24 활성화
  digitalWrite(10,HIGH);     //CAN 통신 비활성화

  //Datapack RF Trans
  float datapack[8],datapack1[8];
  datapack[0]=4;
  datapack[1]=sen11; datapack[2]=sen22; datapack[3]=sen33; datapack[4]=RPM;
  datapack[5]=lin_vel; datapack[6]=motor_temp; datapack[7]=heatsink_temp; 
  datapack1[0]=8;
  datapack1[1]= torque_buff*0.0625; datapack1[2]= torque_demand*0.0625; datapack1[3]= motor_vol*0.0625;
  datapack1[4]= motor_curr*0.0625; datapack1[5]= batt_vol*0.0625; datapack1[6]= batt_curr*0.0625; 
  datapack1[7]= throttle_input_vol*0.00390625;

  radio.stopListening(); //송신모드
  radio.write(datapack,sizeof(datapack));
  radio.write(datapack1,sizeof(datapack1));
  
//  for(int i =0;i<7;i++){
//    Serial.print(datapack[i],HEX);
//    Serial.print(" ");}
//  for(int i =0;i<7;i++){
//    Serial.print(datapack1[i],HEX);
//    Serial.print(" ");
//    }
//    Serial.println("");
    delay(10);
}

//////////////////////////////////////////////////////send/////////////////////////////////////////
void mcp2515_send(unsigned int id, byte data[]){
  struct can_frame canMsg;
  canMsg.can_id  = id; //슬레이브의 ID
  canMsg.can_dlc = 8;
  for(int i =0;i<8;i++){
    canMsg.data[i] = data[i];
  }
  mcp2515.sendMessage(&canMsg);
   //Serial.println("[모터 컨트롤러에 보낸 메시지]");
   Serial.println(id,HEX);
   for (int i = 0; i<canMsg.can_dlc; i++)  {
    Serial.print(canMsg.data[i],HEX);
    Serial.print(" ");
   }
   Serial.println();
}

///////////////////////////////////////////////////receive1/////////////////////////////////////////
unsigned int mcp2515_receive1(byte recv[]){
  struct can_frame canMsg;
  unsigned long t = millis();
  while(1){
    if(millis() - t > 2000){
      return -1;
      break;
    }
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK && canMsg.can_id == slave1) {
      for(int i =0;i<8;i++){
        recv[i] = canMsg.data[i];
        Serial.print(recv[i],HEX);
        Serial.print(" ");
      }
      Serial.println();
      return canMsg.can_id;
      break;
    }
  }
}

///////////////////////////////////////////////////receive2/////////////////////////////////////////
unsigned int mcp2515_receive2(byte recv[]){
  struct can_frame canMsg;
  unsigned long t = millis();
  while(1){
    if(millis() - t > 2000){
      return -1;
      break;
    }
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK&& canMsg.can_id == slave2) {
      for(int i =0;i<8;i++){
        recv[i] = canMsg.data[i];
        Serial.print(recv[i],HEX);
        Serial.print(" ");
      }
      Serial.println();
      return canMsg.can_id;
      break;
    }
  }
}

/////////////////////////////////////////////////receive3/////////////////////////////////////////
unsigned int mcp2515_receive3(byte recv[]){
  struct can_frame canMsg;
  unsigned long t = millis();
  while(1){
    if(millis() - t > 2000){
      return -1;
      break;
    }
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK&& canMsg.can_id == slave3) {
      for(int i =0;i<8;i++){
        recv[i] = canMsg.data[i];
        Serial.print(recv[i],HEX);
        Serial.print(" ");
      }
      Serial.println();
      return canMsg.can_id;
      break;
    }
  }
}

//////////////////////////////////////////////////temp_sensor////////////////////////////////////////////////
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
  Serial.print(sen11);Serial.print(" ");Serial.print(sen22);Serial.print(" ");Serial.println(sen33);
}

// Interrupt function
void flow () 
{
flow_frequency++;
}
