#include <Arduino.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <mcp2515.h>
#include <stdio.h>
#include <string.h>
#include "RF24.h"

#define rx 0x201
#define tx 0x181

RF24 radio(9, 8); //CE, CS
uint8_t address[6] = "41715";

MCP2515 mcp2515(10);   //cs핀 번호 설정
byte data[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
byte recv[8];
unsigned int id;
const float Diameter = 525.0;  //mm단위
int16_t SpeedRpmMax = 6000, CurrentDevice = 2000, Current200PC = 1070, I_Max_PK = 270;   //0x59, 0xC6, 0xD9, 0xC4
int16_t SpeedActual, RPM, CurrentActual, Current, Motor_Temp, PowerStage_Temp, Air_Temp, M_Out, M_Set, Torque_Out, Torque_Set;

int16_t torque_demand;



void mcp2515_send(unsigned int id, byte data[], unsigned int dlc);
unsigned int mcp2515_receive(byte recv[]);

bool bSpeedActual = false;
bool bCurrentActual = false;
bool bMoter_Temp = false;
bool bPowerStage_Temp = false;
bool bAir_Temp = false;
bool bM_Out = false;
bool bM_Set = false;


volatile int flow_frequency; // Measures flow sensor pulses
uint16_t LPM; // Calculated litres/min
unsigned char flowsensor = 2; // Sensor Input
unsigned long currentTime1;
unsigned long cloopTime1;

unsigned long currentTime2;
unsigned long cloopTime2;
const int aena = 5;
const int adir1 = 6;
const int adir2 = 7;

// const byte CLK = 3;   // define CLK pin (any digital pin)
// const byte DIO = 4;   // define DIO pin (any digital pin)
// TM1637Display display(CLK, DIO);// define dispaly object



void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
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
  currentTime1 = millis();
  cloopTime1 = currentTime1;


  pinMode(adir1, OUTPUT);    //l298n 모터 a 방향 결정 
  pinMode(adir2, OUTPUT);    //l298n 모터 a 방향 결정 
  currentTime2 = millis();
  cloopTime2 = currentTime2;
  digitalWrite(adir1, 1);                           // 디지털 6번 전진
  digitalWrite(adir2, 0);                           // 디지털 7번 전진


  digitalWrite(8, HIGH);     //RF24 비활성화
  digitalWrite(10, LOW);     //CAN 통신 활성화
  delay(10);
  
  byte data1[3] = { 0x3D,0x30,0x32 };
  mcp2515_send(rx, data1, 3); //Request SpeedActual 50ms
  data1[3] = { 0x3D,0x5F,0x32 };
  mcp2515_send(rx, data1, 3); //Request CurrentActual 50ms
  
  data1[3] = { 0x3D,0x49,0x32 };
  mcp2515_send(rx, data1, 3); //Request Moter_Temp 50ms
  data1[3] = { 0x3D,0x4A,0x32 };
  mcp2515_send(rx, data1, 3); //Request PowerStage_Temp 50ms
  data1[3] = { 0x3D,0x4B,0x32 };
  mcp2515_send(rx, data1, 3); //Request Air_Temp 50ms
  
  data1[3] = { 0x3D,0xA0,0x32 };
  mcp2515_send(rx, data1, 3); //Request M_out 50ms
  data1[3] = { 0x3D,0x90,0x32 };
  mcp2515_send(rx, data1, 3); //Request M_set 50ms
}


void loop() {
  currentTime1 = millis();                         // Every 0.5 second, calculate and print litres/min
  if (currentTime1 >= (cloopTime1 + 500)) {
    
    cloopTime1 = currentTime1;                     // Updates cloopTime
    LPM = (2 * flow_frequency / 11);             // Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
    flow_frequency = 0;                            // Reset Counter
    Serial.print(LPM, DEC);                      // Print litres/min
    Serial.println(" L/Min");
  }


  currentTime2 = millis();                         // Every 2 second
  if (currentTime2 >= (cloopTime2 + 2000)) {
    cloopTime2 = currentTime2;
    if(id == -1){
      analogWrite(aena, 255);                                   //디지털 5번 핀이 a 모터 속도 조절 핀  0~255
    }
    else if (Moter_Temp > 40 || PowerStage_Temp > 40) {
      analogWrite(aena, 255);
    }
    else if (Moter_Temp > 35 || PowerStage_Temp > 35) {
      analogWrite(aena, 200);
    }
    else if (Moter_Temp > 30 || PowerStage_Temp > 30) {
      analogWrite(aena, 150);
    }
    else if (Moter_Temp > 25 || PowerStage_Temp > 25) {
      analogWrite(aena, 100);                        
    }
    else if (Moter_Temp > 20 || PowerStage_Temp > 20) {
      analogWrite(aena, 50);                        
    }
    else{
      analogWrite(aena, 0);  
    }
  }
  

  for (;;) {
    id = mcp2515_receive(recv);
    if (id == -1) { 
      Serial.println("컨트롤러 오프라인! / 팬속도 255 ");
    }
    else if (id == 0x30) {                                          // SpeedActual
      if (bSpeedActual == true){  
        Serial.println("SpeedActual 중복");
      }
      else{
        SpeedActual  = ((uint16_t)recv[2] << 8) | recv[1];
        RPM = SpeedRpmMax * (SpeedActual / 32767);
        bSpeedActual == true;        
        //display.showNumberDec(RPM);
      }
    }
    else if (id == 0x5F) {                                          // CurrentActual
      if (bCurrentActual == true){
        Serial.println("CurrentActual 중복");
      }
      else{
        CurrentActual = convertToInt(recv, canMsg.can_dlc);
        Current = 2 / 10 * CurrentDevice * (CurrentActual / Current200PC);        
        bCurrentActual = true;
      }
    }


    else if (id == 0x49) {                                          // Moter_Temp
      if (bMoter_Temp == true){
        Serial.println("Moter_Temp 중복");
      }
      else{
        Moter_Temp = ((uint16_t)recv[2] << 8) | recv[1];
        bMoter_Temp = true;
      }
    }
    else if (id == 0x4A) {                                          // PowerStage_Temp
      if (bPowerStage_Temp == true){
        Serial.println("PowerStage_Temp 중복");
      }
      else{
        PowerStage_Temp = ((uint16_t)recv[2] << 8) | recv[1];
        bPowerStage_Temp = true;
      }
    }
    else if (id == 0x4B) {                                          //Air_Temp
      if (bAir_Temp == true){
        Serial.println("Air_Temp 중복");
      }
      else{
        Air_Temp = ((uint16_t)recv[2] << 8) | recv[1];
        bAir_Temp = true;
      }
    }


    else if (id == 0xA0) {                                          //M_Out
      if (bM_Out == true){
        Serial.println("M_Out 중복");
      }
      else{
        M_Out = ((uint16_t)recv[2] << 8) | recv[1];
        Torque_Out = M_Out / 32767 * I_Max_PK / 1.4142 * 0.61;
        bM_Out = true;
      }
    }
    else if (id == 0x90) {                                          //M_Set
      if (bM_Set == true){
        Serial.println("M_Set 중복");
      }
      else{
        M_Set = ((uint16_t)recv[2] << 8) | recv[1];
        Torque_Set = M_Set / 32767 * I_Max_PK / 1.4142 * 0.61; 
        bM_Set = true;
      }
    }

    if (bSpeedActual == true && bCurrentActual == true && bMoter_Temp == true && bPowerStage_Temp == true && bAir_Temp == true && bM_Out == true && bM_Set == true){
      break;
    }
  }


  delay(10);
  digitalWrite(8, LOW);       //RF24 활성화
  digitalWrite(10, HIGH);     //CAN 통신 비활성화

  bool bSpeedActual = false;
  bool bCurrentActual = false;
  bool bMoter_Temp = false;
  bool bPowerStage_Temp= false;
  bool bAir_Temp = false;
  bool bM_Out = false;
  bool bM_Set = false;
  
  //Datapack RF Trans
  float datapack[8];
  datapack[0] = RPM; datapack[1] = Motor_Temp; datapack[2] = PowerStage_Temp; datapack[3] = Air_Temp;
  datapack[4] = LPM; datapack[5] = Torque_Out; datapack[6] = Torque_Set, datapack[7] = ;

  radio.stopListening(); //송신모드
  radio.write(datapack, sizeof(datapack));

  delay(10);
  digitalWrite(8, HIGH);     //RF24 비활성화
  digitalWrite(10, LOW);     //CAN 통신 활성화


  //  for(int i =0;i<7;i++){
  //    Serial.print(datapack[i],HEX);
  //    Serial.print(" ");}
  //  for(int i =0;i<7;i++){
  //    Serial.print(datapack1[i],HEX);
  //    Serial.print(" ");
  //    }
  //    Serial.println("");
}

//////////////////////////////////////////////////////send/////////////////////////////////////////
void mcp2515_send(unsigned int id, byte data[], unsigned int dlc) {
  struct can_frame canMsg;
  canMsg.can_id = id; //슬레이브의 ID
  canMsg.can_dlc = dlc;
  for (int i = 0; i < dlc; i++) {
    canMsg.data[i] = data[i];
  }
  mcp2515.sendMessage(&canMsg);
  Serial.println("[모터 컨트롤러에 보낸 메시지]");
  Serial.println(id, HEX);
  for (int i = 0; i < canMsg.can_dlc; i++) {
    Serial.print(canMsg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

///////////////////////////////////////////////////receive/////////////////////////////////////////
unsigned int mcp2515_receive(byte recv[]) {
  struct can_frame canMsg;
  unsigned long t = millis();
  int ret;
  while (1) {
    if (millis() - t > 2000) {
      ret = -1;
      break;
    }
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK && canMsg.can_id == tx) {
      dlc = canMsg.can_dlc;
      for (int i = 0; i < dlc; i++) {
        recv[i] = canMsg.data[i];
        Serial.print(recv[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      ret = canMsg.data[0];
      break;
    }
  }
  return ret;
}

////////////////////////////////////////////비트연산 함수/////////////////////////////////////////
int convertToInt(byte recv[], int n){
  int ret = 0;
  for (int i = 1; i < n; i++){
    ret = ret << 8;
    ret = (ret | recv[i]);
  }
  return ret;
}


////////////////////////////////////////////Interrupt function/////////////////////////////////////////
void flow(){
  flow_frequency++;
}
