#include <Arduino.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <mcp2515.h>
#include <stdio.h>
#include <string.h>
#include "RF24.h"
#include "Wire.h"
#include <MPU6050_light.h>

#define rx 0x201
#define tx 0x181

MPU6050 mpu(Wire);
long timer = 0;

RF24 radio(9, 8); //CE, CS
uint8_t address[6] = "41715";

MCP2515 mcp2515(10);   //cs핀 번호 설정
byte recv[8];
byte id;


int16_t SpeedRpmMax = 6000, CurrentDevice = 2000, Current200PC = 1070, I_Max_PK = 424;   //0x59, 0xC6, 0xD9, 0xC4
int16_t SpeedActual, RPM, CurrentActual, Current, Rad,
        Motor_Temp, PowerStage_Temp, Air_Temp, 
        M_Out, M_Set, Torque_Out, Torque_Set,
        Lv_Temp, Xaccle, Yaccle, Zaccle, Xangle, Yangle;
float LPM1, Lv_Temp1, Xaccle1, Yaccle1, Zaccle1, Xangle1, Yangle1;

void mcp2515_send(byte sid, byte data[], byte dlc);
byte mcp2515_receive();

bool bSpeedActual = false;
bool bCurrentActual = false;
bool bMotor_Temp = false;
bool bPowerStage_Temp = false;
bool bAir_Temp = false;
bool bM_Out = false;
bool bM_Set = false;


volatile int flow_frequency; // Measures flow sensor pulses
int16_t LPM; 
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
  Serial.println("Mcp2515 setup success");


  radio.begin(); //아두이노-RF모듈간 통신라인
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.stopListening(); //기본 :  송신모드
  Serial.println("NRF24 setup success");


  Wire.begin();    // 가속도 센서 부팅
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");


  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), flow, RISING); // Setup Interrupt
  //sei();  Enable interrupts
  currentTime1 = millis();
  cloopTime1 = currentTime1;
  Serial.println("Flow sensor ready");


  pinMode(adir1, OUTPUT);    //l298n 모터 a 방향 결정 
  pinMode(adir2, OUTPUT);    //l298n 모터 a 방향 결정 
  currentTime2 = millis();
  cloopTime2 = currentTime2;
  digitalWrite(adir1, 1);                           // 디지털 6번 전진
  digitalWrite(adir2, 0);                           // 디지털 7번 전진
  analogWrite(aena, 0);  
  Serial.println("L298N to radiator connect / fan speed = 0 ");
  Rad = 0;


  digitalWrite(8, HIGH);     //RF24 비활성화
  digitalWrite(10, LOW);     //CAN 통신 활성화
  delay(10);
  
  byte data[3] = { 0x3D,0x30,0x32 };
  mcp2515_send(rx, data, 3); //Request SpeedActual 50ms
  data[1] = 0x49;
  mcp2515_send(rx, data, 3); //Request Motor_Temp 50ms
  data[1] = 0x4A;
  mcp2515_send(rx, data, 3); //Request PowerStage_Temp 50ms
  data[1] = 0x4B;
  mcp2515_send(rx, data, 3); //Request Air_Temp 50ms
  data[1] = 0xA0;
  mcp2515_send(rx, data, 3); //Request M_out 50ms
  //data1[3] = { 0x3D,0x90,0x32 };
  //mcp2515_send(rx, data1, 3); //Request M_set 50ms
  //data1[3] = { 0x3D,0x5F,0x32 };
  //mcp2515_send(rx, data1, 3); //Request CurrentActual 50ms
  Serial.println("Request data transfer success ");
}


void loop() {
  currentTime1 = millis();                         // Every 0.4 second, calculate and print litres/min
  if (currentTime1 >= (cloopTime1 + 400)) {
    cloopTime1 = currentTime1;                     // Updates cloopTime
    LPM1 = (2.5f * flow_frequency / 11.0f);             // Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
    flow_frequency = 0;
    LPM = LPM1 * 100;                            // Reset Counter
    Serial.print(LPM1);                      // Print litres/min
    Serial.println(" L/Min");
  }


  for (;;) {
    Serial.println("루프 진입");
    mpu.update();
    id = mcp2515_receive();
    if (id == 1) { 
      Serial.println("컨트롤러 오프라인! / 팬속도 255 ");
      break;
    }
    else if (id == 0x30) {                                          // SpeedActual
      if (bSpeedActual == true){  
        Serial.println("SpeedActual 중복");
      }
      else{
        SpeedActual  = ((int16_t)recv[2] << 8) | recv[1];
        RPM = SpeedRpmMax * (SpeedActual / 32767);
        bSpeedActual == true;        
        //display.showNumberDec(RPM);
      }
    }
    else if (id == 0x49) {                                          // Motor_Temp
      if (bMotor_Temp == true){
        Serial.println("Motor_Temp 중복");
      }
      else{
        Motor_Temp = ((int16_t)recv[2] << 8) | recv[1];
        bMotor_Temp = true;
      }
    }
    else if (id == 0x4A) {                                          // PowerStage_Temp
      if (bPowerStage_Temp == true){
        Serial.println("PowerStage_Temp 중복");
      }
      else{
        PowerStage_Temp = ((int16_t)recv[2] << 8) | recv[1];
        bPowerStage_Temp = true;
      }
    }
    else if (id == 0x4B) {                                          //Air_Temp
      if (bAir_Temp == true){
        Serial.println("Air_Temp 중복");
      }
      else{
        Air_Temp = ((int16_t)recv[2] << 8) | recv[1];
        bAir_Temp = true;
      }
    }
    else if (id == 0xA0) {                                          //M_Out
      if (bM_Out == true){
        Serial.println("M_Out 중복");
      }
      else{
        M_Out = ((int16_t)recv[2] << 8) | recv[1];
        Torque_Out = M_Out / 32767 * I_Max_PK / 1.4142 * 0.61;
        bM_Out = true;
      }
    }
    
    // else if (id == 0x90) {                                          //M_Set
    //   if (bM_Set == true){
    //     Serial.println("M_Set 중복");
    //   }
    //   else{
    //     M_Set = ((uint16_t)recv[2] << 8) | recv[1];
    //     Torque_Set = M_Set / 32767 * I_Max_PK / 1.4142 * 0.61; 
    //     bM_Set = true;
    //   }
    // }
    // else if (id == 0x5F) {                                          // CurrentActual
    //   if (bCurrentActual == true){
    //     Serial.println("CurrentActual 중복");
    //   }
    //   else{
    //     CurrentActual = convertToInt(recv, canMsg.can_dlc);
    //     Current = 2 / 10 * CurrentDevice * (CurrentActual / Current200PC);        
    //     bCurrentActual = true;
    //   }
    // }

    if (bSpeedActual == true && bMotor_Temp == true && bPowerStage_Temp == true && bAir_Temp == true && bM_Out == true ){
      mpu.update();
      accsen();
      break;
    }
  }


  currentTime2 = millis();                         // Every second
  if (currentTime2 >= (cloopTime2 + 1000)) {
    cloopTime2 = currentTime2;
    if(id == 1){
      analogWrite(aena, 255);                       //디지털 5번 핀이 a 모터 속도 조절 핀  0~255
      Serial.println("can 통신 실패 팬속도 최대"); 
      Rad = 1; 
    }
    else if (Motor_Temp >= 11646 || PowerStage_Temp >= 20250) {  //40도
      analogWrite(aena, 255);
      Serial.println("온도 40 이상 팬속도 최대");
      Rad = 40; 
    }
    else if (Motor_Temp >= 11364 || PowerStage_Temp >= 19733) {  //35도
      analogWrite(aena, 200);
      Serial.println("온도 35 이상 팬속도 200");
      Rad = 35; 
    }
    else if (Motor_Temp >= 11080 || PowerStage_Temp >= 19247) {  //30도
      analogWrite(aena, 150);
      Serial.println("온도 30 이상 팬속도 150");
      Rad = 30; 
    }
    else if (Motor_Temp >= 10795 || PowerStage_Temp >= 18797) {  //25도
      analogWrite(aena, 100);                
      Serial.println("온도 25 이상 팬속도 100");        
      Rad = 25; 
    }
    else if (Motor_Temp >= 10510 || PowerStage_Temp >= 18387) {  //20도
      analogWrite(aena, 50); 
      Serial.println("온도 20 이상 팬속도 50");                       
      Rad = 20; 
    }
    else{
      analogWrite(aena, 0);  
      Serial.println("팬 off");
      Rad = 0; 
    }
  }


  digitalWrite(8, LOW);       //RF24 활성화
  digitalWrite(10, HIGH);     //CAN 통신 비활성화
  delay(10);

  bSpeedActual = false;
  bMotor_Temp = false;
  bPowerStage_Temp= false;
  bAir_Temp = false;
  bM_Out = false;
  //bCurrentActual = false;
  //bM_Set = false;
  
  //Datapack RF Trans
  int16_t datapack[13];
  datapack[0] = RPM; datapack[1] = Motor_Temp; datapack[2] = PowerStage_Temp; datapack[3] = Air_Temp;
  datapack[4] = LPM; datapack[5] = Torque_Out; datapack[6] = Lv_Temp, 
  datapack[7] = Xaccle, datapack[8] = Yaccle, datapack[9] = Zaccle,
  datapack[10] = Xangle, datapack[11] = Yangle , datapack[12] = Rad; 

  radio.stopListening(); //송신모드
  radio.write(datapack, sizeof(datapack));
  Serial.println(output);

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
void mcp2515_send(byte sid, byte data[], byte dlc) {
  struct can_frame canMsg;
  canMsg.can_id = sid;                    //슬레이브의 ID
  canMsg.can_dlc = dlc;
  for (int i = 0; i < dlc; i++) {
    canMsg.data[i] = data[i];
  }
  mcp2515.sendMessage(&canMsg);
  Serial.println("[모터 컨트롤러에 보낸 메시지]");
  Serial.println(sid, HEX);
  for (int i = 0; i < canMsg.can_dlc; i++) {
    Serial.print(canMsg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

///////////////////////////////////////////////////receive/////////////////////////////////////////
byte mcp2515_receive() {
  struct can_frame canMsg;
  unsigned long t = millis();
  byte ret;
  while (1) {
    if (millis() - t > 1000) {
      ret = (byte)1;
      Serial.println(ret);
      break;
    }
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK && canMsg.can_id == tx) {
      int dlc = canMsg.can_dlc;
      Serial.println("[모터 컨트롤러에서 받은 메시지]");
      Serial.println(canMsg.data[0]);
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

////////////////////////////////////////////가속도 센서/////////////////////////////////////////
void accsen(){
  Lv_Temp1 = mpu.getTemp();
  Lv_Temp = Lv_Temp1 * 100; 
  Serial.print(F("TEMPERATURE: "));Serial.println(Lv_Temp1);
  Xaccle1 = mpu.getAccX();
  Xaccle= Xaccle1 * 100;
  Serial.print(F("ACCELERO  X: "));Serial.print(Xaccle1);
  Yaccle1 = mpu.getAccY();
  Yaccle = Yaccle1 * 100;
  Serial.print("\tY: ");Serial.print(Yaccle1);
  Zaccle1 = mpu.getAccZ();
  Zaccle = Zaccle1 * 100;
  Serial.print("\tZ: ");Serial.println(Zaccle1);
  Xangle1 = mpu.getAngleX();
  Xangle = Xangle1 * 100;
  Serial.print(F("ANGLE     X: "));Serial.print(Xangle1);
  Yangle1 = mpu.getAngleY();
  Yangle = Yangle1 * 100;
  Serial.print("\tY: ");Serial.println(Yangle1);
  Serial.println(F("=====================================================\n"));
  timer = millis();
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
