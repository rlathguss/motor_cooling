#include <Arduino.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <mcp2515.h>
#include <stdio.h>
#include <string.h>

#define rx 0x201
#define tx 0x181


MCP2515 mcp2515(10);   //cs핀 번호 설정
byte recv[8];
int id;

int16_t SpeedRpmMax, CurrentDevice = 2000, Current200PC = 1070, I_Max_PK;   //0x59, 0xC6, 0xD9, 0xC4
int16_t SpeedActual, RPM, CurrentActual, Current, Rad,
        Motor_Temp, PowerStage_Temp, Air_Temp, 
        M_Out, M_Set, Torque_Out, Torque_Set,
        Lv_Temp, Xaccle, Yaccle, Zaccle, Xangle, Yangle;
float LPM1, Lv_Temp1, Xaccle1, Yaccle1, Zaccle1, Xangle1, Yangle1;

void mcp2515_send(byte sid, byte data[], byte dlc);
byte mcp2515_receive();


void setup() {

  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  Serial.println("Mcp2515 setup success");


  byte data[3] = { 0x3D,0x59,0x64 };
  mcp2515_send(rx, data, 3); //Request SpeedRpmMax 100ms
  data[1] = 0xc4;
  mcp2515_send(rx, data, 3); //Request I_Max_PK 100ms
  Serial.println("Request data transfer success ");
}

void loop() {
  id = mcp2515_receive();
  if(id = 0x59){
    SpeedRpmMax = ((int16_t)recv[2] << 8) | recv[1];
    Serial.print("SpeedRpmMax : ");
    Serial.println(SpeedRpmMax);
  }
  else if(id = 0xc4){
    I_Max_PK = ((int16_t)recv[2] << 8) | recv[1];
    Serial.print("I_Max_PK : ");
    Serial.println(I_Max_PK);
  }
}


/////////////////////////////////////////////////////send/////////////////////////////////////////
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
      // Serial.println("[모터 컨트롤러에서 받은 메시지]");
      // Serial.println(canMsg.data[0]);
      // for (int i = 0; i < dlc; i++) {
      //   recv[i] = canMsg.data[i];
      //   Serial.print(recv[i], HEX);
      //   Serial.print(" ");
      // }
      // Serial.println();
      ret = canMsg.data[0];
      break;
    }
  }
  return ret;
}
