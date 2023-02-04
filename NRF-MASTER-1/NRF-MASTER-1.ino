#include <SPI.h>
#include "RF24.h"
#include <ArduinoJson.h>

RF24 radio(9, 10); //CE, SS
uint8_t address[6] = "41715";
const float Diameter = 525.0;  //mm단위
float RPM, Lin_vel, PowerStage_Temp, Motor_Temp, Air_Temp, LPM, Torque_Out, 
      Lv_Temp, Xaccle, Yaccle, Zaccle, Xangle, Yangle, Rad, CoolantTemp;
float mt(uint16_t x);
float pst(uint16_t x);
float at(uint16_t x);

void setup() {
  Serial.begin(115200);
  radio.begin(); 
  radio.setPALevel(RF24_PA_MAX); 
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening(); 
}



void loop() {
  bool is_timeout = false;
  unsigned long t = millis();
  while(true){
    if(radio.available()){
      break;
      if(millis() - t > 3000){
        is_timeout = true;
        break;
      }
    }
  } 
  
  if(is_timeout){
    Serial.println("통신시간 초과!");
  }
  else{
    int16_t datapack[14];
    radio.read(datapack,sizeof(datapack));
    RPM = datapack[0];
    Lin_vel = (Diameter) * RPM * PI * 60/1000000/2.8;      //KM/H  기어비 2.8
    Motor_Temp = mt(datapack[1]);
    PowerStage_Temp = pst(datapack[2]);
    Air_Temp = at(datapack[3]);
    LPM = datapack[4]/100;
    Torque_Out = datapack[5];
    Lv_Temp = datapack[6]/100;
    Xaccle = datapack[7]/100;
    Yaccle = datapack[8]/100;
    Zaccle = datapack[9]/100;
    Xangle = datapack[10]/100;
    Yangle = datapack[11]/100;
    Rad = datapack[12];
    CoolantTemp = datapack[13]/100;
    Serial.println(sizeof(datapack));

    String output;
    StaticJsonDocument<128> doc;
    doc["RPM"] = RPM;
    doc["Lin_vel"] = Lin_vel;
    doc["Motor_Temp"] = Motor_Temp;
    doc["PowerStage_Temp"] = PowerStage_Temp;
    doc["Air_Temp"] = Air_Temp;
    doc["LPM"] = LPM;
    doc["Torque_Out"] = Torque_Out;
    doc["Lv_Temp"] = Lv_Temp;
    doc["Xaccle"] = Xaccle;
    doc["Yaccle"] = Yaccle;
    doc["Zaccle"] = Zaccle;
    doc["Xangle"] = Xangle;
    doc["Yangle"] = Yangle;
    doc["Rad"] = Rad;
    doc["CoolantTemp"] = CoolantTemp;
    serializeJson(doc, output);
    Serial.println(output);

  }
}


////////////////////////////////////////////모터 온도 계산함수/////////////////////////////////////////
float mt(uint16_t x){
  float mtemp;
  float p1 = 1.1 ;
  float p2 = 2.733 ; 
  float p3 = -0.7292 ;
  float p4 = -4.595 ;
  float p5 = 0.1875 ;
  float p6 = 4.073 ;
  float p7 = 44.51 ;
  float p8 = 74.18 ;
  mtemp = p1*pow(x,7) + p2*pow(x,6) + p3*pow(x,5) + p4*pow(x,4) + p5*pow(x,3) + p6*pow(x,2) + p7*x + p8;
  
  return mtemp;
}


////////////////////////////////////////////컨트롤러 온도 계산함수/////////////////////////////////////////
float pst(uint16_t x){
  float pstemp;
  float p1 = 0.3135 ;
  float p2 = -0.3306 ; 
  float p3 = -0.1763 ;
  float p4 = 0.4608 ;
  float p5 = 2.982 ;
  float p6 = 0.4211 ;
  float p7 = 32.43 ;
  float p8 = 62.27 ;
  pstemp = p1*pow(x,7) + p2*pow(x,6) + p3*pow(x,5) + p4*pow(x,4) + p5*pow(x,3) + p6*pow(x,2) + p7*x + p8;
  
  return pstemp;
}


//////////////////////////////////////////// 외기 온도 계산함수/////////////////////////////////////////
float at(uint16_t x){
  float atemp;
  float p1 = 0.002892 ;
  float p2 = 0.002666 ; 
  float p3 = -0.006184 ;
  float p4 = 0.01245 ;
  float p5 = 0.2406 ;
  float p6 = 1.611 ;
  float p7 = 32.21 ;
  float p8 = 50.96 ;

  atemp = p1*pow(x,7) + p2*pow(x,6) + p3*pow(x,5) + p4*pow(x,4) + p5*pow(x,3) + p6*pow(x,2) + p7*x + p8;
  
  return atemp;
}



