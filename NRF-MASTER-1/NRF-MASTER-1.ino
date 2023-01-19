#include <SPI.h>
#include "RF24.h"
#include <ArduinoJson.h>
RF24 radio(9, 10); //CE, SS
uint8_t address[6] = "41715";
const float Diameter = 525.0;  //mm단위
float lin_vel,PowerStage_Temp, Motor_Temp,Air_Temp;
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
  
////////////////////////////////////////Datapack RF Trans//////////////////////////////////////
  bool is_timeout = false;
  unsigned long t = millis();
  while(true){
    if(radio.available()){break;
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
    float datapack[8],datalog[14];
    radio.read(datapack,sizeof(datapack));
    datalog[0]=datapack[0];
    datalog[1]=datapack[1];
    datalog[2]=datapack[2];
    datalog[3]=datapack[3];
    datalog[4]=datapack[4];
    datalog[5]=datapack[5];
    datalog[6]=datapack[6];
    datalog[7]=datapack[7];
    Motor_Temp = mt(datalog[1]);
    PowerStage_Temp = pst(datalog[2]);
    Air_Temp = at(datalog[3]);
    lin_vel = (Diameter) * datalog[0] * PI * 60/1000000/2.8  //KM/H  기어비 2.8

    //Serial.print(datalog[0]);Serial.print("  ");Serial.print(datalog[1]);Serial.print("  ");
    //Serial.print(datalog[2]);Serial.print("  ");Serial.print(datalog[3]);Serial.print("  ");
    //Serial.print(datalog[4]);Serial.print("  ");Serial.print(datalog[5]);Serial.print("  ");
    //Serial.print(datalog[6]);Serial.print("  ");Serial.print(datalog[7]);Serial.print("  ");
    //Serial.print(datalog[8]);Serial.print("  ");Serial.print(datalog[9]);Serial.print("  ");
    //Serial.print(datalog[10]);Serial.print("  ");Serial.print(datalog[11]);Serial.print("  ");
    //Serial.print(datalog[12]);Serial.print("  ");Serial.println(datalog[13]);

    String output;
    StaticJsonDocument<128> doc;
    doc["RPM"] = datalog[0];
    doc["Lin_vel"] = Lin_vel;
    doc["Motor_Temp"] = Motor_Temp;
    doc["PowerStage_Temp"] = PowerStage_Temp;
    doc["Air_Temp"] = Air_Temp;
    doc["LPM"] = datalog[4];
    doc["Torque_Out"] = datalog[5];
    doc["Torque_Set"] = datalog[6];
    serializeJson(doc, output);
    Serial.println(output);
  }
  delay(10);
}

//Serial.println("Senor1, Senor2, Senor3, RPM, Lin_vel, Motor_temp, Heatsink_temp, Motor_torque, Torque_demand, Motor_vol, Motor_curr, Batt_vol, Batt_curr, Throttle_input_vol");
//{"Sensor1":123,"Sensor2":123,"Sensor3":123,"RPM":123,"Lin_vel":123,"Motor_temp":123,"Heatsink_temp":123,"Motor_torque":123,"Torque_demand":123,"Motor_vol":123,"Motor_curr":123,"Batt_vol":123,"Batt_curr":123,"Throttle_input_vol":123,}


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
  float p1 = 0.002892;
  float p2 = 0.002666 
  float p3 = -0.006184 ;
  float p4 = 0.01245;
  float p5 = 0.2406;
  float p6 = 1.611;
  float p7 = 32.21;
  float p8 = 50.96 ;

  atemp = p1*pow(x,7) + p2*pow(x,6) + p3*pow(x,5) + p4*pow(x,4) + p5*pow(x,3) + p6*pow(x,2) + p7*x + p8;
  
  return atemp;
}



