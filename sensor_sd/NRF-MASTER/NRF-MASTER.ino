#include <SPI.h>
#include "RF24.h"
#include "SoftReset.h"
RF24 radio(9, 10); //CE, SS
uint8_t address[6] = "04171";

void setup() {
  Serial.begin(115200); 
  radio.begin(); 
  radio.setPALevel(RF24_PA_MAX); // 통신 거리에 따라 설정값 변화
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);
  radio.startListening(); //기본 : 수신모드

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

  while(1){
    if(radio.begin()){
      int c3;
      radio.startListening();
      radio.read(&c3,sizeof(c3));
      //Serial.println(c3);  
      delay(10);
        if (c3==1285){   
         Serial.println("Digitalsensor_connected");              //----3333-------디지털 신호 양호
         delay(500);
         break;
        }
        if (c3==1542){   
          Serial.println("ERR_Digitalsensor");            
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
      Serial.println("Time, Senor1, Senor2, Senor3, lin_vel"); // legend
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

void loop() {

    radio.startListening(); //수신모드
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
      float sensor_data[4];
      radio.read(sensor_data,sizeof(sensor_data));
      Serial.print(sensor_data[0]); Serial.print("  ");
      Serial.print(sensor_data[1]); Serial.print("  ");
      Serial.print(sensor_data[2]); Serial.print("  ");
      Serial.println(sensor_data[3]);//Serial.print("  ");
      //Serial.println(sensor_data[4]);
    }
    
  String p3;
  if(Serial.available()){
    p3 = Serial.readString();
    if(p3.indexOf("reset") != -1){
      Serial.println("Restart the both board ");
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
