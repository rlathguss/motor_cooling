#include <SPI.h>
#include "RF24.h"
#include "SoftReset.h"
RF24 radio(9, 10); //CE, SS
uint8_t address[6] = "04171";

void setup() {
  Serial.begin(9600)
  radio.begin(); 
  radio.setPALevel(RF24_PA_MAX); 
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);
  radio.startListening(); 

/////////////////////////////////////////Slv Start 0000////////////////////////////////////////
  String p1;
  Serial.println("Start slv?");
  while(1){
    p1 = Serial.readString();
    if(p1.indexOf("yes") != -1){
      for(int i=0;i<30;i++){
        radio.stopListening(); 
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

/////////////////////////////////////////Radio check 1111//////////////////////////////////////
  while(1){
    if(radio.begin()){
      int c1;
      radio.startListening();
      radio.read(&c1,sizeof(c1));
      //Serial.println(c1);
      delay(10);
        if (c1==257){ 
          Serial.println("radio_connected");  
          delay(500);              
          break;
        }
    }
    else{   
      Serial.println("ERR_Radio");            
      delay(500);
      break;
    }
  }
  
///////////////////////////////////analogpin input check 2222//////////////////////////////////
  while(1){
    if(radio.begin()){
      int c2;
      radio.startListening();
      radio.read(&c2,sizeof(c2));
      delay(10);
        if (c2==771){   
          Serial.println("Analogsensor_connected"); 
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

/////////////////////////////////////////Can Trans check 3333//////////////////////////////////
  while(1){
    if(radio.begin()){
      int c3;
      radio.startListening();
      radio.read(&c3,sizeof(c3));
      //Serial.println(c3);  
      delay(10);
        if (c3==1285){   
         Serial.println("CanTrans_connected");
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

/////////////////////////////////////////Record OR Reset///////////////////////////////////////
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
      Serial.println("Senor1, Senor2, Senor3, RPM, Lin_vel, Motor_temp, Heatsink_temp, Motor_torque, Torque_demand, Motor_vol, Motor_curr, Batt_vol, Batt_curr, Throttle_input_vol"); // legend
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

////////////////////////////////////////Datapack RF Trans//////////////////////////////////////
  radio.startListening(); 
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
    float datapack[14];
    radio.read(datapack,sizeof(datapack));
    Serial.print(datapack[0]);Serial.print("  ");Serial.print(datapack[1]);Serial.print("  ");
    Serial.print(datapack[2]);Serial.print("  ");Serial.print(datapack[3]);Serial.print("  ");
    Serial.print(datapack[4]);Serial.print("  ");Serial.print(datapack[5]);Serial.print("  ");
    Serial.print(datapack[6]);Serial.print("  ");Serial.print(datapack[7]);Serial.print("  ");
    Serial.print(datapack[8]);Serial.print("  ");Serial.print(datapack[9]);Serial.print("  ");
    Serial.print(datapack[10]);Serial.print("  ");Serial.print(datapack[11]);Serial.print("  ");
    Serial.print(datapack[12]);Serial.print("  ");Serial.print(datapack[13]);Serial.print("  ");
    Serial.println(datapack[14]);
  }

///////////////////////////////////////////////Reset/////////////////////////////////////////////
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
