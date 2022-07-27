#include <SPI.h>
#include <SD.h>
#define coolantsensorDivider 4700            
#define NUMSAMPLES 50

int j=0;
float tt=1;
const int chipSelect = 4;
const float steinconstA = 0.001092768344245138;       
const float steinconstB = 0.000181450723833218;      
const float steinconstC = 0.000000222855858126706000;   

void setup() {
  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");
  Serial.println("snsor1 , sensor2 , sensor3");
}

void loop() {

  uint8_t i;  
  float average1=0, average2=0, average3=0;    
  for (i=0; i<NUMSAMPLES; i++) {
    average1+=analogRead(A0);
    average2+=analogRead(A1);
    average3+=analogRead(A2);
    delay(10);
  }
  
  average1 /= NUMSAMPLES;     
  average2 /= NUMSAMPLES;
  average3 /= NUMSAMPLES;                                      
  average1 = (coolantsensorDivider*average1)/(1023-average1);      
  average2 = (coolantsensorDivider*average2)/(1023-average2);    
  average3 = (coolantsensorDivider*average3)/(1023-average3);    

  float steinhart1, steinhart2, steinhart3;                           
  steinhart1 = log(average1);                   
  steinhart1 = pow(steinhart1,3);                 
  steinhart1 *= steinconstC;                     
  steinhart1 += (steinconstB*(log(average1)));    
  steinhart1 += steinconstA;                     
  steinhart1 = 1.0/steinhart1;                  
  steinhart1 -= 273.15;   

  steinhart2 = log(average2);                   
  steinhart2 = pow(steinhart2,3);                 
  steinhart2 *= steinconstC;                     
  steinhart2 += (steinconstB*(log(average2)));    
  steinhart2 += steinconstA;                     
  steinhart2 = 1.0/steinhart2;                  
  steinhart2 -= 273.15;

  steinhart3 = log(average3);                   
  steinhart3 = pow(steinhart3,3);                 
  steinhart3 *= steinconstC;                     
  steinhart3 += (steinconstB*(log(average3)));    
  steinhart3 += steinconstA;                     
  steinhart3 = 1.0/steinhart3;                  
  steinhart3 -= 273.15;

  float t=tt;
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    if (tt>=10){
      Serial.print(steinhart1);Serial.print(" , ");Serial.print(steinhart2);Serial.print(" , ");Serial.println(steinhart3);
      dataFile.print(String(t));dataFile.print(" ");dataFile.print(String(steinhart1));dataFile.print(" ");dataFile.print(String(steinhart2));dataFile.print(" ");dataFile.println(String(steinhart3));
      dataFile.close();
      }
  }
  else {
    Serial.println("error opening datalog.txt");
    while(1);
  }
  tt =tt+0.5;
}
