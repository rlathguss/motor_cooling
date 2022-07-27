#include <SPI.h>
#include <SD.h>

int j=0;
float tt=1;
const int chipSelect = 4;

#define coolantsensorDivider 4700   
             
const float steinconstA = 0.001092768344245138;       
const float steinconstB = 0.000181450723833218;      
const float steinconstC = 0.000000222855858126706000;   

void setup() {
    Serial.begin(9600);
    Serial.println("Filtered_sneor1:,sensor1:,Filtered_sneor2:,sensor2:,Filtered_sneor3:,sensor3:"); // legend
}

void loop() {

  uint8_t i;  
  float average1=0, average2=0, average3=0;  
  float sensor_value1 = analogRead(A0),sensor_value2= analogRead(A1),sensor_value3 = analogRead(A2);
  for (i=0; i<100; i++) {                      
    average1+=analogRead(A0);
    average2+=analogRead(A1);
    average3+=analogRead(A2);
    delayMicroseconds(100);
  }

  average1 /= 100;     
  average2 /= 100;
  average3 /= 100;                                      
  average1 = (coolantsensorDivider*average1)/(1023-average1);      
  average2 = (coolantsensorDivider*average2)/(1023-average2);    
  average3 = (coolantsensorDivider*average3)/(1023-average3);    

  sensor_value1 = (coolantsensorDivider*sensor_value1)/(1023-sensor_value1);
  sensor_value2 = (coolantsensorDivider*sensor_value2)/(1023-sensor_value2);
  sensor_value3 = (coolantsensorDivider*sensor_value3)/(1023-sensor_value3);     

  float steinhart1, steinhart2, steinhart3,sen1,sen2,sen3;                         
  steinhart1 = log(average1);                   
  steinhart1 = pow(steinhart1,3);                 
  steinhart1 *= steinconstC;                     
  steinhart1 += (steinconstB*(log(average1)));    
  steinhart1 += steinconstA;                     
  steinhart1 = 1.0/steinhart1;                  
  steinhart1 -= 273.15;   

  sen1 = log(sensor_value1);                   
  sen1 = pow(sen1,3);                 
  sen1 *= steinconstC;                     
  sen1 += (steinconstB*(log(sensor_value1)));    
  sen1 += steinconstA;                     
  sen1 = 1.0/sen1;                  
  sen1 -= 273.15;   
  
  steinhart2 = log(average2);                   
  steinhart2 = pow(steinhart2,3);                 
  steinhart2 *= steinconstC;                     
  steinhart2 += (steinconstB*(log(average2)));    
  steinhart2 += steinconstA;                     
  steinhart2 = 1.0/steinhart2;                  
  steinhart2 -= 273.15;

  sen2 = log(sensor_value2);                   
  sen2 = pow(sen2,3);                 
  sen2 *= steinconstC;                     
  sen2 += (steinconstB*(log(sensor_value2)));    
  sen2 += steinconstA;                     
  sen2 = 1.0/sen2;                  
  sen2 -= 273.15; 

  steinhart3 = log(average3);                   
  steinhart3 = pow(steinhart3,3);                 
  steinhart3 *= steinconstC;                     
  steinhart3 += (steinconstB*(log(average3)));    
  steinhart3 += steinconstA;                     
  steinhart3 = 1.0/steinhart3;                  
  steinhart3 -= 273.15;

  sen3 = log(sensor_value3);                   
  sen3 = pow(sen3,3);                 
  sen3 *= steinconstC;                     
  sen3 += (steinconstB*(log(sensor_value3)));    
  sen3 += steinconstA;                     
  sen3 = 1.0/sen3;                  
  sen3 -= 273.15; 
  
  Serial.print(steinhart1);Serial.print(",");Serial.print(sen1);
  Serial.print(",");Serial.print(steinhart2);Serial.print(",");Serial.print(sen2);
  Serial.print(",");Serial.print(steinhart3);Serial.print(","); Serial.println(sen3);
}
