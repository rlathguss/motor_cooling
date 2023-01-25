volatile int flow_frequency; // Measures flow sensor pulses
float l_min; // Calculated litres/min
unsigned char flowsensor = 2; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;
void flow () // Interrupt function
{
flow_frequency++;
}

void setup() {

pinMode(flowsensor, INPUT);
digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
Serial.begin(115200);
attachInterrupt(0, flow, RISING); // Setup Interrupt
 //sei(); // Enable interrupts
currentTime = millis();
cloopTime = currentTime;
}

void loop() {
 currentTime = millis();
 // Every second, calculate and print litres/hour
if(currentTime >= (cloopTime + 400))
{
 cloopTime = currentTime; // Updates cloopTime

 // Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
 l_min = (2.5 *flow_frequency / 11);
 

 flow_frequency = 0; // Reset Counter
 Serial.print(l_min); // Print litres/min
 Serial.println(" L/Min");
  }

}
