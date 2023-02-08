// Include the library:
#include "SharpIR.h"
#include "HCSR04.h"

// Define model and input pin:
#define IRPin A0
#define model 1080

// Create variable to store the distance:
int distance_cm;

/* Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/

// Create a new instance of the SharpIR class:
SharpIR mySensor = SharpIR(IRPin, model);
UltraSonicDistanceSensor distanceSensor(12, 8);  // Initialize sensor that uses digital pins 13 and 12.

void setup() {
  // Begin serial communication at a baudrate of 9600:
  Serial.begin(9600);
  pinMode(11, OUTPUT); // D11 = PB3 = D3 
}

void loop() {
  // Get a distance measurement and store it as distance_cm:
  //distance_cm = mySensor.distance();
  distance_cm = distanceSensor.measureDistanceCm();
  
  // Print the measured distance to the serial monitor:
  Serial.print("Mean distance: ");
  Serial.print(distance_cm);

  if (distance_cm <= 15) {
    analogWrite(11, 0);
  } else if (distance_cm > 45) {
    analogWrite(11, 255);
  }
  else {
    analogWrite(11, 255*((float) (distance_cm - 15)/30));
  }
  Serial.println(" cm");

  delay(50);
}
