// int analogReading = 0;

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   pinMode(A0, INPUT);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   if (Serial.available() > 0) {
//     char byteBuffer = Serial.read();
//     analogReading = analogRead(A0);
//     Serial.print("analog reading:");
//     Serial.println(analogReading);
//   }
// }

// src: https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/

/*SHARP GP2Y0A21YK0F IR distance sensor with Arduino and SharpIR library example code. More info: https://www.makerguides.com */

// Include the library:
#include <SharpIR.h>

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

void setup() {
  // Begin serial communication at a baudrate of 9600:
  Serial.begin(9600);
  // pinMode(D11, OUTPUT); // D11 = PB3 = D3 
}

void loop() {
  // Get a distance measurement and store it as distance_cm:
  distance_cm = mySensor.distance();

  // Print the measured distance to the serial monitor:
  Serial.print("Mean distance: ");
  Serial.print(distance_cm);

  if (distance_cm <= 15) {
    // 100%
    //analogWrite(D11, 255); @TODO: how the hell do we control "D3"? what is our interface to it???
  } else if (distance_cm > 45) {
    // 0%
    //digitalWrite(D11, 0); 
  }
  else {
    // scale based on value
  }
  Serial.println(" cm");

  delay(1000);
}
