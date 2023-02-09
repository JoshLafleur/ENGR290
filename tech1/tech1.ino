// Include the library:
#include "SharpIR.h"
#include "HCSR04.h"

// Define model and input pin:
#define IRPin A0
#define model 1080

// Create variable to store the distance:
int distance_cm;

// global status vars
// MODE 0 --> IR sensor
// MODE 1 --> US sensor
int mode = -1;

// US SENSOR SPECIFIC

// IR SENSOR SPECIFIC
int trigPin = 14;   //Trig – green Jumper // 14 --> PB0 
int echoPin = 5;    //Echo – yellow Jumper // 5 --> PD3
long duration, cm, inches;

/* Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/

// Create a new instance of the SharpIR class:
SharpIR mySensor = SharpIR(IRPin, model);
UltraSonicDistanceSensor distanceSensor(12, 8);  // Initialize sensor that uses digital pins 13 and 12.

// function to reset arduino
void(* resetFunc) (void) = 0; // declare reset function @ address 0


int selectMode() {
  Serial.println("Which mode would you like to test?");
  Serial.println("OPTIONS:");
  Serial.println("0 --> IR sensor");
  Serial.println("1 --> US sensor");

  // Wait for user input
  while (Serial.available() == 0);

  int option = Serial.parseInt();

  if (option == 0) {
    Serial.println("You chose: option 0 --> IR sensor");
    return option;
  } else if (option == 1) {
    Serial.println("You chose: option 1 - US sensor");
    return option;
  } else {
    Serial.println("Invalid option, try again");
    delay(100);
    resetFunc();
  }
}

// the calm before the storm...
void pauseChamp() {
  Serial.print("Starting in 3 seconds.");
  delay(1000);
  Serial.print(".");
  delay(1000);
  Serial.println(".");
  delay(1000);
}

void setup() {
  // Begin serial communication at a baudrate of 9600:
  Serial.begin(9600);

  mode = selectMode();
  pauseChamp();

  if (mode == 0) { // IR SENSOR
  
  } else if (mode == 1) { // US SENSOR

    //pinMode(trigPin, OUTPUT); // 14 --> PB0
    DDRB &= ~(1 << 0);
    //pinMode(echoPin, INPUT);  // 5  --> PD3
    DDRD |= (1 << 3);
  }

  // pinMode(11, OUTPUT); // D11 = PB3 = D3
  DDRB &= ~(1 << 3);
}

void loop() {
  // Get a distance measurement and store it as distance_cm:
  //distance_cm = mySensor.distance();

  if (mode == 0) { // IR SENSOR
    distance_cm = mySensor.distance();    
  }
  else if (mode == 1) { // US SENSOR
    distance_cm = distanceSensor.measureDistanceCm();
  }
    
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
