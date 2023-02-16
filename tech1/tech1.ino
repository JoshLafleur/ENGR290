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
int trigPin = 13; // yellow jumper --> PB5
int echoPin = 8;// blue jumper --> PB0

// IR SENSOR SPECIFIC

long duration, cm, inches;

// We are using model GP2Y0A21YK0F i.e. 1080
// the code for this class object was taken from here: https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
// and slightly modified as needed
SharpIR mySensor = SharpIR(IRPin, model);
// this code was originally taken from here: https://github.com/Martinsos/arduino-lib-hc-sr04/blob/master/src/HCSR04.cpp
// and slightly modified as needed
UltraSonicDistanceSensor distanceSensor(12, 8);  // Initialize sensor that uses digital pins 13 and 12.

// function to reset arduino
void(* resetFunc) (void) = 0; // declare reset function @ address 0

void setBrightnessAndWait(int brightness, int waitTime) {
    //Serial.print("Setting brightness to: ");
    //Serial.print(brightness);
    //Serial.println("/255");
    
    OCR2A = brightness;
    OCR2B = brightness;
    
    delay(waitTime);
}

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
    _delay_ms(1000);
    resetFunc();
  }
}

// the calm before the storm...
void pauseChamp() {
  Serial.print("Starting in 3 seconds.");
  _delay_ms(1000);
  Serial.print(".");
  _delay_ms(1000);
  Serial.println(".");
  _delay_ms(1000);
}

void setup() {
  // Begin serial communication at a baudrate of 9600:
  Serial.begin(9600);

 // mode = selectMode(); // @TODO: why does using this completely halt the whole arduino process when we introduce baremetal equivalent of analogWrite????
  mode = 1; // TEMP WORK AROUND
  pauseChamp();

  if (mode == 0) { // IR SENSOR
  
  } else if (mode == 1) { // US SENSOR

    //pinMode(trigPin, OUTPUT); // 14 --> PB5
    DDRB |= (1 << 5);
    //pinMode(echoPin, INPUT);  // 5  --> PB0
    DDRD &= ~(1 << 0); 
  }

  // pinMode(11, OUTPUT); // D11 = PB3 = D3
  DDRB |= (1 << 3);

    // setup for analogwrite equivalent via PWM

    // OCR = Output Compare Register, 2 for Clock 2
    OCR2A = 255; // set duty cycles to maximum value
    OCR2B = 255;
    
    // timer/counter2 control register A
    // set fast PWM mode, non inverting
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // WGM --> Wave Form Generation Mode
    // set prescaler to 64 (i.e. divide clock speed of timer 2)
    TCCR2B = (1 << CS22);
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

  // spec: "15cm or less - 100%""
  if (distance_cm <= 15) {
    //analogWrite(11, 0);
    //digitalWrite(11, HIGH);
    // TEMP FIX because regular analogWrite (linear scaling one) messes up since it influences some bit in a timer register... or smth like that
    // this fixes the issue because analogWrite(0) isn't actually possible to do DC = 0 on arduino
    // so the pin is released
    //analogWrite(11, 0);
    setBrightnessAndWait(1, 10);    
    PORTB &= ~(1 << 3); // @TODO: is this not working properly? i.e. why does LED D3 not seem to budge....
    

    //digitalWrite(13, HIGH); // pin 13 = PB5
    PORTB |= (1 << 5);

  }
  // spec: "45cm or more - 0%"
  else if (distance_cm > 45) {
    //analogWrite(11, 255);
    //digitalWrite(11, LOW); // pin 11 = PB3
    // TEMP FIX (see the explanation above)
    //analogWrite(11, 0);
    setBrightnessAndWait(254, 10);
    PORTB |= (1 << 3); // @TODO: is this not working properly? i.e. why does LED D3 not seem to budge...
    //digitalWrite(13,HIGH); // pin 13 = PB5
    PORTB |= (1 << 5);
  }
  else {
    // spec: "linearly increases from 45 cm to 15 cm"
    // analogWrite(11, (255*((float) (distance_cm - 15)/30)));
    setBrightnessAndWait((255*((float) (distance_cm - 15)/30)), 10);
    //digitalWrite(13, LOW); // pin 13 = PB5
    PORTB &= ~(1 << 5);
  }
  Serial.println(" cm");

  _delay_ms(50);
}
