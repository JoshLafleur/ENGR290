// Example by Hisham Marzouk
//Officialhrm.com

// PD2 - echo - D2 (physical 5)
// PB3 - trig - D11 (physical 14)

// src: https://www.officialhrm.com/arduino/arduino-hc-sr04
int trigPin = 14;//Trig – green Jumper 
int echoPin = 5;    //Echo – yellow Jumper
long duration, cm, inches;
// void setup() {
//  Serial.begin (9600);
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
// }
// void loop() {

//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(5);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  pinMode(echoPin, INPUT);
//  duration = pulseIn(echoPin, HIGH);
//  cm = (duration/2) / 29.1;
//  inches = (duration/2) / 74;
//  Serial.print(cm);
//  Serial.println();
//  delay(350); 
//  }

