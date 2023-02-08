// Include the library:
#include "SharpIR.h"
#include "HCSR04.h"

// Define model and input pin:
#define IRPin A0
#define model 1080

// Compiler directives to compute UBBR
#define F_CPU 16000000 // F_CPU needs to be declared BEFORE including delay.h
#define BAUD 9600
#define MYUBBR F_CPU/16/BAUD-1 // UBBR --> Usart Baud Rate register

// even though arduino IDE imports these automatically, just in case we build this in a different environment
// we still want our register names to resolve properly
#include <avr/io.h>
#include <stdio.h>

// Create variable to store the distance:
int distance_cm;
// and to print that value
char* s_distance_cm;

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
  USART_init(MYUBBR);
  pinMode(11, OUTPUT); // D11 = PB3 = D3 
}

void loop() {
  // Get a distance measurement and store it as distance_cm:
  //distance_cm = mySensor.distance();
  distance_cm = distanceSensor.measureDistanceCm();
  
  // Print the measured distance to the serial monitor:
  // Serial.print("Mean distance: ");
  // Serial.print(distance_cm);
  msg_transmit("Mean distance: ", 15);

  //itoa(distance_cm, s_distance_cm, 10);
  // msg_transmit(s_distance_cm, 3); // numerical values maximum of 3 digits
  Serial.print(distance_cm, DEC);
  // msg_transmit("cm\n", 3); //why does this act so flaky?? sometimes prints m, sometimes cm... hmmm
  Serial.println(" cm");

  /*
                                              UART DEBUGGING ATTEMPTS START
  */
  // if (distance_cm >= 100) {
  //   msg_transmit(" cm\n", 4);
  // } else {
  //   msg_transmit("  cm\n", 5);
  // }

  // why does using iota and printing the value block everything ??????
  // char* s;
  // itoa(distance_cm, s, 10);
  
  // char buffer[32];

  // for (int i = 0; i < strlen(s); i++) {
  //   buffer[i] = s[i];
  // }

  // Serial.print(buffer);
  //Serial.println(" cm");


  //msg_transmit(s, 4); // will only be 3 digits max;
  //Serial.println(typeid(distance_cm).name())
  //Serial.print(distance_cm);

    /*
                                              UART DEBUGGING ATTEMPTS END
  */

  if (distance_cm <= 15) {
    analogWrite(11, 0);
  } else if (distance_cm > 45) {
    analogWrite(11, 255);
  }
  else {
    analogWrite(11, 255*((float) (distance_cm - 15)/30));
  }

  delay(50);
}


void USART_init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8); // Shift needed to clear out first 8 bits
    UBRR0L = (unsigned char) (ubrr);

    // Enable receiver and transmitter
    UCSR0B = ((1 << RXEN0) | (1 << TXEN0));

    // Set two stop bits with USBS0, set frame to 8 bits of data with UCSz00
    UCSR0C = ((1 << USBS0) | (3 << UCSZ00));
}

void USART_transmit(char data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));

    // Place data into buffer and send
    UDR0 = data;
}

void msg_transmit(char* data, int len) {
    for (int i = 0; i < len; i++) {
        USART_transmit(data[i]);
        // _delay_us(10);
    }
}
