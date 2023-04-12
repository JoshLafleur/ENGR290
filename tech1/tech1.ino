// Compiler directives to compute UBBR
// clock goes brr at 16 MHz
#define F_CPU 16000000 // F_CPU needs to be declared BEFORE including delay.h
// standard baudrate setting 9600 bits/s
#define BAUD 9600
// formula from datasheet page 146 --> UBRRn = ((f_OSC/16 * BAUD) - 1)
#define MYUBRR F_CPU/16/BAUD-1 // asynchronous normal mode (where double speed operation U2Xn = 0)

#include <avr/io.h>
#include <string.h>
#include <avr/delay.h>

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
UltraSonicDistanceSensor distanceSensor(12, 8);  // Initialize sensor that uses digital pins 12 --> PB4, 8 --> PB0

int index = 0;
char buffer[256];

// this method was taken from: https://www.geeksforgeeks.org/program-count-digits-integer-3-different-methods
int countDigit(long long n)
{
    if (n == 0)
        return 1;
    int count = 0;
    while (n != 0) {
        n = n / 10;
        ++count;
    }
    return count;
}

void USART_init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8); // Shift needed to clear out first 8 bits
    UBRR0L = (unsigned char) (ubrr);

    // Enable receiver and transmitter
    UCSR0B = ((1 << RXEN0) | (1 << TXEN0));

    // Set two stop bits with USBS0, set frame to 8 bits of data with UCSz00
    // based on datasheet example code page 149
    UCSR0C = ((1 << USBS0) | (3 << UCSZ00));
}

// based on C code example page 150
void USART_transmit(char data) {
    // wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));

    // place data into buffer and send
    UDR0 = data;
}

// based on C code example page 169
char USART_receive(void) {
    // Wait for empty transmit buffer
    while(!(UCSR0A & (1 << RXC0)));

    // get data recvd from buffer
    return UDR0;
}

// receive multiple chars given expected input message length
char* msg_receive(int len) {
    char msg_buffer[len + 1];

    for (int i = 0; i < len; i++) {
        // Wait for empty transmit buffer
        while(!(UCSR0A & (1 << RXC0)));

        // Place data into buffer and send
        msg_buffer[i] = UDR0;
    }

    return msg_buffer;
}

// send multiple chars given string pointer and length
void msg_transmit(char* data, int len) {
    for (int i = 0; i < len; i++) {
        USART_transmit(data[i]);
        //_delay_us(10);
    }
}


// function to reset arduino
void(* resetFunc) (void) = 0; // declare reset function @ address 0

void setBrightnessAndWait(int brightness, int waitTime) {
    // "the result of the compare can be used by the waveform generator to generate a PWM or variable frequency output on the output compare pins (OC2A and OC2B)"
    // page 117
    OCR2A = brightness;
    OCR2B = brightness;
    
    _delay_ms(waitTime);
}

int selectMode() {
  msg_transmit("Which mode would you like to test?\n", 35);
  msg_transmit("OPTIONS:\n", 9);
  msg_transmit("0 --> IR sensor\n", 16);
  msg_transmit("1 --> US sensor\n", 16);

  //char *msg = msg_receive(1);
  //int option = atoi(*msg);
  int option = 1;

  if (option == 0) {
    msg_transmit("You chose: option 0 --> IR sensor\n", 35);
    return option;
  } else if (option == 1) {
    msg_transmit("You chose: option 1 - US sensor\n", 32);
    return option;
  } else {
    msg_transmit("Invalid option, try again\n", 26);
    _delay_ms(1000);
    resetFunc();
  }
}

// the calm before the storm...
void pauseChamp() {
  msg_transmit("Starting in 3 seconds.", 22);
  _delay_ms(1000);
  msg_transmit(".", 1);
  _delay_ms(1000);
  msg_transmit(".", 1);
  _delay_ms(1000);
}

void setup() {
  // Begin serial communication at a baudrate of 9600:
  USART_init(MYUBRR);

  // ask user to chose IR vs US mode
  // to reset just close/reopen serial monitor via: ctrl + shift + m
  mode = selectMode();
  pauseChamp();

  // pinMode(11, OUTPUT); // D11 = PB3 = D3
  DDRB |= (1 << PB3); // set external LED D3 to output mode

  // setup for analogwrite equivalent via PWM

  // OCR = Output Compare Register, 2 for Clock 2
  OCR2A = 255; // set duty cycles to maximum value
  OCR2B = 255;
  
  // timer/counter2 control register A
  // set fast PWM mode, non inverting
  // page 128
  // COM2A1 --> clear OC2A on compare match
  // COM2B1 --> clear OC2B on compare match
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // WGM --> Wave Form Generation Mode
  // set prescaler to 64 (i.e. divide clock speed of timer 1)
  TCCR2B = (1 << CS22);
}

char distance_str[3];
int numOfDigits = 0;

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
  msg_transmit("Mean distance: ", 15);
  itoa(distance_cm, distance_str, 10);
  numOfDigits = countDigit(distance_cm);
  
  msg_transmit(distance_str, numOfDigits);
  msg_transmit(" cm\n", 4);

  // spec: "15cm or less - 100%""
  if (distance_cm <= 15) {
    //analogWrite(11, 0);
    setBrightnessAndWait(1, 10);    
    PORTB &= ~(1 << PB3);
    

    //digitalWrite(13, HIGH); // pin 13 = PB5
    PORTB |= (1 << PB5);

  }
  // spec: "45cm or more - 0%"
  else if (distance_cm > 45) {
    //analogWrite(11, 255);
    //digitalWrite(11, HIGH); // pin 11 = PB3
    setBrightnessAndWait(254, 10);
    PORTB |= (1 << 3);
    //digitalWrite(13,HIGH); // pin 13 = PB5
    PORTB |= (1 << 5);
  }
  else {
    // spec: "linearly increases from 45 cm to 15 cm"
    // analogWrite(11, (255*((float) (distance_cm - 15)/30)));
    setBrightnessAndWait((255*((float) (distance_cm - 15)/30)), 10);
    //digitalWrite(13, LOW); // pin 13 = PB5
    PORTB &= ~(1 << PB5);
  }

  _delay_ms(50);
}
