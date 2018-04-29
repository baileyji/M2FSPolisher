/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

#include <LCD2.h>

LCD2 lcddisplay = LCD2(&Serial3);
bool firstcall=true;
unsigned int i=0;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial3.begin(9600);
//  lcddisplay.bootbaudreset();
  lcddisplay.init();
//  lcddisplay.setbaud9600();
//  lcddisplay.turnDisplayOff();
//  delay(50);
//  lcddisplay.init();
//  delay(605);
//  lcddisplay.turnDisplayOn();
  lcddisplay.display("Hello World!");
  
  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  firstcall=true;
  SerialUSB.begin(0);
  Serial.begin(115200);
//  Serial3.begin(9600);
  
  SerialUSB.println("booting");
  Serial.println("booting");

  lcddisplay.display("Booting");
  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);  
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100); 
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100); 
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100); 
}

// the loop function runs over and over again forever
void loop() {
  if (firstcall) {
    SerialUSB.println("booted");
    Serial.println("booted");
    lcddisplay.display("booted");
    firstcall=false;
  }
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                       // wait for a second
  SerialUSB.println(i);
  Serial.println(i);
  lcddisplay.clear();
  lcddisplay.serial->print(i);
  i++;
}
