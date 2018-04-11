// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include <SerialCommands.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif


#define PIN 6
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_RGBW           + NEO_KHZ800);

int delayval = 500; // delay for half a second



char serial_command_buffer_[32];
SerialCommands serial_commands(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");

//This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print(F("Unrecognized command ["));
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

//expects 3 parameters x,y,v
void cmd_setled_cb(SerialCommands* sender)
{
  char* arg_str;
  unsigned int x, y;
  unsigned long v;
  
  arg_str = sender->Next();
  if (arg_str == NULL) {
    sender->GetSerial()->println(F("ERROR: X Y V"));
    return;
  }
  x = atoi(arg_str);

  arg_str = sender->Next();
  if (arg_str == NULL) {
    sender->GetSerial()->println(F("ERROR: X Y V"));
    return;
  }
  y = atoi(arg_str);

  arg_str = sender->Next();
  if (arg_str == NULL) {
    sender->GetSerial()->println(F("ERROR: X Y V"));
    return;
  }
  v = atol(arg_str);

  matrix.setPassThruColor(v);
  matrix.drawPixel(x, y, 0);
//  matrix.drawPixel(x, y, matrix.Color( (v&0xFF000000)>>24, (v&0x00FF0000)>>16,(v&0x0000FF00)>>8, v&0x0000FFFF) );

}

void cmd_setbri_cb(SerialCommands* sender)
{
  char* arg_str;
  unsigned int v;
  
  arg_str = sender->Next();
  if (arg_str == NULL) {
    sender->GetSerial()->println(F("ERROR: V"));
    return;
  }
  matrix.setBrightness(atoi(arg_str));
}


SerialCommand cmd_bright("BRI", cmd_setbri_cb);
SerialCommand cmd_setled("LED", cmd_setled_cb);

void setup() {
  Serial.begin(115200);
  matrix.begin();
  matrix.setBrightness(40);

  serial_commands.SetDefaultHandler(cmd_unrecognized);
  serial_commands.AddCommand(&cmd_bright);
  serial_commands.AddCommand(&cmd_setled);

}

void loop() {

  serial_commands.ReadSerial();
  matrix.show();
  delay(100); // Delay for a period of time (in milliseconds).

}

