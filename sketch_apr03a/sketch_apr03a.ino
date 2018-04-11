#include <Encoder.h>
#include <Buttons.h>
#include <PID_v1.h>
#include <DueTimer.h>
#include <LCD.h>
#include <PololuQik.h>


#define QIK_ERROR_PIN 2
#define QIK_RESET_PIN 3
#define X_QUADA_PIN 4
#define X_QUADB_PIN 5
#define Y_QUADA_PIN 6
#define Y_QUADB_PIN 7
#define LEFT_BUTTON_PIN 8
#define RIGHT_BUTTON_PIN 9
#define NEG_X_LIM_PIN 20
#define NEG_Y_LIM_PIN 21
#define POS_X_LIM_PIN 22
#define POS_Y_LIM_PIN 23
#define OK_BUTTON_PIN 24
#define ESTOP_BUTTON_PIN 25
#define SPEED_PIN x

#define QIK_BAUD_RATE 115200
#define QIK_MIN_SPEED -127
#define QIK_MAX_SPEED 127
#define QIK_FULL_BREAK 127
#define SWITCH_POLL_US 1000

#define DT_INTERVAL_MS 1  //Should be long relative to the math and has PID implications old used ~10 ms?

#define DEFAULT_PATH_ID 1

#define NUM_PATHS 1
#define NUM_SETTINGS 0
#define NUM_MENU_ITEMS NUM_PATHS+NUM_SETTINGS
#define CONTINUE_INDEX = NUM_MENU_ITEMS



double pidXOut, pidXSet, pidXIn;
double pidYOut, pidYSet, pidYIn;

PID xPID = PID(&pidXIn, &pidXOut, &pidXSet, 2,5,1, DIRECT);  //TODO sort out constants
PID yPID = PID(&pidYIn, &pidYOut, &pidYSet, 2,5,1, DIRECT);

PololuQik2s12v10 Qik = PololuQik2s12v10(QIK_RESET_PIN, QIK_ERROR_PIN);

LCD lcddisplay = LCD();

Encoder xQuad = Encoder(X_QUADA_PIN, X_QUADB_PIN);
Encoder yQuad = Encoder(Y_QUADA_PIN, Y_QUADB_PIN);

byte pins[] = {NEG_X_LIM_PIN, NEG_Y_LIM_PIN, POS_X_LIM_PIN, POS_Y_LIM_PIN,
    OK_BUTTON_PIN, ESTOP_BUTTON_PIN, LEFT_BUTTON_PIN, RIGHT_BUTTON_PIN};

typedef struct VECTOR2D_DOUBLE {
    double x;
    double y;
} vec2d_t;

typedef struct VECTOR2D_INT32 {
    int32_t x;
    int32_t y;
} vec2dint_t;


vec2d_t currentvel;
vec2d_t currentpos;

void posvelISR() {
    int32_t x = xQuad.read();
    int32_t y = yQuad.read();
    currentvel.x = (x-currentpos.x) / DT_INTERVAL_MS;
    currentvel.y = (y-currentpos.y) / DT_INTERVAL_MS;
    currentpos.x = x;
    currentpos.y = y;
}

void setup() {
    //Configure IO
    Buttons.begin(pins, 8);

    //Configure LCD
    lcddisplay.init();
    lcddisplay.display("Booting...");
    
    //Connect to Qik
    Qik.init(115200);

    //Start computing speed
    Timer8.attachInterrupt(posvelISR).start(DT_INTERVAL_MS);

    ///configure pid objects
    pidXSet=0.0;
    pidXIn=0.0;
    pidYSet=0.0;
    pidYIn=0.0;
    xPID.SetOutputLimits(QIK_MIN_SPEED, QIK_MAX_SPEED);
    yPID.SetOutputLimits(QIK_MIN_SPEED, QIK_MAX_SPEED);
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
    

}

void loop() {
  // put your main code here, to run repeatedly:

}
