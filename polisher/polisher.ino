#include <Arduino.h>
#include <PID_v1.h>
//#include <DueTimer.h>
#include <Encoder.h>
#include <LCD2.h>
#include <PololuQik.h>
#include <Buttons.h>
#include <PolishPath.h>
#include <SerialCommands.h>
#include <LEDEffect.h>
#include <PIDAutotuner.h>

//#define SERIAL_TX_BUFFER_SIZE 600

#ifndef vec2d_t_h
#define vec2d_t_h
typedef struct VECTOR2D_DOUBLE {
    double x;
    double y;
} vec2d_t;
typedef struct VECTOR2D_INT32 {
    int32_t x;
    int32_t y;
} vec2dint_t;
#endif

/*
 DIO 0-53

 Serial: 0 (RX) and 1 (TX)  USB-SERIAL (programming)
 Serial1: 19 (RX) and 18 (TX) Qik
 Serial2: 17 (RX) and 16 (TX)
 Serial3: 15 (RX) and 14 (TX) LCD
 SerialUSB
 LED 13
 
 Avoid 0,1, 13 - 19

 SerialUSB commands
 report status
 qik get/set params
 decelleration test: measure deceleration rate for various brake values also will determine impulse response
 Tuning: use tuning code
 */


//quad pins 26,28,34,36
//qik pins err 40 rst 42
//limits 23,25,27,29


#define QIK_ERROR_PIN 40
#define QIK_RESET_PIN 42
#define X_QUADA_PIN 34
#define X_QUADB_PIN 36
#define Y_QUADA_PIN 26
#define Y_QUADB_PIN 28

#define LEFT_BUTTON_PIN 9
#define RIGHT_BUTTON_PIN 11
#define OK_BUTTON_PIN 10
#define ESTOP_BUTTON_PIN 12
#define LED_PIN 13

#define NEG_X_LIM_PIN 23
#define NEG_Y_LIM_PIN 25
#define POS_X_LIM_PIN 27
#define POS_Y_LIM_PIN 29

#define SPEED_PIN A0


#define QIK_BAUD_RATE 115200
#define QIK_MIN_SPEED -127
#define QIK_MAX_SPEED 127
#define QIK_FULL_BRAKE 127
#define QIK_PAUSE_BRAKE 63
#define SWITCH_POLL_US 1000

//This is a maximum per axis speed
#define MAX_COUNTS_PER_MS 10.6667   // about 6.5 cm/s per axis, from MarkI
#define ADC_MAX 1023.0
#define ADC_TO_SPEED  MAX_COUNTS_PER_MS/ADC_MAX
#define IN_PER_COUNT 0.00024157
#define COUNTS_PER_IN 1.0/IN_PER_COUNT
#define MM_PER_COUNT 0.00613100
#define SPEED_TO_MMM 367.86  //convert cts/ms to mm/min

//These are maximum vector speeds
#define MAX_SPEED 10.65
#define CALIBRATE_SPEED 2.0  //MarkI equiv. was about 10.09
#define SLEW_SPEED 8.0  //in units OF COUNTS/ms

#define SOFT_LIMIT_COUNTS 815 //about 5mm
#define POSITION_TOLERANCE 238 //238 is from MarkI ~1.5mm, 16.311 = 0.1mm

#define LOAD_Y_POSITION             15L*maxpos.y/16L   //from MarkI
#define LOAD_X_POSITION             maxpos.x/2         //from MarkI


//old polisher the waypoints were about .14 mm apart
//max p error 654 um. avg 326 um min. detectable speed ~1.15/127
// so speeds below .1 cts/ms (~.61 mm/s) are not readable
#define PID_INTERVAL_MS 10

#define DEFAULT_PATH_ID 1

#define NUM_PATHS 4

#define MAX_MENU_LENGTH NUM_PATHS+1
#define CONTINUE_INDEX NUM_PATHS

#define SCREEN_REFRESH_MS 200



//Hardware (encoders, motor driver, buttons, LCD, LED)
LEDEffect led(LED_PIN);
PololuQik2s12v10 Qik = PololuQik2s12v10(QIK_RESET_PIN, QIK_ERROR_PIN);
LCD2 lcddisplay = LCD2(&Serial3);
Encoder xQuad = Encoder(X_QUADA_PIN, X_QUADB_PIN);
Encoder yQuad = Encoder(Y_QUADA_PIN, Y_QUADB_PIN);
enum Button {NXLIM, NYLIM, PXLIM, PYLIM, OK, ESTOP, LEFT, RIGHT};
const byte ButtonPins[] = {NEG_X_LIM_PIN, NEG_Y_LIM_PIN, POS_X_LIM_PIN, POS_Y_LIM_PIN,
                           OK_BUTTON_PIN, ESTOP_BUTTON_PIN, LEFT_BUTTON_PIN, RIGHT_BUTTON_PIN};


//Polisher State
unsigned long lastScreenRefresh;
unsigned long computetimes[]={0,0,0}, ncomputetimes[]={0,0,0}, maxtimes[]={0,0,0};
int freeram[]={0,0,0};

enum State {RUN, CFG, FAIL, IDLE, UNCAL};
enum FailReason {EHALT, SOFTLIM, LIMITS, CALFAIL, SWBUG};
State currentstate = UNCAL;
bool calibrated = false, pathsconfigured=false;

int menuindex = 0;
int menulen = MAX_MENU_LENGTH - 1;

vec2d_t computedpos;
vec2d_t currentpos;
vec2d_t goalpos;
vec2d_t followerror;
double disttogoal;

vec2d_t currentvel;
vec2d_t goalvel;
double speedSetting=0;

vec2dint_t qikcurrent={0,0};
vec2dint_t qikspeed={0,0};

vec2dint_t maxpos = {51314, 63996};     //From Mark 1
vec2dint_t negSoftLim = {SOFT_LIMIT_COUNTS, SOFT_LIMIT_COUNTS};     //About 5mm away based on Polisher Mark1
vec2dint_t posSoftLim = {51314-SOFT_LIMIT_COUNTS, 63996-SOFT_LIMIT_COUNTS};

//PID  TODO sort out PID constants
double kDx=0, kIx=0, kDy=0, kIy=0;
double kPx=QIK_MAX_SPEED/MAX_COUNTS_PER_MS;
double kPy=QIK_MAX_SPEED/MAX_COUNTS_PER_MS;
double pidXOut, pidYOut;
PID xPID = PID(&currentvel.x, &pidXOut, &goalvel.x, kPx, kIx, kDx, DIRECT);
PID yPID = PID(&currentvel.y, &pidYOut, &goalvel.y, kPy, kIy, kDy, DIRECT);

//Serial interface
char _cmdbuf[32];
SerialCommands cmdIO(&SerialUSB, _cmdbuf, sizeof(_cmdbuf), "\n", " ");
void cmd_unrecognized(SerialCommands* sender, const char* cmd) {
    sender->GetSerial()->print(F("Unrecognized command ["));
    sender->GetSerial()->print(cmd);
    sender->GetSerial()->println("]");
}
//-------------------------------
void report_cb(SerialCommands* sender) {report(sender->GetSerial());}
void qikget_cb(SerialCommands* sender) {tellqikparams(sender->GetSerial());}
void clearstat_cb(SerialCommands* sender) {
    for (int i=0;i++;i<3) computetimes[i]=ncomputetimes[i]=maxtimes[i]=0;
}
void pidtune_cb(SerialCommands* sender) {autotunePID();}
void qikset_cb(SerialCommands* sender) {
    char* par_str = sender->Next();
    char* val_str = sender->Next();
    if ((par_str == NULL) || (val_str == NULL)) {
        sender->GetSerial()->println(F("Must specify parameter and value"));
        return;
    }

    byte stat = Qik.setConfigurationParameter(atoi(par_str), atoi(val_str));
    switch (stat) {
        case 0:
            sender->GetSerial()->println("OK.");
            break;
        case 1:
            sender->GetSerial()->println("Bad Param");
            break;
        case 2:
            sender->GetSerial()->println("Bad Value.");
            break;
    }
}
void deceltest_cb(SerialCommands* sender) {
    vec2d_t pos=currentpos;
    
    sender->GetSerial()->println(F("Decel 0 brake"));
    deceltest(0);
    delay(2000);
    
    sender->GetSerial()->println(F("Decel 32 brake"));
    deceltest(32);
    delay(2000);
    
    sender->GetSerial()->println(F("Decel 95 brake"));
    deceltest(95);
    delay(2000);
    
    sender->GetSerial()->println(F("Decel 127 brake"));
    deceltest(QIK_FULL_BRAKE);
    delay(2000);
    
    goto_pos(pos);
}

SerialCommand cmd_report("S", report_cb);
SerialCommand cmd_getqik("QG", qikget_cb);
SerialCommand cmd_decel("Deceltest", deceltest_cb);
SerialCommand cmd_pidtune("Tunepid", pidtune_cb);
SerialCommand cmd_setqik("QS", qikset_cb);
SerialCommand cmd_clear("clear",clearstat_cb);

//Paths
PolishPath* currentpath;
PolishPath* paths[NUM_PATHS];


//Debugging
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
    char top;
#ifdef __arm__
    return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
#else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}



/*////////////////////////////////////////////////////////////////////////////////
Setup
*/////////////////////////////////////////////////////////////////////////////////
void setup() {

    SerialUSB.begin(2000000);
    //Configure LCD
    Serial3.begin(9600);
    lcddisplay.init();
    delay(500); //required by display
    lcddisplay.display(F("Booting..."));
    
    //Configure IO
    Buttons.begin(ButtonPins, 8);
    
    //Connect to Qik
    //Qik.init(QIK_BAUD_RATE);

    ///Configure pid objects
    xPID.SetOutputLimits(QIK_MIN_SPEED, QIK_MAX_SPEED);
    yPID.SetOutputLimits(QIK_MIN_SPEED, QIK_MAX_SPEED);
    xPID.SetSampleTime(PID_INTERVAL_MS);
    yPID.SetSampleTime(PID_INTERVAL_MS);
    xPID.SetMode(MANUAL);
    yPID.SetMode(MANUAL);

    //Read inital speed setting
    speedSetting = readSpeed();
    
    //Setup SerialUSB
    cmdIO.SetDefaultHandler(cmd_unrecognized);
    cmdIO.AddCommand(&cmd_report);
    cmdIO.AddCommand(&cmd_decel);
    cmdIO.AddCommand(&cmd_getqik);
    cmdIO.AddCommand(&cmd_pidtune);
    cmdIO.AddCommand(&cmd_setqik);
    cmdIO.AddCommand(&cmd_clear);
    
    //Enter uncalibrated state
    currentstate=UNCAL;
    led.fadeUp(2000);
    lcddisplay.display(F("Calibration     Required     OK?"));

}


/*////////////////////////////////////////////////////////////////////////////////
Main Loop
*/////////////////////////////////////////////////////////////////////////////////
void loop() {
    
    unsigned long tic, loopTic, housekeeepingTic=0, timeSincePosvel=millis(), printTic;
    bool pidWillRun;
    
    loopTic = micros();
    
    //Guarantee paths are configured
    configure_paths();
    
    //Monitor Estop
    if (Buttons.clickedsince(ESTOP))
        enterFail(EHALT);
    
    //Monitor Limits
    if (hardLimitsTripped()) {
        bounceOffLimits();
        enterFail(LIMITS);
    }
    if (calibrated && softLimitsTripped())
        enterIdleSoftLimits();
    
    //LED
    led.update();
    
    //Handle Serial commands
    cmdIO.ReadSerial();
    
    //Update readings
    pidWillRun = xPID.WillCompute() || yPID.WillCompute();
    if (pidWillRun || timeSincePosvel>=PID_INTERVAL_MS) {
        tic = micros();
        int32_t x = xQuad.read();
        int32_t y = yQuad.read();
        currentvel.x = (x-currentpos.x) / PID_INTERVAL_MS;
        currentvel.y = (y-currentpos.y) / PID_INTERVAL_MS;
        currentpos.x = x;
        currentpos.y = y;
        computedpos.x += currentvel.x*PID_INTERVAL_MS;
        computedpos.y += currentvel.y*PID_INTERVAL_MS;
        disttogoal = dist(goalpos, currentpos);
        followerror.x = computedpos.x-currentpos.x;
        followerror.y = computedpos.y-currentpos.y;
        timeSincePosvel = millis();
        tic = micros()-tic;
        
        computetimes[0] += tic;
        ncomputetimes[0]++;
        freeram[0]=freeMemory();
        if (tic>maxtimes[0]) maxtimes[0] = tic;
    } else if (housekeeepingTic - loopTic > 5000){
        //~.35ms
        qikcurrent.x=Qik.getM0CurrentMilliamps();
        qikcurrent.y=Qik.getM1CurrentMilliamps();
        housekeeepingTic=loopTic;
    }

    if ((printTic - loopTic) > 2000000) {
      report(&SerialUSB);
      printTic=micros();
    }
    
    //Update Display
    displayScreen(false);
    
    //State machine
    switch(currentstate){
            
        case FAIL:   //We're done until a reset
            delay(1);
            break;
            
        case UNCAL:  //We're uncalibrated
            if (Buttons.clickedsince(OK)) {
                calibrated = calibrate();
                if (calibrated) enterIdleFromUncal();
                else enterFail(CALFAIL);
            }
            break;

        case IDLE:   //We are idle
            
            //Update menu selection
            menulen = currentpath->isMore() ? MAX_MENU_LENGTH: MAX_MENU_LENGTH-1;
            menuindex -= (int) Buttons.clickedsince(LEFT);
            if (menuindex<0) menuindex = menulen-1;
            menuindex += (int) Buttons.clickedsince(RIGHT);
            if (menuindex >= menulen) menuindex %= menulen;

            //When the soft limits are tripped the screen is only showing that so ignore
            if (anyLimitTripped())
                Buttons.clearChangeFlag(OK); //just to be thorough
            else if (Buttons.clickedsince(OK)) {
                if (menuindex!=CONTINUE_INDEX){
                    currentpath = paths[menuindex];
                    currentpath->reset(currentpos);
                    computedpos=currentpos;
                    //if we are at start skip the starting point so we don't pause right away
                    if (inPosition(currentpath->current))
                        currentpath->stepForward(speedSetting);
                }
                goalpos = currentpath->current;
                enterRun();
            }
            break;

        case RUN:   //Movement is needed
            if (!calibrated) {
                enterFail(SWBUG);
                break;
            }
            
            if (Buttons.clickedsince(OK)) {
                enterIdleFromRun(currentpath->isMore());
                break;
            }

            tic = micros();
            if (pidWillRun) {
                
                if ((disttogoal < PID_INTERVAL_MS*speedSetting/2) || (currentpath->goalIsBehind(currentpos))) {
                    bool ismore = currentpath->isMore();
                    if (ismore) {
                        currentpath->stepForward(speedSetting);
                        goalpos = currentpath->current;
                    }
                    //both start and end are pause points
                    if (currentpath->isPausePoint())
                        enterIdleFromRun(ismore);
                        break;
                }
                
                //What speed should we use
                speedSetting = (currentpath->isStart()) ? SLEW_SPEED:readSpeed();
                //Get velocity vector toward path destination
                goalvel = currentpath->vTowardGoal(currentpos, speedSetting);
                
                //Call PID speed computation.  //Its possible that we'd miss an update
                //but if thats the case then well.....
                xPID.Compute();
                yPID.Compute();

                tic = micros()-tic;
                computetimes[1] += tic;
                ncomputetimes[1]++;
                freeram[1]=freeMemory();
                if (tic>maxtimes[1]) maxtimes[1] = tic;
                
                setqikspeeds(pidXOut,pidYOut);
            }
            break;
    }
    loopTic = micros()-loopTic;
    computetimes[2] += loopTic;
    ncomputetimes[2]++;
    freeram[2]=freeMemory();
    if (loopTic>maxtimes[2]) maxtimes[2] = loopTic;
    
}

void setqikspeeds(int x, int y) {
    x = negLX() ? max(x,0):x;
    x = posLX() ? min(x,0):x;
    y = negLY() ? max(y,0):y;
    y = posLY() ? min(y,0):y;
    Qik.setM0Speed(x);
    Qik.setM1Speed(y);
    qikspeed.x=x;
    qikspeed.y=y;
}

void setqikbrakes(unsigned char x, unsigned char y){
    Qik.setBrakes(x, y);
    qikspeed.x=0;
    qikspeed.y=0;
    goalvel.x=0;
    goalvel.y=0;
}

double dist(vec2d_t a, vec2d_t b){
    double dx=a.x-b.x;
    double dy=a.y-b.y;
    return sqrt(dx*dx+dy*dy);
}


void report(Stream* stream) {
    // ~49 ms to send this on the serial bus ~566 char
    
    char statestr[]="NONE";
    switch(currentstate) {
        case RUN:
            strcpy(statestr,"RUN ");
            break;
        case CFG:
            strcpy(statestr,"CFG ");
            break;
        case FAIL:
            strcpy(statestr,"FAIL");
            break;
        case IDLE:
            strcpy(statestr,"IDLE");
            break;
        case UNCAL:
            strcpy(statestr,"UCAL");
            break;
    }
    unsigned long tic=micros();
    stream->print("\033[2J");
    stream->printf(F("State: %4s  -XLim: %1d  +XLim: %1d  -YLim: %1d  +YLim: %1d  Cailb: %1d  Menu: %1d\n"),
                   statestr, negLX(), posLX(), negLY(), posLY(), calibrated, menuindex);
    stream->printf(F("MaxPos: %5d, %5d   SWLIM: %5d, %5d  %5d, %5d\n"),
                   maxpos.x, maxpos.y, negSoftLim.x, negSoftLim.y,
                   posSoftLim.x,posSoftLim.y);
    stream->printf(F("T1: %5d (%5d) ms  T1: %5d (%5d) ms  T1: %5d (%5d) ms  \n"),
                   computetimes[0]/ncomputetimes[0], maxtimes[0], computetimes[1]/ncomputetimes[1],
                   maxtimes[1], computetimes[2]/ncomputetimes[2], maxtimes[2]);
    stream->printf(F("RAM1: %5db  RAM2: %5db  RAM3: %5db\n"), freeram[0], freeram[1], freeram[2]);
    stream->printf(F("xPID: Kp=%10g  Ki=%10g  Kd=%10g\n"), xPID.GetKp(), xPID.GetKi(), xPID.GetKd());
    stream->printf(F("yPID: Kp=%10g  Ki=%10g  Kd=%10g\n\n"), yPID.GetKp(), yPID.GetKi(), yPID.GetKd());
    
    stream->printf(F("Position: %5.0g, %5.0g   Computed Delta: %5.0g, %5.0g   FollowingError: %5.0g\n"),
                   currentpos.x, currentpos.y, currentpos.x-computedpos.x, currentpos.y-computedpos.y,
                   followerror);
    stream->printf(F("Destination: %5.0g, %5.0g\n"), goalpos.x, goalpos.y);
    stream->printf(F("Velocity: %6.2g, %6.2g (%6.2g, %6.2g)   Error: (%6.2g, %6.2g)  Speed: %5.2g\n"),
                   currentvel.x, currentvel.y, goalvel.x, goalvel.y,
                   goalvel.x-currentvel.x, goalvel.y-currentvel.y, speedSetting);
    stream->printf(F("Qik Speed: %4d, %4d   Qik Current: %5d, %5d mA\n"),
                   qikspeed.x, qikspeed.y, qikcurrent.x, qikcurrent.y);
    
    stream->printf(F("Message took %5d us."), tic-micros());
    /*
     --------------------------------------------------------------------------------
     State: XXXX  -XLim: X  +XLim:X  -YLim:X  +YLim: X  Cailb: X  Menu: #
     MaxPos: #####, #####   SWLIM: #####, #####  #####, #####
     T1: ##### (#####) ms  T2: ##### (#####) ms  T3: ##### (#####) ms
     RAM1: #####b  RAM2: #####b  RAM3: #####b\n
     XPID: Kp=XXXXXXXXXX  Ki=XXXXXXXXXX  Kd=XXXXXXXXXX
     yPID: Kp=XXXXXXXXXX  Ki=XXXXXXXXXX  Kd=XXXXXXXXXX
     
     Position: #####,#####   Computed Delta: #####, #####   FollowingError: #####
     Destination:  #####,#####
     Velocity:  -##.##, -##.## (-##.##, -##.##)   Error: -##.##, -##.##  Speed:##.##
     Qik Speed: ####,#### Qik Current:  #####,#####
     */
}

void tellqikparams(Stream* stream) {
    unsigned int param, param1;
    param = Qik.getConfigurationParameter(QIK_CONFIG_DEVICE_ID);
    param1 = Qik.getFirmwareVersion();
    stream->printf(F("Qik %3d FW: %3d PWM "),param, param1);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_PWM_PARAMETER);
    switch(param) {
        case 0:
            stream->print(F("19.7kHz"));
            break;
        case 1:
            stream->print(F("9.8kHz"));
            break;
        case 2:
            stream->print(F("2.5kHz"));
            break;
        case 3:
            stream->print(F("1.2kHz"));
            break;
        case 4:
            stream->print(F("310Hz"));
            break;
        case 5:
            stream->print(F("150Hz"));
            break;
    }
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_SHUT_DOWN_MOTORS_ON_ERROR);
    stream->print(F("\nShutdown on:"));
    if (param) {
        if (param&&0b001) stream->print(F(" serial "));
        if (param&&0b010) stream->print(F(" current "));
        if (param&&0b010) stream->print(F(" fault"));
    } else stream->print(F(" none"));
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_SERIAL_TIMEOUT);
    param = 250*(param&&0b1111)*(((unsigned int)2)<<((param&111000)>>4));
    stream->printf(F("\nSerial timout is %6d ms"), param);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M0_ACCELERATION);
    param1 = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M1_ACCELERATION);
    stream->printf(F("\nMotor Accel %3d, %3d "), param, param1);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M0_BRAKE_DURATION);
    param1 = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M1_BRAKE_DURATION);
    stream->printf(F("\nMotor brake times: %3d, %3d x10ms "), param, param1);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M0_CURRENT_LIMIT_DIV_2);
    param1 = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M1_CURRENT_LIMIT_DIV_2);
    stream->printf(F("\nMotor current limits: %5d, %5d mA "), param*300, param1*300);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M0_CURRENT_LIMIT_RESPONSE);
    param1 = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M1_CURRENT_LIMIT_RESPONSE);
    stream->printf(F("\nMotor current limit response: %3d, %3d"), param, param1);
}

void displayScreen(bool force) {
    
    long t = millis();
    
    if (softLimitsTripped()){
        /*
         ----------------
         Soft Limit Trip
         -X -Y +X +Y
         */
        lcddisplay.display(F("Soft Limit Trip"));
        if (negSLX()) lcddisplay.print(F("-X "));
        if (negSLY()) lcddisplay.print(F("-Y "));
        if (posSLX()) lcddisplay.print(F("+X "));
        if (posSLY()) lcddisplay.print(F("+Y"));
        lastScreenRefresh=t;
        return;
    }
    
    if (!force && (t-lastScreenRefresh < SCREEN_REFRESH_MS))
        return;
    lastScreenRefresh=t;
    
    switch(currentstate){
        case IDLE:
            displayIdleScreen();
            break;
        case RUN:
            displayRunScreen();
            break;
        default:
            break;
    }
}

void displayRunScreen() {
    /*
     ----------------
     Run  Xxxxx Yyyyy
     PathNa xxx% XXXm
     */
    unsigned int timeleft=currentpath->timeLeft;
    if (timeleft > 999) timeleft=999;
    lcddisplay.clear();
    lcddisplay.serial->printf(F("Run  X%4.0g Y%4.0g%6s %3d%% %3dm"),
                              currentpos.x*MM_PER_COUNT,currentpos.y*MM_PER_COUNT,
                              currentpath->name, currentpath->percentComplete, timeleft);
}

void displayIdleScreen() {
    
    char namestr[]="Path#";
    //char parstr[]="  0";
    /*
     ----------------
     Ready.       Go?
     PathName    xxx%
     ----------------
     PathName
     Slew to start?
     ----------------
     Paused.  Resume?
     PathName     xx%
    */
    
    lcddisplay.clear();
    if (menuindex!=CONTINUE_INDEX) {
        namestr[4] = menuindex+'0';
        if (inPosition(paths[menuindex]->startpos))
            lcddisplay.serial->printf(F("Ready.       Go?%8s    %3d%%"), namestr, 0);
        else
            lcddisplay.serial->printf(F("%8s        Slew to start?"), namestr);
    } else {
        lcddisplay.serial->printf(F("Paused.  Resume?%8s    %3d%%"),
                                  namestr, currentpath->percentComplete);

    }
    
}


bool softLimitsTripped() {
    return negSLX()||posSLX()||negSLY()||posSLY();
}

bool hardLimitsTripped() {
    return negLX() || posLX() || negLY() || posLY();
}

bool posOutsideLimits(vec2d_t pos) {
    return (pos.x<negSoftLim.x || pos.x>posSoftLim.x ||
            pos.y<negSoftLim.y || pos.y>posSoftLim.y);
}

inline bool anyLimitTripped() {
    return softLimitsTripped() || hardLimitsTripped();
}

inline bool negSLX() {return xQuad.read()<negSoftLim.x;}
inline bool posSLX() {return xQuad.read()<posSoftLim.x;}
inline bool negSLY() {return yQuad.read()<negSoftLim.y;}
inline bool posSLY() {return yQuad.read()<posSoftLim.y;}

inline bool negLX() {return Buttons.down(NXLIM);}
inline bool posLX() {return Buttons.down(PXLIM);}
inline bool negLY() {return Buttons.down(NYLIM);}
inline bool posLY() {return Buttons.down(PYLIM);}

double readSpeed(){
    return analogRead(SPEED_PIN)*ADC_TO_SPEED;
}

void bounceOffLimits() {
    ehalt();
    if (negLX() && !posLX()) {
        Qik.setM0Speed(QIK_MAX_SPEED);
        qikspeed.x=QIK_MAX_SPEED;
    }
    if (posLX() && !negLX()) {
        Qik.setM0Speed(-QIK_MAX_SPEED);
        qikspeed.x=-QIK_MAX_SPEED;
    }
    if (negLY() && !posLY()) {
        Qik.setM1Speed(QIK_MAX_SPEED);
        qikspeed.y=QIK_MAX_SPEED;
    }
    if (posLY() && !negLY()) {
        Qik.setM1Speed(-QIK_MAX_SPEED);
        qikspeed.y=-QIK_MAX_SPEED;
    }
    delay(250);
    setqikspeeds(0,0);
}


void enterIdleFromUncal(){
    currentstate=IDLE;
    configure_paths();
    led.breathe(3000);
    Buttons.clearChangeFlag(LEFT);
    Buttons.clearChangeFlag(RIGHT);
    Buttons.clearChangeFlag(OK);
    goalvel.x=0;
    goalvel.y=0;
    menuindex = 0;
    menulen = MAX_MENU_LENGTH-1;
    displayScreen(true);
}


void enterIdleSoftLimits() {
    currentstate=IDLE;
    led.breathe(1000);
    xPID.SetMode(MANUAL);
    yPID.SetMode(MANUAL);
    setqikbrakes(QIK_PAUSE_BRAKE, QIK_PAUSE_BRAKE);
    
    Buttons.clearChangeFlag(LEFT);
    Buttons.clearChangeFlag(RIGHT);
    Buttons.clearChangeFlag(OK);
    
    menulen = MAX_MENU_LENGTH-1; //We can't continue anything from here
    menuindex = 0;
    
    displayScreen(true);
}

void enterIdleFromRun(bool allowcontinue) {
    currentstate=IDLE;
    led.breathe(3000);
    xPID.SetMode(MANUAL);
    yPID.SetMode(MANUAL);
    setqikbrakes(QIK_PAUSE_BRAKE, QIK_PAUSE_BRAKE);
    
    Buttons.clearChangeFlag(LEFT);
    Buttons.clearChangeFlag(RIGHT);
    Buttons.clearChangeFlag(OK);
    
    menulen = allowcontinue ? MAX_MENU_LENGTH: MAX_MENU_LENGTH-1;
    menuindex = allowcontinue ? CONTINUE_INDEX : currentpath->nextpath;
    
    displayScreen(true);
}

void enterRun() {
    currentstate=RUN;
    led.blink(1500);
    displayScreen(true);
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
}

void enterFail(FailReason reason){
    currentstate=FAIL;
    led.blink(750);
    ehalt();
    switch (reason) {
        case EHALT:
            lcddisplay.display(F("E-Stop Pressed"));
            break;
        case SOFTLIM:
            lcddisplay.display(F("Soft Limits"));
            break;
        case LIMITS:
            lcddisplay.display(F("Hard Limits"));
            break;
        case CALFAIL:
            lcddisplay.display(F("Calibration Fail"));
            break;
        case SWBUG:
            lcddisplay.display(F("Software Failure"));
            break;
        default:
            lcddisplay.display(F("Unknown Failure"));
            break;
    }
}

inline void ehalt() {
    //if (Qik.errorFlag)
        //Qik.reset();
    setqikbrakes(QIK_FULL_BRAKE, QIK_FULL_BRAKE);
}

    
bool calibrate() {
    
    vec2d_t pos;
    
    //Calibrate the y-axis
    if(!negLY()) {
        Qik.setM1Speed(-CALIBRATE_SPEED);
        while(!negLY()) {delay(1);}
        Qik.setM1Brake(QIK_FULL_BRAKE);
    }
    yQuad.write(0);
    
    Qik.setM1Speed(CALIBRATE_SPEED);
    while(!posLY()) {delay(1);}
    Qik.setM1Brake(QIK_FULL_BRAKE);
    maxpos.y=yQuad.read();
    
    //Go to y center
    pos.y=maxpos.y/2;
    pos.x=xQuad.read();
    if (!goto_pos(pos))
        return false;
    
    //Calibrate the x axis
    if(!negLX()) {
        Qik.setM0Speed(-CALIBRATE_SPEED);
        while(!negLX()) {delay(1);}
        Qik.setM0Brake(QIK_FULL_BRAKE);
    }
    xQuad.write(0);

    Qik.setM0Speed(CALIBRATE_SPEED);
    while(!posLX()) {delay(1);}
    Qik.setM0Brake(QIK_FULL_BRAKE);
    maxpos.x=xQuad.read();
    
    //define soft limits
    negSoftLim.x = SOFT_LIMIT_COUNTS;
    negSoftLim.y = SOFT_LIMIT_COUNTS;
    posSoftLim.x = maxpos.x - SOFT_LIMIT_COUNTS;
    posSoftLim.y = maxpos.y - SOFT_LIMIT_COUNTS;
    
    //goto center
    pos.x=maxpos.x/2;
    if (!goto_pos(pos))
        return false;
    
    return !Qik.errorFlag;

}

void configure_paths() {
    if (!calibrated || pathsconfigured) return;
    vec2d_t loadpos = {LOAD_X_POSITION, LOAD_Y_POSITION};

    paths[0]=new LemPath("Lemni+", maxpos, 0, 0.5*COUNTS_PER_IN, 0.0);
    paths[1]=new LemPath("Lemni-", maxpos, 1, 0.5*COUNTS_PER_IN, 0.0);
    paths[2]=new PointPath("Load P", loadpos);
    paths[3]=new LemPath("SkuLem", maxpos, 0, 6.5*COUNTS_PER_IN, 12*COUNTS_PER_IN);
    paths[0]->nextpath=1;
    paths[1]->nextpath=0;
    paths[2]->nextpath=0;
    paths[3]->nextpath=0;
    
    LemPath* p;
    p = (LemPath*) paths[0];
    SerialUSB.printf(F("Name: %s   Next: %s   NLem: %g   Wave#: %g\n"),
                     p->name, paths[p->nextpath]->name, p->nlem, p->awavenum);
    SerialUSB.printf(F("Size: %g   XStretch: %g   Points: %d   Length: %d\n"),
                     p->lemsize, p->xstretch, p->finali, p->pathLength);
    SerialUSB.printf(F("Start: %5d,%5d   End: %5d,%5d\n"),
                     p->startpos.x, p->startpos.y, p->finalpos.x, p->finalpos.y);
    p = (LemPath*) paths[1];
    SerialUSB.printf(F("Name: %s   Next: %s   NLem: %g   Wave#: %g\n"),
                     p->name, paths[p->nextpath]->name, p->nlem, p->awavenum);
    SerialUSB.printf(F("Size: %g   XStretch: %g   Points: %d   Length: %d\n"),
                     p->lemsize, p->xstretch, p->finali, p->pathLength);
    SerialUSB.printf(F("Start: %5d,%5d   End: %5d,%5d\n"),
                     p->startpos.x, p->startpos.y, p->finalpos.x, p->finalpos.y);
    p = (LemPath*) paths[3];
    SerialUSB.printf(F("Name: %s   Next: %s   NLem: %g   Wave#: %g\n"),
                     p->name, paths[p->nextpath]->name, p->nlem, p->awavenum);
    SerialUSB.printf(F("Size: %g   XStretch: %g   Points: %d   Length: %d\n"),
                     p->lemsize, p->xstretch, p->finali, p->pathLength);
    SerialUSB.printf(F("Start: %5d,%5d   End: %5d,%5d\n"),
                     p->startpos.x, p->startpos.y, p->finalpos.x, p->finalpos.y);
    
    SerialUSB.printf(F("Name: %s   Start: %5d,%5d\n"), paths[2]->name, paths[2]->startpos.x, paths[2]->startpos.y);
    
    
    pathsconfigured = true;
}

bool goto_pos(vec2dint_t posint) {
    vec2d_t pos;
    pos.x=posint.x;
    pos.y=posint.y;
    return goto_pos(pos);
}

bool goto_pos(vec2d_t pos) {
    //Will not allow intentional moves past softlim but will
    // allow incidental past
    
    //Don't move beyond limits
    if (posOutsideLimits(pos))
        return false;
    
    if (inPosition(pos))
        return true;
    
    goalpos=pos;
    
    //While we arent in position loop
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
    while(!inPosition(pos) && !Buttons.down(ESTOP)) {
        if (xPID.WillCompute()||yPID.WillCompute()) {
            goalvel = vTowardGoal(goalpos, SLEW_SPEED);
            xPID.Compute();
            yPID.Compute();
            setqikspeeds(pidXOut, pidYOut);
        }
        
        if (hardLimitsTripped()) {
            bounceOffLimits();
            enterFail(LIMITS);
            break;
        }
    }
    xPID.SetMode(MANUAL);
    yPID.SetMode(MANUAL);
    
    setqikbrakes(QIK_PAUSE_BRAKE, QIK_PAUSE_BRAKE);

    currentpos.x=xQuad.read();
    currentpos.y=yQuad.read();
    
    return inPosition();
}


void deceltest(char brake_amt) {

    if (!calibrated || currentstate!=IDLE) {
        SerialUSB.println(F("Must be calibrated and idle"));
        return;
    }
    
    vec2d_t pos, stoppos, vel, decel, dpos,maxvel={0,0};
    unsigned long tic, temp;
    
    lcddisplay.display(F("Running Decel Test"));
    SerialUSB.println(F("Running Decel Test"));
    
    //Go to start of test position
    if (!goto_pos(negSoftLim) || currentstate==FAIL) {
        SerialUSB.println(F("Uable to achieve start position"));
        return;
    }
    
    //Stop moving pos
    stoppos.x=maxpos.x/4;
    stoppos.y=maxpos.y/4;
    
    //Full Speed
    SerialUSB.println(F("#x vx 20ms increments"));
    pos.x=xQuad.read();
    setqikspeeds(QIK_MAX_SPEED, 0);
    
    //Wait till quarter travel, monitoring speed
    while (pos.x<stoppos.x && !posLX()) {
        delay(20);
        temp=xQuad.read();
        vel.x=(temp-pos.x)/20.0;
        SerialUSB.printf(F("%d %g\n"), temp, vel.x);
        maxvel.x = max(vel.x, maxvel.x);
        pos.x=temp;
    }
    
    if (posLX()) {
        bounceOffLimits();
        SerialUSB.println(F("Hit X limit during test"));
        return;
    }
    
    //Halt
    if (brake_amt==0) Qik.setM0Speed(0);
    else Qik.setM0Brake(brake_amt);
    tic=micros();
    
    //Wait until stopped
    while(dpos.x!=0) {
        delay(1);
        temp=xQuad.read();
        dpos.x=temp-pos.x;
        pos.x=temp;
    }
    decel.x=vel.x*1000/(tic-micros());
    
    if (posLX()) {
        SerialUSB.println(F("Hit X limit during test"));
        return;
    }
    
    //Full Speed
    SerialUSB.println(F("#x vx 20ms increments"));
    setqikspeeds(0, QIK_MAX_SPEED);
    
    //Wait till quarter travel, monitoring speed
    pos.y=yQuad.read();
    while (pos.y<stoppos.y && !posLY()) {
        delay(20);
        temp=yQuad.read();
        vel.y=(temp-pos.y)/20.0;
        SerialUSB.printf(F("%d %g\n"), temp, vel.x);
        maxvel.y = max(vel.y, maxvel.y);
        pos.y=temp;
    }
    
    if (posLY()) {
        bounceOffLimits();
        SerialUSB.println(F("Hit Y limit during test"));
        return;
    }
    
    //Halt
    if (brake_amt==0) Qik.setM1Speed(0);
    else Qik.setM1Brake(brake_amt);
    tic=micros();

    //Wait until stopped
    while(dpos.y!=0) {
        delay(1);
        temp=yQuad.read();
        dpos.y=temp-pos.y;
        pos.y=temp;
    }
    decel.y=vel.y*1000/(tic-micros());
    
    if (posLY()) {
        SerialUSB.println(F("Hit Y limit during test"));
        return;
    }
    
    SerialUSB.printf(F("Max speed: %6.3g, %6.3g cts/ms"), maxvel.x, maxvel.y);
    SerialUSB.printf(F("Decel at %3d brake: %g, %g cts/ms^2"),brake_amt, decel.x,decel.y);
}


bool inPosition_x() {
    return abs(xQuad.read()-goalpos.x) < POSITION_TOLERANCE;
}

bool inPosition_y() {
    return abs(yQuad.read()-goalpos.y) < POSITION_TOLERANCE;
}

inline bool inPosition() {
    return inPosition_x() && inPosition_y();
}

bool inPosition(vec2d_t pos) {
    return (abs(xQuad.read()-pos.x) < POSITION_TOLERANCE &&
            abs(yQuad.read()-pos.y) < POSITION_TOLERANCE);
}

bool inPosition(vec2dint_t pos) {
    return (abs(xQuad.read()-pos.x) < POSITION_TOLERANCE &&
            abs(yQuad.read()-pos.y) < POSITION_TOLERANCE);
}


vec2d_t vTowardGoal(vec2d_t goal, double speed) {
    
    vec2d_t ret={0,0};
    
    double dx = goal.x-xQuad.read();
    double dy = goal.y-yQuad.read();
    double mag;
    
    if (abs(dx) <= POSITION_TOLERANCE)
        dx = 0.0;
    if (abs(dy) <= POSITION_TOLERANCE)
        dy = 0.0;
    
    if ((dy==0) && (dx==0))
        return ret;
    else if (dy==0) {
        ret.x = (dx > 0) ? speed: -speed;
    } else if (dx==0) {
        ret.y = (dy > 0) ? speed: -speed;
    } else {
        mag = sqrt(dy*dy + dx*dx);
        ret.x = dx*speed/mag;
        ret.y = dy*speed/mag;
    }
    return ret;
}



void autotunePID() {
    
    //at maxspeed we can cross the travel in ~{51314, 63996}/10.6667 ~ 4.8s and 6s
    // or about 480 ~ 600 PID loops. We will run at half speed so tune no more than 800 cycles
    // should take about 8 seconds
    
    PIDAutotuner tunerX = PIDAutotuner(), tunerY = PIDAutotuner();
    unsigned long microseconds, prevMicroseconds;
    double outputX, outputY;
    vec2d_t pos = {negSoftLim.x, negSoftLim.y};
    
    if (!goto_pos(pos)){
        SerialUSB.println(F("Could not attain autotuning start position"));
        return;
    }
    
    currentpos.x=xQuad.read();
    currentpos.y=yQuad.read();
    
    // Set the target value to tune to
    tunerX.setTargetInputValue(MAX_SPEED/2);
    tunerX.setLoopInterval(PID_INTERVAL_MS*1000);
    tunerX.setOutputRange(QIK_MIN_SPEED, QIK_MAX_SPEED);
    tunerX.setTuningCycles(800);
    tunerX.setZNMode(PIDAutotuner::znModeBasicPID);
    
    tunerY.setTargetInputValue(MAX_SPEED/2);
    tunerY.setLoopInterval(PID_INTERVAL_MS*1000);
    tunerY.setOutputRange(QIK_MIN_SPEED, QIK_MAX_SPEED);
    tunerY.setTuningCycles(800);
    tunerY.setZNMode(PIDAutotuner::znModeBasicPID);
    
    // This must be called immediately before the tuning loop
    SerialUSB.printf(F("#us x y vx vy"));
    tunerX.startTuningLoop();
    tunerY.startTuningLoop();
    while (!tunerX.isFinished() || tunerY.isFinished()) {
        
        microseconds = micros();
        if (microseconds-prevMicroseconds>(PID_INTERVAL_MS*1000)) {
            prevMicroseconds = microseconds;
        
            int32_t x = xQuad.read();
            int32_t y = yQuad.read();
            currentvel.x = (x-currentpos.x) / PID_INTERVAL_MS;
            currentvel.y = (y-currentpos.y) / PID_INTERVAL_MS;
            currentpos.x = x;
            currentpos.y = y;
            SerialUSB.printf(F("%d %d %d %g %g\n"), microseconds, currentpos.x, currentpos.y, currentvel.x, currentvel.y);
            /*
            computedpos.x += currentvel.x*PID_INTERVAL_MS;
            computedpos.y += currentvel.y*PID_INTERVAL_MS;
            disttogoal = dist(goalpos, currentpos);
            followerror.x = computedpos.x-currentpos.x;
            followerror.y = computedpos.y-currentpos.y;
            timeSincePosvel = millis();
            */

            // Call tunePID() with the input value
            outputX = tunerX.tunePID(currentvel.x);
            outputY = tunerY.tunePID(currentvel.y);
            outputX = max(round(outputX),QIK_MIN_SPEED);
            outputY = max(round(outputY),QIK_MIN_SPEED);
            outputX = min(round(outputX),QIK_MAX_SPEED);
            outputY = min(round(outputY),QIK_MAX_SPEED);
            setqikspeeds((int) outputX, outputY);
        }
        
        if (anyLimitTripped()) {
            ehalt();
            bounceOffLimits();
            break;
        }
    }
    
    setqikspeeds(0, 0);
    
    // Get PID gains - set your PID controller's gains to these
    SerialUSB.printf(F("PIDx: %g %g %g"), tunerX.getKp(), tunerX.getKi(), tunerX.getKd());
    SerialUSB.printf(F("PIDy: %g %g %g"), tunerY.getKp(), tunerY.getKi(), tunerY.getKd());

}

