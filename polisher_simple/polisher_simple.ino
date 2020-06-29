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

 Serial: 0 (RX) and 1 (TX)  USB-SERIAL
 Serial1: 19 (RX) and 18 (TX)   Qik
 Serial2: 17 (RX) and 16 (TX)
 Serial3: 15 (RX) and 14 (TX)   LCD
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

#define QIK_ERROR_PIN 17
#define QIK_RESET_PIN 42
#define X_QUADA_PIN 36
#define X_QUADB_PIN 34
#define Y_QUADA_PIN 28
#define Y_QUADB_PIN 26

#define LEFT_BUTTON_PIN 9
#define RIGHT_BUTTON_PIN 11
#define OK_BUTTON_PIN 10
#define ESTOP_BUTTON_PIN 12
#define LED_PIN 13

#define NEG_X_LIM_PIN 27
#define NEG_Y_LIM_PIN 23
#define POS_X_LIM_PIN 29
#define POS_Y_LIM_PIN 25

#define SPEED_PIN A0

#define LED_UNCAL_T 50
#define IDLE_LED_T 200
#define IDLESOFTLIM_LED_T 200
#define FAIL_LED_T 60
#define RUN_LED_T 300

#define QIK_BAUD_RATE 115200
#define QIK_MIN_SPEED -127
#define QIK_MAX_SPEED 127
#define QIK_FULL_BRAKE 127
#define QIK_PAUSE_BRAKE 95
#define SWITCH_POLL_US 1000

//This is a maximum per axis speed
//#define MAX_COUNTS_PER_MS 11.12   // about 6.8 cm/s per axis, new motors, no load
#define MAX_COUNTS_PER_MS_X 10.25   // as built, old motors
#define MAX_COUNTS_PER_MS_Y 10.70   // as built, old motors


/*
 From Deceltest:
 10.550, 11.300 (OLD MOTORS, ASSEMBLED)
 11.25375 11.001249999999999 (NEW MOTORS NO LOAD)
 Decel at   0 brake: 0.0176213, 0.0122463 cts/ms^2  581,910ms to dead stop
 Decel at  32 brake: 0.0203323, 0.0143966 cts/ms^2  501,774ms
 Decel at  95 brake: 0.0480106, 0.0388041 cts/ms^2  212,289ms
 Decel at 127 brake: 0.0915430, 0.0871765 cts/ms^2  111,128ms  NOT REMOTELY LINEAR!!!!
 */
#define ADC_MAX 1023.0
#define ADC_TO_SPEED  MAX_COUNTS_PER_MS_X/ADC_MAX
#define IN_PER_COUNT 0.00024157
#define COUNTS_PER_IN 1.0/IN_PER_COUNT
#define MM_PER_COUNT 0.00613100
#define SPEED_TO_MMM 367.86  //convert cts/ms to mm/min
#define SPEED_TO_QIK_X QIK_MAX_SPEED/MAX_COUNTS_PER_MS_X //convert cts/ms to ~QIK value
#define SPEED_TO_QIK_Y QIK_MAX_SPEED/MAX_COUNTS_PER_MS_X
#define QIK_MIN_ATTAINABLE 7

//These are maximum vector speeds
#define CALIBRATE_SPEED 8.5  //in //MarkI equiv. was about 10.09
#define SLEW_SPEED 8.0  //in units OF COUNTS/ms

#define SOFT_LIMIT_COUNTS 2300 //about 14mm
#define POSITION_TOLERANCE 238 //238 is from MarkI ~1.5mm, 16.311 = 0.1mm
#define PATH_TOLERANCE 120

#define LOAD_Y_POSITION             15L*maxpos.y/16L   //from MarkI
#define LOAD_X_POSITION             maxpos.x/2         //from MarkI


//old polisher the waypoints were about .14 mm apart
//max p error 654 um. avg 326 um min. detectable speed ~1.15/127
// so speeds below .1 cts/ms (~.61 mm/s) are not readable
#define PID_INTERVAL_MS 150
#define PATH_INTERVAL_MS 10
#define SPEED_INTERVAL_MS 10

#define DEFAULT_PATH_ID 1

#define NUM_PATHS 7

#define MAX_MENU_LENGTH NUM_PATHS+1
#define CONTINUE_INDEX NUM_PATHS

#define SCREEN_REFRESH_MS 700



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
unsigned long housekeeepingTic=0, timeOfPosVel=0, printTic=0, nPID=0, pathAdvanceTime;
unsigned long computetimes[]={0,0,0}, ncomputetimes[]={0,0,0}, maxtimes[]={0,0,0};
bool calibrated = false, pathsconfigured=false, failRecoveryAllowed=false, justbooted=true;
int freeram[]={0,0,0};

enum State {RUN, CFG, FAIL, IDLE, UNCAL};
enum FailReason {EHALT, SOFTLIM, LIMITS, CALFAIL, SWBUG, QIKERR};

State currentstate = UNCAL, laststate=UNCAL;

int menuindex = 0;
int menulen = MAX_MENU_LENGTH - 1;

vec2d_t computedpos;
vec2d_t currentpos, ancientpos;
vec2d_t goalpos;
vec2d_t poserr;  //results from last inposition test
vec2d_t followerror;
double disttogoal;

vec2d_t currentvel;
vec2d_t goalvel;
double speedSetting=0.0;

unsigned long timeOfPosVel2=0, posvel_xinterval=0, posvel_yinterval=0;
vec2d_t currentpos2, currentvel2;


vec2dint_t qikcurrent={0,0};
vec2dint_t qikspeed={0,0};

vec2dint_t maxpos = {51287, 64102};     //From Mark 1: 51314, 63996 markII 51287, 64102

vec2dint_t negSoftLim = {SOFT_LIMIT_COUNTS, SOFT_LIMIT_COUNTS};     //About 5mm away based on Polisher Mark1
vec2dint_t posSoftLim = {51287-SOFT_LIMIT_COUNTS, 64102-SOFT_LIMIT_COUNTS};

double kPx=SPEED_TO_QIK_X, kIx=.5, kDx=1;
double kPy=SPEED_TO_QIK_Y, kIy=.5, kDy=1;
double pidXOut, pidYOut;
PID xPID = PID(&currentvel.x, &pidXOut, &goalvel.x, kPx, kIx, kDx, DIRECT);
PID yPID = PID(&currentvel.y, &pidYOut, &goalvel.y, kPy, kIy, kDy, DIRECT);

//Paths
PolishPath* currentpath;
PolishPath* paths[NUM_PATHS];


//Serial interface
char _cmdbuf[32];
SerialCommands cmdIO(&SerialUSB, _cmdbuf, sizeof(_cmdbuf), "\n", " ");
void cmd_unrecognized(SerialCommands* sender, const char* cmd) {
    sender->GetSerial()->print(F("Unrecognized command ["));
    sender->GetSerial()->print(cmd);
    sender->GetSerial()->println("]");
}
//-------------------------------
void tunepidx_cb(SerialCommands* sender) {PIDTuneLoopX();};
void badqik_cb(SerialCommands* sender) {
        Serial1.write(0x01);
        byte cmd[2];
        cmd[0] = 0x86;
        cmd[1] = 0xFF;
        Serial1.write(cmd, 2);
}
void report_cb(SerialCommands* sender) {report(sender->GetSerial());}
void qikget_cb(SerialCommands* sender) {tellqikparams(sender->GetSerial());}
void clearstat_cb(SerialCommands* sender) {
    sender->GetSerial()->println(F("Clearing stats."));
    for (int i=0;i++;i<3) {
        computetimes[i]=0;
        ncomputetimes[i]=0;
        maxtimes[i]=0;
    }
}
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
void minspeedtest_cb(SerialCommands* sender) {

    char* val_str = sender->Next();
    if ((val_str == NULL)) {
        sender->GetSerial()->println(F("Must specify value"));
        return;
    }
    unsigned int spd = min(max(atoi(val_str),0),50);
    sender->GetSerial()->printf(F("minspeed test: %d\n"), spd);
    minspeedtest(spd);
}

void deceltest_cb(SerialCommands* sender) {
    vec2d_t pos=currentpos;
    
    if (!calibrated) {
        SerialUSB.println(F("Must be calibrated!"));
        return;
    }
    
    sender->GetSerial()->println(F("Decel 0 brake"));
    lcddisplay.display(F("Running Movement Test"));
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
    lcddisplay.display(F("Movement Test Done"));
    
    goto_pos(pos);
}

SerialCommand cmd_report("S", report_cb);
SerialCommand cmd_getqik("QG", qikget_cb);
SerialCommand cmd_decel("Deceltest", deceltest_cb);
SerialCommand cmd_setqik("QS", qikset_cb);
SerialCommand cmd_clear("clear",clearstat_cb);
SerialCommand cmd_badqik("badqik",badqik_cb);
SerialCommand cmd_tunex("tunex", tunepidx_cb);
SerialCommand cmd_minspd("minspeed",minspeedtest_cb);


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
    lcddisplay.display(F("Booting..."));
    
    delay(2500);
    
    //Configure IO
    Buttons.begin(ButtonPins, 8);
    
    //Connect to Qik
    Qik.init(QIK_BAUD_RATE);

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
    cmdIO.AddCommand(&cmd_setqik);
    cmdIO.AddCommand(&cmd_clear);
    cmdIO.AddCommand(&cmd_badqik);
    cmdIO.AddCommand(&cmd_tunex);
    cmdIO.AddCommand(&cmd_minspd);
    
    //Enter uncalibrated state
    currentstate=UNCAL;
    led.fadeUp(LED_UNCAL_T);
    SerialUSB.println(F("Calibration Required     OK?"));
    lcddisplay.display(F("Calibration     Required     OK?"));
    justbooted=true;
}

/*////////////////////////////////////////////////////////////////////////////////
Main Loop
*/////////////////////////////////////////////////////////////////////////////////
void loop() {
    
    unsigned long tic, loopTic;
    bool ismore;
    
    if (justbooted) { //Placeholder
        justbooted=false;
    }
    
    loopTic = micros();
    
    //Guarantee paths are configured
    configure_paths();

    //Monitor Estop
    estopmonitor();
 
    //Monitor Limits
    if (hardLimitsTripped()) {
        bounceOffLimits();
        enterFail(LIMITS);
    }
    
    if (calibrated && softLimitsTripped()) {
        bounceOffSoftLimits();
        enterFail(SOFTLIM);
    }

    //LED
    led.update();
    
    //Handle Serial commands
    cmdIO.ReadSerial();
    
    //Monitor for Qik errors
    if (Qik.isError()) {
        enterFail(QIKERR);
    }
    
    //Update readings
    updateposvel();
    updateposvel2();
    if (housekeeepingTic - loopTic > 500000){
        qikcurrent.x=Qik.getM0CurrentMilliamps(); //~.175ms
        qikcurrent.y=Qik.getM1CurrentMilliamps();
        housekeeepingTic=loopTic;
    }

    if ((loopTic - printTic) > 5000000 ) {
      report(&SerialUSB);
      printTic=micros();
    }
    
    //Update Display
    displayScreen(false);
    
    
    //State machine
    switch(currentstate){
     
        case FAIL:   //We're done until a reset
            if (failRecoveryAllowed && Buttons.clickedsince(OK)) enterIdleFromFail();
            else delay(10);
            break;
     
        case UNCAL:  //We're uncalibrated
            if (Buttons.clickedsince(OK)) {
                calibrated = calibrate();
                if (calibrated) enterIdleFromUncal();
                else enterFail(CALFAIL);
            }
            break;

        case IDLE:   //We are idle
            if (!calibrated) {
                enterFail(SWBUG);
                break;
            }
            
            //Update menu selection
            menulen = (currentpath->isMore() && !currentpath->isStart()) ? MAX_MENU_LENGTH: MAX_MENU_LENGTH-1;
            menuindex -= (int) Buttons.clickedsince(LEFT);
            if (menuindex<0) menuindex = menulen-1;
            menuindex += (int) Buttons.clickedsince(RIGHT);
            if (menuindex >= menulen) menuindex %= menulen;

            //When the soft limits are tripped the screen is only showing that so ignore
            if (hardLimitsTripped()) {
                SerialUSB.println(F("IDLE: limit tripped clearing ok"));
                Buttons.clearChangeFlag(OK); //just to be thorough
            
            } else if (Buttons.clickedsince(OK)) {
                SerialUSB.println(F("IDLE: OK Pressed"));
                if (menuindex!=CONTINUE_INDEX){
                    
                    currentpath = paths[max(0,min(menuindex,NUM_PATHS-1))];
                    currentpath->reset(currentpos);
                    resetposvel();
                    
                    //if we are at start skip the starting point so we don't pause right away
                    if (inPosition(currentpath->current)) {
                        SerialUSB.println(F("Stepping forward in IDLE"));
                        currentpath->stepForward(speedSetting);
                    }
                }
                
                updategoal(currentpath->current);
                currentpath->print();
                enterRun();
            }
            break;

        case RUN:   //Movement is needed
            if (!calibrated) {
                enterFail(SWBUG);
                break;
            }
     
            if (Buttons.clickedsince(OK)) {
                SerialUSB.println(F("Pause button pressed, pasuing."));
                enterIdleFromRun(currentpath->isMore() && !currentpath->isStart());//verify isstarrt conndition no implicatiions
                break;
            }
            
            tic = micros();
            bool pastg = pastgoal();
            bool advance = (disttogoal < PATH_TOLERANCE) || pastg;//|| (disttogoal<PATH_INTERVAL_MS*speedSetting/2);
            if (advance){//||currentpath->goalIsBehind(currentpos)){
                
                SerialUSB.printf("\nAdvancing (Dist:%.0f  T/N/P/B: %d/%d/%d/%d). G:%.0f,%.0f D:%.0f,%.0f\n",
                                 disttogoal,
                                 disttogoal < PATH_TOLERANCE, disttogoal<PATH_INTERVAL_MS*speedSetting/2,
                                 pastg, currentpath->goalIsBehind(currentpos),
                                 goalpos.x,goalpos.y, goalpos.x-xQuad.read(),
                                 goalpos.y-yQuad.read());
                
                currentpath->print();
                
                
                if (currentpath->isEnd() && currentpath->autonextpath) {
                    currentpath = paths[max(0,min(currentpath->nextpath,NUM_PATHS-1))];
                    currentpath->reset(currentpos);
                    currentpath->skipstartpause=true;
                    updategoal(currentpath->current);
                    SerialUSB.println(F("Stepping to next path in RUN"));
                }
                else {
                    int skippedpoints = 0;
                    while (currentpath->goalIsBehind(currentpos) && currentpath->isMore() &&
                           !currentpath->isPausePoint()){
                        SerialUSB.printf("SKP%d G:%.0f,%.0f D:%.0f,%.0f V: %.2f,%.2f QV: %.1f,%.1f\n",
                                         currentpath->_i,
                                         goalpos.x,goalpos.y, goalpos.x-xQuad.read(),goalpos.y-yQuad.read(),
                                         goalvel.x, goalvel.y, pidXOut, pidYOut);
                        currentpath->stepForward(speedSetting);
                        updategoal(currentpath->current);
                        skippedpoints++;
                    }
                    if (skippedpoints)
                        SerialUSB.printf("!!!!!SKIPPED %d PATH POINTS. LOWER SPEED!!!!!", skippedpoints);

                    if (currentpath->isPausePoint()) enterIdleFromRun(currentpath->isMore());
                    
                    if (currentpath->isMore() && skippedpoints==0) {
                        
                        SerialUSB.printf("%d G:%.0f,%.0f D:%.0f,%.0f V: %.2f,%.2f QV: %.1f,%.1f\n",
                                         currentpath->_i,
                                         goalpos.x,goalpos.y, goalpos.x-xQuad.read(),goalpos.y-yQuad.read(),
                                         goalvel.x, goalvel.y, pidXOut, pidYOut);
                        
                        SerialUSB.printf("nPID:%d dt:%d ms\n", nPID, millis()-pathAdvanceTime);
                        
                        currentpath->stepForward(speedSetting);
                        updategoal(currentpath->current);
                    }
                }
                nPID=0;
                pathAdvanceTime=millis();
                if (currentstate!=RUN) break;

            }
            
            if (advance || laststate!=RUN) {
                speedSetting = (currentpath->isStart() && !currentpath->skipstartpause) ? SLEW_SPEED:readSpeed();
                goalvel = currentpath->vTowardGoal((vec2d_t) {xQuad.read(), yQuad.read()}, speedSetting);

                pidXOut = goalvel.x*SPEED_TO_QIK_X;
                pidYOut = goalvel.y*SPEED_TO_QIK_Y;
                
                
                if (abs(pidXOut)<QIK_MIN_ATTAINABLE && abs(pidXOut)!=0) {
                    pidXOut=pidXOut<0? -QIK_MIN_ATTAINABLE:QIK_MIN_ATTAINABLE;
                }
                if (abs(pidYOut)<QIK_MIN_ATTAINABLE && abs(pidYOut)!=0) {
                    pidYOut=pidYOut<0? -QIK_MIN_ATTAINABLE:QIK_MIN_ATTAINABLE;
                }
                
                
                setqikspeeds(pidXOut, pidYOut);
            }
            
            /*
            if (xPID.Compute() || yPID.Compute()) {
                setqikspeeds(pidXOut, pidYOut);
                nPID++;
            }
            */
            
            tic = micros()-tic;
            computetimes[1] += tic;
            ncomputetimes[1]++;
            freeram[1]=freeMemory();
            if (tic>maxtimes[1]) maxtimes[1] = tic;
            break;
    }
    
    loopTic = micros()-loopTic;
    computetimes[2] += loopTic;
    ncomputetimes[2]++;
    freeram[2]=freeMemory();
    if (loopTic>maxtimes[2]) maxtimes[2] = loopTic;
    
}

bool pastgoal() {
    bool xpast= (((xQuad.read()>=goalpos.x) && (goalvel.x>0)) ||
                 ((xQuad.read()<=goalpos.x) && (goalvel.x<0)));
    bool ypast= (((yQuad.read()>=goalpos.y) && (goalvel.y>0)) ||
                 ((yQuad.read()<=goalpos.y) && (goalvel.y<0)));
    return xpast && ypast;
    
}




bool updateposvel2(){
    //true if updated
    
    int32_t x, y;
    unsigned long tic = micros();
    
    if ((tic-timeOfPosVel2)>=2000) {
        x = xQuad.read();
        y = yQuad.read();
        timeOfPosVel2 = micros();
        if (x==currentpos2.x) {
            posvel_xinterval+=2.0;
            currentvel2.x = posvel_xinterval>10 ? 0 : (x-ancientpos.x) / posvel_xinterval;
        } else {
             posvel_xinterval=1.0;
             currentvel2.x = (x-currentpos2.x) / 2.0;
             ancientpos.x = currentpos2.x;
        }
        
        if (y==currentpos2.y) {
            posvel_yinterval+=1;
            currentvel2.y = posvel_yinterval>10 ? 0 : (y-ancientpos.y) / posvel_yinterval;
        } else {
            posvel_yinterval=2.0;
            currentvel2.y = (y-currentpos2.y) / 2.0;
            ancientpos.y = currentpos2.y;
        }
        
        currentpos2.x = x;
        currentpos2.y = y;
        timeOfPosVel2=tic;
        
        return true;
    } else return false;
    
}

bool updateposvel(){
    //true if updated

    int32_t x, y;
    unsigned long tic = micros();
    
    if ((tic-timeOfPosVel)>=(SPEED_INTERVAL_MS*1000)) {
        x = xQuad.read();
        y = yQuad.read();
        timeOfPosVel = micros();
        /*
        if (x==currentpos.x) {
            posvel_xinterval+=SPEED_INTERVAL_MS;
            currentvel.x = (x-ancientpos.x) / posvel_xinterval;
         } else {
            posvel_xinterval=SPEED_INTERVAL_MS;
            currentvel.x = (x-currentpos.x) / posvel_xinterval;
            ancientpos.x = currentpos.x;
        }
        
        if (abs(currentvel.x)<MIN_SPEED_DETECTABLE) currentvel.x=0;
        */
        currentvel.x = (x-currentpos.x) / SPEED_INTERVAL_MS;
        currentvel.y = (y-currentpos.y) / SPEED_INTERVAL_MS;
        currentpos.x = x;
        currentpos.y = y;
        computedpos.x += currentvel.x*SPEED_INTERVAL_MS;
        computedpos.y += currentvel.y*SPEED_INTERVAL_MS;
        disttogoal = dist(goalpos, currentpos);
        followerror.x = computedpos.x-currentpos.x;
        followerror.y = computedpos.y-currentpos.y;

        tic = micros()-tic;
    
        computetimes[0] += tic;  //47 mean 85 max (irrelevant!!!)
        ncomputetimes[0]++;
        freeram[0]=freeMemory();
        if (tic>maxtimes[0]) maxtimes[0] = tic;
        return true;
    } else return false;
    
}

void updategoal(vec2d_t goal){
    vec2d_t pos = {(double) xQuad.read(), (double) yQuad.read()};
    goalpos=goal;
    disttogoal = dist(goalpos, pos);
}

void resetposvel() {
    ancientpos.x=currentpos.x=computedpos.x=xQuad.read();
    ancientpos.x=currentpos.y=computedpos.y=yQuad.read();
    currentvel.x=currentvel.y=0;
}

inline void setqikspeeds(double x, double y) { setqikspeeds((int) round(x),(int) round(y));}
void setqikspeeds(int x, int y) {
    x = negLX() ? max(x,0):x;
    x = posLX() ? min(x,0):x;
    y = negLY() ? max(y,0):y;
    y = posLY() ? min(y,0):y;
//    SerialUSB.printf("Setting speeds: %d, %d\n",x,y);
    Qik.setM0Speed(x);
    Qik.setM1Speed(y);
    Serial1.flush();
    qikspeed.x=x;
    qikspeed.y=y;
}

void setqikbrakes(unsigned char x, unsigned char y){
    Qik.setBrakes(x, y);
//    SerialUSB.printf("Setting breaks: %d, %d\n",x,y);
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
    stream->println(F("\n===================================="));
    stream->println(F("---State---"));
    stream->printf(F("State: %4s  -XLim: %1d  +XLim: %1d  -YLim: %1d  +YLim: %1d  Cailb: %1d  Menu: %1d\n"),
                   statestr, negLX(), posLX(), negLY(), posLY(), calibrated, menuindex);
    stream->printf(F("-XSLim: %1d  +XSLim: %1d  -YSLim: %1d  +YSLim: %1d\n"),
                   negSLX(), posSLX(), negSLY(), posSLY());
    stream->printf(F("XEnc: %d  YEnc: %d\n"), xQuad.read(),yQuad.read());
    stream->printf(F("Buttons: L: U%d D%d Cl%d Re%d  R %d  OK %d  ES %d \n"),
                   Buttons.up(LEFT), Buttons.down(LEFT), Buttons.clicked(LEFT), Buttons.released(LEFT),
                   Buttons.up(RIGHT), Buttons.up(OK), Buttons.up(ESTOP));
    stream->printf(F("MaxPos: %5d, %5d   SWLIM: %5d, %5d  %5d, %5d\n"),
                   maxpos.x, maxpos.y, negSoftLim.x, negSoftLim.y,
                   posSoftLim.x,posSoftLim.y);
    
    stream->println(F("---Qik---"));
    tellqikparams(stream);
    stream->printf(F("Speeds: %4d, %4d   Currents: %5d, %5d mA   EPin: %d   Errors:%d\n"),
                   qikspeed.x, qikspeed.y, qikcurrent.x, qikcurrent.y, Qik.isError(), Qik.getErrors());

    stream->println(F("---DEBUG---"));
    stream->printf(F("T0: %5d (%5d) us  T1: %5d (%5d) us  T2: %5d (%5d) us  \n"),
                   computetimes[0]/ncomputetimes[0], maxtimes[0], computetimes[1]/ncomputetimes[1],
                   maxtimes[1], computetimes[2]/ncomputetimes[2], maxtimes[2]);
    stream->printf(F("RAM1: %5db  RAM2: %5db  RAM3: %5db\n"), freeram[0], freeram[1], freeram[2]);
    
    stream->println(F("---PID---"));
    stream->printf(F("xPID: Kp=%10g  Ki=%10g  Kd=%10g\n"), xPID.GetKp(), xPID.GetKi(), xPID.GetKd());
    stream->printf(F("yPID: Kp=%10g  Ki=%10g  Kd=%10g\n"), yPID.GetKp(), yPID.GetKi(), yPID.GetKd());
    
    stream->println(F("---Vectors---"));
    stream->printf(F("Position: %5g, %5g   CompDelta: %5g, %5g   FollowErr: %5g \n"),
                   currentpos.x, currentpos.y, currentpos.x-computedpos.x, currentpos.y-computedpos.y,
                   followerror);
    stream->printf(F("Dest: %5g, %5g  Dist: %.2f mm Speed: %5.2g   ASpeed: %5.2g ct/ms\n"),
                   goalpos.x, goalpos.y, disttogoal*MM_PER_COUNT, speedSetting, readSpeed());
    stream->printf(F("Velocity: %6.2g, %6.2g (%6.2g, %6.2g)   Error: (%6.2g, %6.2g)  ct/ms\n"),
                   currentvel.x, currentvel.y, goalvel.x, goalvel.y,
                   goalvel.x-currentvel.x, goalvel.y-currentvel.y);
    stream->printf(F("Velocity2: %6.2g, %6.2gct/ms\n"),currentvel2.x, currentvel2.y);
    
    stream->println(F("---Path---"));
    stream->printf(F("Path: %s %d/%d %d %%  %f min\n"),
                   currentpath->name, currentpath->_i, currentpath->finali,
                   currentpath->percentComplete, currentpath->timeLeft);
    
    stream->printf(F("Message took %5d us.\n\n"), micros()-tic);
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
        if (param&0b001) stream->print(F(" serial "));
        if (param&0b010) stream->print(F(" current "));
        if (param&0b100) stream->print(F(" fault"));
    } else stream->print(F(" none"));
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_SERIAL_TIMEOUT);
    param = 250*(param&&0b1111)*(((unsigned int)2)<<((param&111000)>>4));
    stream->printf(F("\nSerial timout is %6d ms\n"), param);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M0_ACCELERATION);
    param1 = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M1_ACCELERATION);
    stream->printf(F("Motor Accel %3d, %3d\n"), param, param1);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M0_BRAKE_DURATION);
    param1 = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M1_BRAKE_DURATION);
    stream->printf(F("Motor brake times: %3d, %3d x10ms\n"), param, param1);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M0_CURRENT_LIMIT_DIV_2);
    param1 = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M1_CURRENT_LIMIT_DIV_2);
    stream->printf(F("Motor current limits: %5d, %5d mA\n"), param*300, param1*300);
    
    param = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M0_CURRENT_LIMIT_RESPONSE);
    param1 = Qik.getConfigurationParameter(QIK_CONFIG_MOTOR_M1_CURRENT_LIMIT_RESPONSE);
    stream->printf(F("Motor current limit response: %3d, %3d\n"), param, param1);
}

char softLimCode() {
    return (((char) negSLX())    || ((char) posSLX())<<1 ||
            ((char) negSLY())<<2 || ((char) posSLY())<<3 );
}


void displayScreen(bool force) {
    
    long t = millis();
    
    if (!force && (abs(t-lastScreenRefresh) < SCREEN_REFRESH_MS)) return;
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
    lcddisplay.clear();
    //lcddisplay.serial->printf(F("Run  X%4.0f Y%4.0f%-6s %3d%% "),
    //                          currentpos.x*MM_PER_COUNT,currentpos.y*MM_PER_COUNT,
    //                          currentpath->name, currentpath->percentComplete);
    lcddisplay.serial->printf(F("Pause.       OK?%-6s %3d%% "),
                              currentpath->name, currentpath->percentComplete);
    if (currentpath->timeLeft < 9.95) lcddisplay.serial->printf("%3.1fm", currentpath->timeLeft);
    else lcddisplay.serial->printf("%3d",(int) round(currentpath->timeLeft));
}

void displayIdleScreen() {
    
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
        if (inPosition(paths[menuindex]->startpos))
            lcddisplay.serial->printf(F("Ready.       Go?%-12s%3d%%"), paths[menuindex]->name, 0);
        else
            lcddisplay.serial->printf(F("<-   %-6s   ->Go to start. OK?"), paths[menuindex]->name);
    } else
        lcddisplay.serial->printf(F("Resume.      OK?%-12s%3d%%"),
                                  currentpath->name, currentpath->percentComplete);
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
inline bool posSLX() {return xQuad.read()>posSoftLim.x;}
inline bool negSLY() {return yQuad.read()<negSoftLim.y;}
inline bool posSLY() {return yQuad.read()>posSoftLim.y;}

inline bool negLX() {return Buttons.down(NXLIM);}
inline bool posLX() {return Buttons.down(PXLIM);}
inline bool negLY() {return Buttons.down(NYLIM);}
inline bool posLY() {return Buttons.down(PYLIM);}

double readSpeed(){
    return analogRead(SPEED_PIN)*ADC_TO_SPEED;
}

void bounceOffLimits() {
    ehalt();
    if (negLX() && !(posLX() || posSLX())) {
        Qik.setM0Speed(QIK_MAX_SPEED);
        SerialUSB.println("QikM0 +max (BOL)");
        qikspeed.x=QIK_MAX_SPEED;
    }
    if (posLX() && !(negLX()||negSLX())) {
        Qik.setM0Speed(-QIK_MAX_SPEED);
        SerialUSB.println("QikM0 -max (BOL)");
        qikspeed.x=-QIK_MAX_SPEED;
    }
    if (negLY() && !(posLY()||posSLY())) {
        Qik.setM1Speed(QIK_MAX_SPEED);
        SerialUSB.println("QikM1 +max (BOL)");
        qikspeed.y=QIK_MAX_SPEED;
    }
    if (posLY() && !(negLY()||negSLY())) {
        Qik.setM1Speed(-QIK_MAX_SPEED);
        SerialUSB.println("QikM0 -max (BOL)");
        qikspeed.y=-QIK_MAX_SPEED;
    }
    delay(150);
    setqikspeeds(0,0);
}

void bounceOffSoftLimits() {
    ehalt();
    if (negSLX() && !(posLX() || posSLX())) {
        Qik.setM0Speed(QIK_MAX_SPEED);
        SerialUSB.println("QikM0 +max (BOL)");
        qikspeed.x=QIK_MAX_SPEED;
    }
    if (posSLX() && !(negLX()||negSLX())) {
        Qik.setM0Speed(-QIK_MAX_SPEED);
        SerialUSB.println("QikM0 -max (BOL)");
        qikspeed.x=-QIK_MAX_SPEED;
    }
    if (negSLY() && !(posLY()||posSLY())) {
        Qik.setM1Speed(QIK_MAX_SPEED);
        SerialUSB.println("QikM1 +max (BOL)");
        qikspeed.y=QIK_MAX_SPEED;
    }
    if (posSLY() && !(negLY()||negSLY())) {
        Qik.setM1Speed(-QIK_MAX_SPEED);
        SerialUSB.println("QikM0 -max (BOL)");
        qikspeed.y=-QIK_MAX_SPEED;
    }
    delay(150);
    setqikspeeds(0,0);
}



void enterIdleFromUncal(){
    laststate=currentstate;
    currentstate=IDLE;
    configure_paths();
    currentpath=paths[0];
    led.breathe(IDLE_LED_T);
    Buttons.clearChangeFlag(LEFT);
    Buttons.clearChangeFlag(RIGHT);
    Buttons.clearChangeFlag(OK);
    goalvel.x=0;
    goalvel.y=0;
    menuindex = 0;
    menulen = MAX_MENU_LENGTH-1;
    displayScreen(true);
    SerialUSB.println(F("Enter IDLE from UNCAL"));
}


void enterIdleFromFail() {
    laststate=currentstate;
    currentstate=IDLE;
    led.breathe(IDLE_LED_T);
    xPID.SetMode(MANUAL);
    yPID.SetMode(MANUAL);
    setqikbrakes(QIK_FULL_BRAKE, QIK_FULL_BRAKE);
    
    Buttons.clearChangeFlag(LEFT);
    Buttons.clearChangeFlag(RIGHT);
    Buttons.clearChangeFlag(OK);
    
    menulen = MAX_MENU_LENGTH-1; //We can't continue anything from here
    menuindex = 0;
    SerialUSB.println(F("Enter IDLE from FAIL"));
    displayScreen(true);
}

void enterIdleFromRun(bool allowcontinue) {
    laststate=currentstate;
    currentstate=IDLE;
    SerialUSB.printf(F("Enter IDLE from RUN: Continue ok: %d\n"), allowcontinue);
    SerialUSB.printf(F("C: %.0f,%.0f G: %.0f,%.0f MM2G: %.2f, %.2f PP:%d\n"),
                     currentpos.x,currentpos.y,goalpos.x,goalpos.y,
                     MM_PER_COUNT*(goalpos.x-xQuad.read()),MM_PER_COUNT*(goalpos.y-yQuad.read()),
                     currentpath->isPausePoint());
    led.breathe(IDLE_LED_T);
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
    SerialUSB.println(F("Entering RUN"));
    SerialUSB.printf(F("C: %.0f,%.0f G: %.0f,%.0f MM2G: %.2f, %.2f PP:%d\n"),
                     currentpos.x,currentpos.y,goalpos.x,goalpos.y,
                     MM_PER_COUNT*(goalpos.x-xQuad.read()),MM_PER_COUNT*(goalpos.y-yQuad.read()),
                     currentpath->isPausePoint());
    laststate=currentstate;
    currentstate=RUN;
    led.blink(RUN_LED_T);
    displayScreen(true);
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
    nPID=0;
    
}

void enterFail(FailReason reason){
    SerialUSB.println(F("Entering FAIL"));
    laststate=currentstate;
    currentstate=FAIL;
    led.blink(FAIL_LED_T);
    ehalt();
    failRecoveryAllowed=false;
    switch (reason) {
        case EHALT:
            lcddisplay.display(F("E-Stop Pressed  Clear Error. OK?"));
            failRecoveryAllowed=true;
            break;
        case SOFTLIM:
            lcddisplay.display(F("SLimit: "));
            if (negSLX()||posSLX()) {
                if (negSLX()) lcddisplay.print(F("-"));
                if (posSLX()) lcddisplay.print(F("+"));
                lcddisplay.print(F("X"));
            } else lcddisplay.print(F("NoX"));
            if (negSLY()||posSLY()) {
                if (negSLY()) lcddisplay.print(F("-"));
                if (posSLY()) lcddisplay.print(F("+"));
                lcddisplay.print(F("Y"));
            } else lcddisplay.print(F("NoY"));
            lcddisplay.print2(F("Cleared?     OK?"));
            failRecoveryAllowed=true;
            break;
        case LIMITS:
            lcddisplay.display(F("HLimit: "));
            if (negLX()||posLX()) {
                if (negLX()) lcddisplay.print(F("-"));
                if (posLX()) lcddisplay.print(F("+"));
                lcddisplay.print(F("X"));
            } else lcddisplay.print(F("NoX"));
            if (negLY()||posLY()) {
                if (negLY()) lcddisplay.print(F("-"));
                if (posLY()) lcddisplay.print(F("+"));
                lcddisplay.print(F("Y"));
            } else lcddisplay.print(F("NoY"));
            lcddisplay.print2(F("Cleared?     OK?"));
            failRecoveryAllowed=true;
            break;
        case CALFAIL:
            lcddisplay.display(F("Calibration Fail   Fix Problem.  "));
            break;
        case SWBUG:
            lcddisplay.display(F("Software Failure   Fix Problem.  "));
            break;
        case QIKERR:
            lcddisplay.display(F("Qik Error: "));
            lcddisplay.serial->printf("%5d", Qik.getErrors());
            lcddisplay.print2(F("Checked?     OK?"));
            failRecoveryAllowed=true;
            break;
        default:
            lcddisplay.display(F("Unknown Failure    Fix Problem.  "));
            break;
    }
}

void ehalt() {
    xPID.SetMode(MANUAL);
    xPID.SetMode(MANUAL);
    setqikbrakes(QIK_FULL_BRAKE, QIK_FULL_BRAKE);
}

    
bool calibrate() {
    
    vec2d_t pos;
    int calspeed = min(round(CALIBRATE_SPEED*SPEED_TO_QIK_X), readSpeed());
    Buttons.clearChangeFlag(ESTOP);

    /*
    xQuad.write(maxpos.x/2);
    yQuad.write(maxpos.y/2);
    calibrated=true;
    return true;
    */
    
    //Calibrate the y-axis
    lcddisplay.display(F("Calibrating Y..."));
    SerialUSB.println(F("Driving to Y-"));
    if(!negLY()) {
        Qik.setM1Speed(-calspeed);
        //SerialUSB.print("YPos=[");
        while(!negLY() && !estopmonitor()) {
            //SerialUSB.printf("%d,%d,%d\n", micros(), xQuad.read(), yQuad.read());
            delay(1);
        }
        Qik.setM1Brake(QIK_FULL_BRAKE);
    }
    if (currentstate==FAIL) return false;
    yQuad.write(0);
    
    SerialUSB.printf("Y zeroed: %d,%d\n", xQuad.read(), yQuad.read());
    
    SerialUSB.println(F("Driving to Y+"));
    Qik.setM1Speed(calspeed);
    while(!posLY() && !estopmonitor()) {
        //SerialUSB.printf("%d,%d,%d\n", micros(), xQuad.read(), yQuad.read());
        delay(1);
    }
    Qik.setM1Brake(QIK_FULL_BRAKE);
    if (currentstate==FAIL) {
        return false;
    }
    maxpos.y=yQuad.read();
    SerialUSB.printf(F("Ymax %d\n"), maxpos.y);
    
    //Go to y center
    SerialUSB.println(F("Driving to Y center"));
    pos.y=maxpos.y/2;
    pos.x=xQuad.read();
    if (!goto_pos(pos))
        return false;
    
    SerialUSB.printf("At: %d,%d\n", xQuad.read(), yQuad.read());
    
    //Calibrate the x axis
    lcddisplay.display(F("Calibrating X..."));
    SerialUSB.println(F("Driving to X-"));
    if(!negLX()) {
        Qik.setM0Speed(-calspeed);
        //SerialUSB.print("XPos=[");
        while(!negLX() && !estopmonitor()) {
            //SerialUSB.printf("%d,%d,%d\n", micros(), xQuad.read(), yQuad.read());
            delay(1);
        }
        Qik.setM0Brake(QIK_FULL_BRAKE);
    }
    if (currentstate==FAIL) return false;
    xQuad.write(0);
    
    SerialUSB.printf("At: %d,%d\n", xQuad.read(), yQuad.read());

    SerialUSB.println(F("Driving to X+"));
    Qik.setM0Speed(calspeed);
    while(!posLX() && !estopmonitor()) {delay(1);}
    Qik.setM0Brake(QIK_FULL_BRAKE);
    if (currentstate==FAIL) return false;
    maxpos.x=xQuad.read();
    SerialUSB.printf(F("Xmax %d\n"), maxpos.x);
    
    SerialUSB.printf("At: %d,%d\n", xQuad.read(), yQuad.read());
    
    //define soft limits
    negSoftLim.x = SOFT_LIMIT_COUNTS;
    negSoftLim.y = SOFT_LIMIT_COUNTS;
    posSoftLim.x = maxpos.x - SOFT_LIMIT_COUNTS;
    posSoftLim.y = maxpos.y - SOFT_LIMIT_COUNTS;
    
    //goto center
    SerialUSB.println(F("Driving to center"));
    pos.x=maxpos.x/2;
    if (!goto_pos(pos))
        return false;
    
    SerialUSB.printf("At: %d,%d\n", xQuad.read(), yQuad.read());
    delay(250);
    return !Qik.errorFlag;

}

void configure_paths() {
    if (!calibrated || pathsconfigured) return;
    vec2d_t loadpos = {LOAD_X_POSITION, LOAD_Y_POSITION};

    SerialUSB.println(F("\n+++++++++++++++++++Configuring Paths+++++++++++++++++++"));
    SerialUSB.printf(F("Free Ram: %d\n"),freeMemory());
    paths[0]=new PointPath("Load_P", loadpos);
    paths[0]->nextpath=1; paths[0]->autonextpath=false;
    
    paths[1]=new LemPath("Fib8P3", maxpos, 0, 0.5*COUNTS_PER_IN, 0.0,false);
    paths[2]=new LemPath("Fib8P2", maxpos, 1, 0.5*COUNTS_PER_IN, 0.0,false);
    paths[3]=new LemPath("Fib8P1", maxpos, 2, 0.5*COUNTS_PER_IN, 0.0,false);
    paths[1]->nextpath=2; paths[1]->autonextpath=true;
    paths[2]->nextpath=3; paths[2]->autonextpath=true;
    paths[3]->nextpath=1; paths[3]->autonextpath=false;
    
    //paths[4]=new LemPath("NewIFU", maxpos, 0, 3*COUNTS_PER_IN, 8.25*COUNTS_PER_IN); //3in half amp, 12 y skew
//    paths[4]=new LemPath("NewIFU", maxpos, 0, .5*COUNTS_PER_IN, .65*COUNTS_PER_IN);
    paths[4]=new LemPath("IFU8P3", maxpos, 0, 0.5*COUNTS_PER_IN, 0.0,true);
    paths[5]=new LemPath("IFU8P2", maxpos, 1, 0.5*COUNTS_PER_IN, 0.0,true);
    paths[6]=new LemPath("IFU8P1", maxpos, 2, 0.5*COUNTS_PER_IN, 0.0,true);
    paths[4]->nextpath=5; paths[4]->autonextpath=true;
    paths[5]->nextpath=6; paths[5]->autonextpath=true;
    paths[6]->nextpath=4; paths[6]->autonextpath=false;

    ((LemPath*)paths[1])->print();
    ((LemPath*)paths[2])->print();
    ((LemPath*)paths[3])->print();
    
    ((LemPath*)paths[4])->print();
    ((LemPath*)paths[5])->print();
    ((LemPath*)paths[6])->print();
    
    //Point path
    SerialUSB.printf(F("Name: %s   Start: %5g,%5g\n"), paths[0]->name, paths[0]->startpos.x, paths[0]->startpos.y);
    
    SerialUSB.printf(F("Configuring Paths Complete. Free Ram: %d\n\n"),freeMemory());
    
    pathsconfigured = true;
}


bool estopmonitor() {
    // handles estop and returns true if an estop
    if (Buttons.clickedsince(ESTOP)){
        enterFail(EHALT);
        return true;
    }
    return false;
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
    
    // this routine assumes we are going in from stopped
    
    SerialUSB.printf(F("At %d, %d. Commanded to %g, %g\n"),
                     xQuad.read(), yQuad.read(), pos.x,pos.y);
    
    //Don't move beyond limits
    if (posOutsideLimits(pos) && calibrated) {
        SerialUSB.println(F("Outside SW Limits, not going."));
        return false;
    }
    
    if (inPosition(pos)) {
        SerialUSB.printf(F("Already there, err (%f, %f)\n"), poserr.x, poserr.y);
        return true;
    }
    
    SerialUSB.println(F("Starting Move"));
    //While we arent in position loop

    resetposvel();
    updategoal(pos);
    pidXOut=pidYOut=0;
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
    while(!inPosition(pos) && !estopmonitor()) {
        updateposvel();
        if (xPID.WillCompute() || yPID.WillCompute()) {
            goalvel = vTowardGoal(goalpos, SLEW_SPEED);
            //SerialUSB.printf(F("PE:%5f, %5f  V:%f, %f  Qv:%f, %f"),
            //                 poserr.x, poserr.y, goalvel.x, goalvel.y, pidXOut, pidYOut);
            xPID.Compute();
            yPID.Compute();
            //SerialUSB.printf(F("  nQv:%f, %f\n"), pidXOut, pidYOut);
            setqikspeeds(pidXOut, pidYOut);
        }
        
        if ((negLX() && pidXOut<0) || (posLX() && pidXOut>0) ||
            (negLY() && pidYOut<0) || (posLY() && pidYOut>0)) {
            bounceOffLimits();
            enterFail(LIMITS);
            SerialUSB.println(F("Tried to drive into limits"));
            break;
        }
    }
    xPID.SetMode(MANUAL);
    yPID.SetMode(MANUAL);
    
    setqikbrakes(QIK_PAUSE_BRAKE, QIK_PAUSE_BRAKE);

    currentpos.x=xQuad.read();
    currentpos.y=yQuad.read();
    SerialUSB.printf(F("Commanded to %g, %g. Attained %d, %d. CurrPos: %g, %g\n"),
                     pos.x, pos.y, xQuad.read(), yQuad.read(), currentpos.x, currentpos.y);
    
    return inPosition();
}



void minspeedtest(unsigned int spd) {
    
    
    if (!calibrated || currentstate!=IDLE) {
        SerialUSB.println(F("Must be calibrated and idle"));
        return;
    }
    
    vec2d_t pos, stoppos, vel, decel, dpos={1,1}, maxvel={0,0};
    unsigned long tic, temp;
    
    lcddisplay.display(F("Running minspeed Test"));
    SerialUSB.println(F("Running minspeed Test"));
    
    //Go to start of test position
    if (!goto_pos(negSoftLim) || currentstate==FAIL) {
        SerialUSB.println(F("Uable to achieve start position"));
        return;
    }
    
    ehalt();
    delay(2000);
    
    //Stop moving pos
    stoppos.x=maxpos.x*.7;
    stoppos.y=maxpos.y*.7;

    
    
    //Wait till quarter travel, monitoring speed
    setqikspeeds(50,0);
    delay(100);
    setqikspeeds(abs(spd),0);
    tic=millis();
    while (xQuad.read()<stoppos.x && !Buttons.clickedsince(ESTOP)) {
        if (posLX() || posSLX()) {
            SerialUSB.println(F("Hit +X lim"));
            break;
        }
        updateposvel2();
        if (millis()-tic > 50) {
            SerialUSB.printf("V:%.2f,%.2f\n",currentvel2.x,currentvel2.y);
            tic=millis();
        }
    }
    
    if (posLX()) bounceOffLimits();

    setqikspeeds(0,50);
    delay(100);
    setqikspeeds(0,abs(spd));
    
    SerialUSB.println(F("Testing Y"));
    tic=millis();
    while (yQuad.read()<stoppos.y && !Buttons.clickedsince(ESTOP)) {
        if (posLY() || posSLY()) {
            SerialUSB.println(F("Hit +Y lim"));
            break;
        }
        updateposvel2();
        if (millis()-tic > 50) {
            SerialUSB.printf("V:%.2f,%.2f\n",currentvel2.x,currentvel2.y);
            tic=millis();
        }
    }
    
    if (posLY()) bounceOffLimits();

    ehalt();
    
}


void deceltest(char brake_amt) {

    if (!calibrated || currentstate!=IDLE) {
        SerialUSB.println(F("Must be calibrated and idle"));
        return;
    }
    
    vec2d_t pos, stoppos, vel, decel, dpos={1,1}, maxvel={0,0};
    unsigned long tic, temp;
    
    lcddisplay.display(F("Running Decel Test"));
    SerialUSB.println(F("Running Decel Test"));
    
    //Go to start of test position
    if (!goto_pos(negSoftLim) || currentstate==FAIL) {
        SerialUSB.println(F("Uable to achieve start position"));
        return;
    }
    
    ehalt();
    delay(2000);
    
    //Stop moving pos
    stoppos.x=maxpos.x*.7;
    stoppos.y=maxpos.y*.7;
    
    //Full Speed
    SerialUSB.println(F("x vx 20ms increments"));
    pos.x=xQuad.read();
    SerialUSB.printf(F("x=(%d, %.5f),\n"), xQuad.read(), 0);
    setqikspeeds(QIK_MAX_SPEED, 0);
    
    //Wait till quarter travel, monitoring speed
    while (pos.x<stoppos.x && !estopmonitor()) {
        if (posLX()) {
            SerialUSB.println(F("Hit +X lim in acceleration"));
            break;
        }
        delay(20);
        temp=xQuad.read();
        vel.x=(temp-pos.x)/20.0;
        SerialUSB.printf(F("(%d, %.5f),\n"), temp, vel.x);
        maxvel.x = max(vel.x, maxvel.x);
        pos.x=temp;
    }
    
    if (posLX()) {
        bounceOffLimits();
        SerialUSB.println(F("Hit X limit during test"));
        return;
    }
    
    SerialUSB.println(F("Decelerating"));
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
    decel.x=vel.x*1000/(micros()-tic);
    SerialUSB.printf(F("X Deceleration took %d us\n"), micros()-tic);
    
    if (posLX()) {
        SerialUSB.println(F("Hit X limit during test"));
        return;
    }
    
    SerialUSB.println(F("Testing Y"));
    
    //Full Speed
    SerialUSB.println(F("y vy 20ms increments"));
    SerialUSB.printf(F("y=[(%d, %.5f),\n"), yQuad.read(), 0);
    setqikspeeds(0, QIK_MAX_SPEED);
    
    //Wait till quarter travel, monitoring speed
    pos.y=yQuad.read();
    while (pos.y<stoppos.y && !estopmonitor()) {
        if (posLY()) {
            SerialUSB.println(F("Hit +Y lim in acceleration"));
            break;
        }
        delay(20);
        temp=yQuad.read();
        vel.y=(temp-pos.y)/20.0;
        SerialUSB.printf(F("(%d, %.5f\n"), temp, vel.y);
        maxvel.y = max(vel.y, maxvel.y);
        pos.y=temp;
    }
    
    if (posLY()) {
        bounceOffLimits();
        SerialUSB.println(F("Hit Y limit during test"));
        return;
    }
    
    SerialUSB.println(F("Decelerating"));
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
    decel.y=vel.y*1000/(micros()-tic);
    SerialUSB.printf(F("Y Deceleration took %d us\n"), micros()-tic);
    
    if (posLY()) {
        SerialUSB.println(F("Hit Y limit during test"));
        return;
    }
    
    SerialUSB.printf(F("Max speed: %6.3f, %6.3f cts/ms\n"), maxvel.x, maxvel.y);
    SerialUSB.printf(F("Decel at %3d brake: %g, %g cts/ms^2\n"),brake_amt, decel.x, decel.y);
}


bool inPosition_x() {
    poserr.x = xQuad.read()-goalpos.x;
    return abs(poserr.x) < POSITION_TOLERANCE;
}

bool inPosition_y() {
    poserr.y = yQuad.read()-goalpos.y;
    return abs(poserr.y) < POSITION_TOLERANCE;
}

inline bool inPosition() {
    return inPosition_x() && inPosition_y();
}

bool inPosition(vec2d_t pos) {
    poserr.x = xQuad.read()-pos.x;
    poserr.y = yQuad.read()-pos.y;
    return (abs(poserr.x) < POSITION_TOLERANCE && abs(poserr.y) < POSITION_TOLERANCE);
}

bool inPosition(vec2dint_t pos) {
    poserr.x = xQuad.read()-pos.x;
    poserr.y = yQuad.read()-pos.y;
    return (abs(poserr.x) < POSITION_TOLERANCE && abs(poserr.y) < POSITION_TOLERANCE);
}



vec2d_t vTowardGoalDebug(vec2d_t goal, double speed) {
    
    
    vec2d_t ret={0,0};
    
    double dx = goal.x-xQuad.read();
    double dy = goal.y-yQuad.read();
    double mag;
    
    if (abs(dx) <= POSITION_TOLERANCE)
        dx = 0.0;
    if (abs(dy) <= POSITION_TOLERANCE)
        dy = 0.0;
    
    SerialUSB.printf(F("g:%f,%f dp:%f,%f p:%d,%d\n"), goal.x,goal.y,dx, dy, xQuad.read(),yQuad.read());

    if ((dy==0) && (dx==0)) {
        0;
    }else if (dy==0) {
        ret.x = (dx > 0) ? speed: -speed;
    } else if (dx==0) {
        ret.y = (dy > 0) ? speed: -speed;
    } else {
        mag = sqrt(dy*dy + dx*dx);
        ret.x = dx*speed/mag;
        ret.y = dy*speed/mag;
    }
    
    SerialUSB.printf(F("s:%f  v2g:%f,%f\n"), speed, ret.x, ret.y);

    
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



void tunePID() {
    
    //at maxspeed we can cross the travel in ~{51314, 63996}/10.6667 ~ 4.8s and 6s
    // or about 480 ~ 600 PID loops. We will run at half speed so tune no more than 800 cycles
    // should take about 8 seconds
    
    unsigned long tic, lastTic;
    double outputX, outputY;
    char num;
    vec2d_t pos = {negSoftLim.x, negSoftLim.y};
    
    if (!calibrated || currentstate!=IDLE) {
        SerialUSB.println(F("Must be calibrated and idle"));
        return;
    }
    
    if (!goto_pos(pos)){
        SerialUSB.println(F("Could not attain autotuning start position"));
        return;
    }

    // Set the target value to tune to    // This must be called immediately before the tuning loop
    SerialUSB.println(F("#ms goalvx goalvy vx vy qvx qvy"));
    SerialUSB.println(F("dat=np.array(["));

    resetposvel();
    pidXOut=pidYOut=0;
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
    while(!posLX() && !estopmonitor()) {
        tic = millis();
        if (SerialUSB.available()){
            num = SerialUSB.read()-'0';
            goalvel.x=max(min(num,MAX_COUNTS_PER_MS_X),0);
            goalvel.y=max(min(num,MAX_COUNTS_PER_MS_Y),0);
        }
        if (tic-lastTic>PID_INTERVAL_MS) {
            updateposvel();
            xPID.Compute();
            yPID.Compute();
            setqikspeeds(pidXOut, pidYOut);
            SerialUSB.printf(F("(%d, %f, %f, %f, %f, %f, %f),\n"), tic,
                             goalvel.x, goalvel.y, currentvel.x, currentvel.y, pidXOut, pidYOut);

        }
        lastTic=tic;
        if (hardLimitsTripped()) {
            ehalt();
            bounceOffLimits();
            break;
        }
    }
    SerialUSB.println(F("]).T\n"));
    
    setqikspeeds(0, 0);
    
}


union {                // This Data structure lets
    byte asBytes[24];    // us take the byte array
    float asFloat[6];    // sent from processing and
}                      // easily convert it to a
pidtuneunion;                   // float array



//unsigned long serialTime; //this will help us know when to talk with processing

void PIDTuneLoopX() {
    //pid-related code
    
    unsigned long tic, lastTic=millis();
    while(SerialUSB.available()) SerialUSB.read();
    
    resetposvel();
    pidXOut=pidYOut=0;
    xPID.SetMode(AUTOMATIC);
    bool updated;
    while(!hardLimitsTripped() && !estopmonitor()) {
        tic = millis();
        updated=updateposvel();
        if (xPID.WillCompute()) {//})(tic-lastTic>PID_INTERVAL_MS) {
            xPID.Compute();
            //yPID.Compute();
            setqikspeeds(pidXOut, pidYOut);
            lastTic=tic;
        }
        if (updated) {
            PIDTuneSerialReceiveX();
            PIDTuneSerialSendX();
        }
    }

    
    ehalt();
    
}

void PIDTuneSerialReceiveX()
{
    
    // read the bytes sent from Processing
    int index=0;
    byte Auto_Man = -1;
    byte Direct_Reverse = -1;
    while(SerialUSB.available()&&index<26)
    {
        if(index==0) Auto_Man = SerialUSB.read();
        else if(index==1) Direct_Reverse = SerialUSB.read();
        else pidtuneunion.asBytes[index-2] = SerialUSB.read();
        index++;
    }
    
    // if the information we got was in the correct format,
    // read it into the system
    if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
    {
        goalvel.x=double(pidtuneunion.asFloat[0]);
        //Input=double(foo.asFloat[1]);       // * the user has the ability to send the
        //   value of "Input"  in most cases (as
        //   in this one) this is not needed.
        if(Auto_Man==0)                       // * only change the output if we are in
        {                                     //   manual mode.  otherwise we'll get an
            pidXOut=round(double(pidtuneunion.asFloat[2]));      //   output blip, then the controller will
        }                                     //   overwrite.
        
        double p, i, d;                       // * read in and set the controller tunings
        p = double(pidtuneunion.asFloat[3]);           //
        i = double(pidtuneunion.asFloat[4]);           //
        d = double(pidtuneunion.asFloat[5]);           //
        xPID.SetTunings(p, i, d);            //
        lcddisplay.display("PID: ");
        lcddisplay.serial->printf("%f %f %f",p,i,d);
        
        if(Auto_Man==0) xPID.SetMode(MANUAL);// * set the controller mode
        else xPID.SetMode(AUTOMATIC);             //
        
        //if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
        //else xPID.SetControllerDirection(REVERSE);          //
    }
    while(SerialUSB.available()) SerialUSB.read(); // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void PIDTuneSerialSendX()
{
    SerialUSB.print("PID ");
    SerialUSB.print(goalvel.x);
    SerialUSB.print(" ");
    SerialUSB.print(currentvel.x);
    SerialUSB.print(" ");
    SerialUSB.print(pidXOut);
    SerialUSB.print(" ");
    SerialUSB.print(xPID.GetKp());
    SerialUSB.print(" ");
    SerialUSB.print(xPID.GetKi());
    SerialUSB.print(" ");
    SerialUSB.print(xPID.GetKd());
    SerialUSB.print(" ");
    if(xPID.GetMode()==AUTOMATIC) SerialUSB.print("Automatic");
    else SerialUSB.print("Manual");
    SerialUSB.print(" ");
    if(xPID.GetDirection()==DIRECT) SerialUSB.println("Direct");
    else SerialUSB.println("Reverse");
}
