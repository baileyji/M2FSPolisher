#include <Arduino.h>
#include <PID_v1.h>
#include <DueTimer.h>
#include <Encoder.h>
#include <LCD2.h>
#include <PololuQik.h>
#include <Buttons.h>
#include <PolishPath.h>
#include <PString.h>
#include <SerialCommands.h>
#include <LEDEffect.h>

#define SERIAL_TX_BUFFER_SIZE 600

#ifndef vec2d_t_h
#define vec2d_t_h
typedef struct VECTOR2D_DOUBLE {
    double x;
    double y;
} vec2d_t;
#endif

/*
 DIO 0-53

 Serial: 0 (RX) and 1 (TX)      Qik
 Serial1: 19 (RX) and 18 (TX)   LCD
 Serial2: 17 (RX) and 16 (TX)
 Serial3: 15 (RX) and 14 (TX)  USB-SERIAL
 SerialUSB
 LED 13
 
 Avoid 0,1, 13 - 19
*/

/*
 SerialUSB commands
 qik get/set params
 
 decelleration test:
 calibrate, move to limit, move to max speed and .25 of travel then idle, report final pos, repeat with full break, 25% break, 75% break
 determine impulse response:
 instant max speed, log actual speeds at time steps, halt
 
 Tuning:
 use tuning code
 */



#define LED_PIN 13

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
#define MM_PER_COUNT 0.00613100
#define SPEED_TO_MMM 367.86  //convert cts/ms to mm/min

//These are maximum vector speeds
#define CALIBRATE_SPEED 2.0  //MarkI equiv. was about 10.09
#define SLEW_SPEED 8.0  //in units OF COUNTS/ms

#define SOFT_LIMIT_COUNTS 815 //about 5mm
#define POSITION_TOLERANCE 238 //238 is from MarkI ~1.5mm, 16.311 = 0.1mm

#define LOAD_Y_POSITION             15L*maxpos.y/16L

//old polisher the waypoints were about .14 mm apart

//max p error 654 um. avg 326 um min. detectable speed ~1.15/127
// so speeds below .1 cts/ms (~.61 mm/s) are not readable
#define PID_INTERVAL_MS 10

#define DEFAULT_PATH_ID 1

#define NUM_PATHS 1

#define PUCK_SIZE                    2.0/INCHES_PER_COUNT

#define MAX_MENU_LENGTH NUM_PATHS+1
#define CONTINUE_INDEX NUM_PATHS

#define DISPLAYLEN 16
#define SCREEN_REFRESH_MS 200

LEDEffect led(LED_PIN);

unsigned long lastScreenRefresh;

enum State {RUN, CFG, FAIL, IDLE, UNCAL};
enum FailReason {EHALT, SOFTLIM, LIMITS, CALFAIL, SWBUG};
enum Button {NXLIM, NYLIM, PXLIM, PYLIM, OK, ESTOP, LEFT, RIGHT};

PololuQik2s12v10 Qik = PololuQik2s12v10(QIK_RESET_PIN, QIK_ERROR_PIN);
LCD2 lcddisplay = LCD2(&Serial1);
Encoder xQuad = Encoder(X_QUADA_PIN, X_QUADB_PIN);
Encoder yQuad = Encoder(Y_QUADA_PIN, Y_QUADB_PIN);

byte pins[] = {NEG_X_LIM_PIN, NEG_Y_LIM_PIN, POS_X_LIM_PIN, POS_Y_LIM_PIN,
    OK_BUTTON_PIN, ESTOP_BUTTON_PIN, LEFT_BUTTON_PIN, RIGHT_BUTTON_PIN};

double speedSetting=0;

typedef struct VECTOR2D_INT32 {
    int32_t x;
    int32_t y;
} vec2dint_t;

vec2dint_t maxpos = {51314, 63996};     //From Mark 1
vec2dint_t negSoftLim = {SOFT_LIMIT_COUNTS, SOFT_LIMIT_COUNTS};     //About 5mm away based on Polisher Mark1
vec2dint_t posSoftLim = {51314-SOFT_LIMIT_COUNTS, 63996-SOFT_LIMIT_COUNTS};

vec2d_t computedpos;
vec2d_t currentpos;
vec2d_t goalpos;
vec2d_t followerror;

vec2d_t currentvel;
vec2d_t goalvel;

vec2dint_t qikcurrent={0,0};
vec2dint_t qikspeed={0,0};


double disttogoal;

//Path paths[NUM_PATHS];
//Path* currentpath;

double pidXOut, pidXSet=0, pidXIn=0;
double pidYOut, pidYSet=0, pidYIn=0;

double kDx=1, kIx=1, kDy=1, kIy=1;
double kPx=QIK_MAX_SPEED/MAX_COUNTS_PER_MS;
double kPy=QIK_MAX_SPEED/MAX_COUNTS_PER_MS;

//TODO sort out PID constants
PID xPID = PID(&currentvel.x, &pidXOut, &goalvel.x, kPx, kIx, kDx, DIRECT);
PID yPID = PID(&currentvel.y, &pidYOut, &goalvel.y, kPy, kIy, kDy, DIRECT);

State currentstate = UNCAL;

bool calibrated = false;
int menuindex = 0;
int menulen = MAX_MENU_LENGTH - 1;

//Debugging
unsigned long computetimes[]={0,0,0}, ncomputetimes[]={0,0,0}, maxtimes[]={0,0,0};
int freeram[]={0,0,0};
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

//Serial interface
char _cmdbuf[32];
SerialCommands cmdIO(&SerialUSB, _cmdbuf, sizeof(_cmdbuf), "\n", " ");
void cmd_unrecognized(SerialCommands* sender, const char* cmd) {
    sender->GetSerial()->print(F("Unrecognized command ["));
    sender->GetSerial()->print(cmd);
    sender->GetSerial()->println("]");
}
//-------------------------------
void report_cb(SerialCommands* sender) {
    report(sender->GetSerial());
}
SerialCommand cmd_report("S", report_cb);


//Paths
PolishPath* currentpath;
PolishPath* paths[2];
LemPath lem1=LemPath(goalpos,0);
LinePath line1=LinePath(goalpos, goalpos);



/*////////////////////////////////////////////////////////////////////////////////
Setup
*/////////////////////////////////////////////////////////////////////////////////
void setup() {
    
    //Setup Paths
    paths[0]=&lem1;
    paths[1]=&line1;
    lem1.nextpath=0;//&line1;
    
    //Configure IO
    Buttons.begin(pins, 8);

    //Configure LCD
    Serial1.begin(9600);
    lcddisplay.init();
    delay(500); //required by display
    lcddisplay.display("Booting...");
    
    //Connect to Qik
    Qik.init(115200);

    ///configure pid objects
    xPID.SetOutputLimits(QIK_MIN_SPEED, QIK_MAX_SPEED);
    yPID.SetOutputLimits(QIK_MIN_SPEED, QIK_MAX_SPEED);
    xPID.SetSampleTime(PID_INTERVAL_MS);
    yPID.SetSampleTime(PID_INTERVAL_MS);
    xPID.SetMode(MANUAL);
    yPID.SetMode(MANUAL);

    //Read inital speed setting
    speedSetting = readSpeed();
    
    //Setup SerialUSB
    SerialUSB.begin(115200);
    cmdIO.SetDefaultHandler(cmd_unrecognized);
    cmdIO.AddCommand(&cmd_report);
    

    //Enter uncalibrated state
    currentstate=UNCAL;
    led.fadeUp(2000);
    lcddisplay.display("Calibration     Required  Ready?");

}



/*////////////////////////////////////////////////////////////////////////////////
Main Loop
*/////////////////////////////////////////////////////////////////////////////////
void loop() {
    
    unsigned long tic, loopTic, housekeeepingTic=0, timeSincePosvel=millis();
    bool pidWillRun;
    
    loopTic = micros();
    
    //Monitor Estop
    if (Buttons.clickedsince(ESTOP))
        enterFail(EHALT);
    
    //Monitor Limits
    if (hardLimitsTripped()) {
        bounceOffLimits();
        enterFail(LIMITS);
    }
    if (softLimitsTripped())
        enterIdleSoftLimits();
    
    //LED
    led.update();
    
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
        //~347us
        qikcurrent.x=Qik.getM0CurrentMilliamps();
        qikcurrent.y=Qik.getM1CurrentMilliamps();
        housekeeepingTic=loopTic;
    }
    //Update Display
    displayScreen(false);
    
    //Handle Serial commands
    cmdIO.ReadSerial();
    
    //State machine
    switch(currentstate){
            
        //We're done until a reset
        case FAIL:
            delay(1);
            break;
            
        //We're uncalibrated
        case UNCAL:
            if (Buttons.clickedsince(OK)) {
                calibrated = calibrate();
                if (calibrated) enterIdleFromUncal();
                else enterFail(CALFAIL);
            }
            break;

        //We are idle
        case IDLE:
            
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
                    currentpath = paths[itemID2pathID(menuindex)];
                    currentpath->reset(currentpos);
                    computedpos=currentpos;
                    //if we are at start skip the starting point so we don't pause right away
                    if (inPosition(currentpath->current)<=POSITION_TOLERANCE)
                        currentpath->stepForward(speedSetting);
                }
                goalpos = currentpath->current;
                enterRun();
            }
            break;

        case RUN:
            if (!calibrated) {
                enterFail(SWBUG);
                break;
            }
            
            if (Buttons.clickedsince(OK)) {
                enterIdleFromRun();
                break;
            }

            tic = micros();
            if (pidWillRun) {
                
                if ((disttogoal < PID_INTERVAL_MS*speedSetting/2)||
                    (currentpath->goalIsBehind(currentpos)))
                {
                    if (currentpath->isPausePoint()) //both 0 and final are pause point
                        enterIdleFromRun();
                    if (currentpath->isMore()){
                        currentpath->stepForward(speedSetting);
                        goalpos = currentpath->current;
                    }
                    //both start and end are pause points
                    if (currentpath->isPausePoint())
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

inline unsigned int itemID2pathID(unsigned int itemID) {return itemID;}

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


void displayScreen(bool force) {
    
    long t = millis();
    
    if (softLimitsTripped()){
        /*
         ----------------
         Soft Limit    -X
         
         */
        lcddisplay.display("Soft Limit    ");
        if (negSLX()) lcddisplay.display("-X");
        else if (negSLY()) lcddisplay.display("-Y");
        else if (posSLX()) lcddisplay.display("+X");
        else if (posSLY()) lcddisplay.display("+Y");
        lcddisplay.display("                ");
        
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
    char buffer[33];
    char name[]="PathNa";
    PString out= PString(buffer, sizeof(buffer));
    
    unsigned int timeleft=currentpath->timeLeft;
    if (timeleft > 999) timeleft=999;
    out.format("Run  X%04d Y%04d%s %3d%% %3dm",
               round(currentpos.x*MM_PER_COUNT),
               round(currentpos.y*MM_PER_COUNT),
               name, currentpath->percentComplete, timeleft);
    lcddisplay.display(buffer);
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
        if (inPosition(paths[menuindex]->startpos)){
            lcddisplay.serial->printf(F("Ready.       Go?%8s    %3d%%"), namestr, 0);
        } else {
            lcddisplay.serial->printf(F("%8s        Slew to start?"), namestr);
        }
    } else {
        lcddisplay.serial->printf(F("Paused.  Resume?%8s    %3d%%"),
                                  namestr, currentpath->percentComplete);
        /*
         namestr[4] = '?';
        itoa(currentpath->percentComplete,parstr,10);
        
        lcddisplay.display("Paused.  Resume?");
        lcddisplay.print(namestr);
        for (int i=0; i<DISPLAYLEN-strlen(namestr)-strlen(parstr)-1; i++)
            lcddisplay.write(' ');
        lcddisplay.print(parstr);
        lcddisplay.write('%');
        */
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

void enterIdleFromRun() {
    currentstate=IDLE;
    led.breathe(3000);
    xPID.SetMode(MANUAL);
    yPID.SetMode(MANUAL);
    setqikbrakes(QIK_PAUSE_BRAKE, QIK_PAUSE_BRAKE);
    
    Buttons.clearChangeFlag(LEFT);
    Buttons.clearChangeFlag(RIGHT);
    Buttons.clearChangeFlag(OK);
    
    menulen = currentpath->isMore() ? MAX_MENU_LENGTH: MAX_MENU_LENGTH-1;
    menuindex = (currentpath->isMore()) ? CONTINUE_INDEX : currentpath->nextpath;
    
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
            lcddisplay.display("E-Stop Pressed");
            break;
        case SOFTLIM:
            lcddisplay.display("Soft Limits");
            break;
        case LIMITS:
            lcddisplay.display("Hard Limits");
            break;
        case CALFAIL:
            lcddisplay.display("Calibration Fail");
            break;
        case SWBUG:
            lcddisplay.display("Software Failure");
            break;
        default:
            lcddisplay.display("Unknown Failure");
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
    if(!Buttons.down(NYLIM)) {
        Qik.setM1Speed(-CALIBRATE_SPEED);
        while(!Buttons.down(NYLIM));
        Qik.setM1Brake(QIK_FULL_BRAKE);
    }
    yQuad.write(0);
    
    Qik.setM1Speed(CALIBRATE_SPEED);
    while(!Buttons.down(PYLIM));
    Qik.setM1Brake(QIK_FULL_BRAKE);
    
    maxpos.y=yQuad.read();
    pos.y=maxpos.y/2;
    pos.x=xQuad.read();
    if (!goto_pos(pos))
        return false;
    
    //Calibrate the x axis
    if(!Buttons.down(NXLIM)) {
        Qik.setM0Speed(-CALIBRATE_SPEED);
        while(!Buttons.down(NXLIM));
        Qik.setM0Brake(QIK_FULL_BRAKE);
    }
    xQuad.write(0);

    Qik.setM0Speed(CALIBRATE_SPEED);
    while(!Buttons.down(PXLIM));
    Qik.setM0Brake(QIK_FULL_BRAKE);
    
    maxpos.x=xQuad.read();
    pos.x=maxpos.x/2;
    if (!goto_pos(pos))
        return false;
    
    if (!Qik.errorFlag) {
        negSoftLim.x = SOFT_LIMIT_COUNTS;
        negSoftLim.y = SOFT_LIMIT_COUNTS;
        posSoftLim.x = maxpos.x - SOFT_LIMIT_COUNTS;
        posSoftLim.y = maxpos.y - SOFT_LIMIT_COUNTS;
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

    return inPosition();
}


void deceltest() {

    if (!calibrated) return;
    vec2d_t pos, stoppos;
    lcddisplay.display(F("Running Decel Test"));
    SerialUSB.println(F("Running Decel Test"));
    if (!goto_pos(negSoftLim) || currentstate==FAIL) {
        SerialUSB.println(F("Uable to achieve start position"));
        return;
    }
    
    pos.x=xQuad.read();
    pos.y=yQuad.read();
    stoppos.x=maxpos.x/4;
    stoppos.y=maxpos.y/4;
    
    //Full Speed
    setqikspeeds(QIK_MAX_SPEED, QIK_MAX_SPEED);
    
    //Wait till quarter travel
    while (pos.x<stoppos.x || pos.y<stoppos.y) {
        if (pos.x<stoppos.x) {
            pos.x=xQuad.read();
            if (pos.x>=stoppos.x) Qik.setM0Speed(0);
        }
        if (pos.y<stoppos.y) {
            pos.y=yQuad.read();
            if (pos.y>=stoppos.y) Qik.setM1Speed(0);
        }
    }
    //report final pos, repeat with full break, 25% break, 75% break
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

