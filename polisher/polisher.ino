#include <Arduino.h>
//#include <PID_v1.h>
//#include <DueTimer.h>
//
//#include <LCD.h>
//#include <PololuQik.h>
//#include <Button.h>
//#include <Setting.h>
//#include <Path.h>

/*
 DIO 0-53

 Serial: 0 (RX) and 1 (TX)
 Serial 1: 19 (RX) and 18 (TX)
 Serial 2: 17 (RX) and 16 (TX)
 Serial 3: 15 (RX) and 14 (TX)  USB-SERIAL
 SerialUSB
 LED 13
 
 Avoid 0,1, 13 - 19
*/


//QIK Serial 0,1
#define QIK_ERROR_PIN 2
#define QIK_RESET_PIN 3
#define X_QUADA_PIN 4
#define X_QUADB_PIN 5
#define Y_QUADA_PIN 6
#define Y_QUADB_PIN 7
#define LEFT_BUTTON_PIN 8
#define LEFT_BUTTON_PIN 9
#define NEG_X_LIM_PIN 20
#define NEG_Y_LIM_PIN 21
#define POS_X_LIM_PIN 22
#define POS_Y_LIM_PIN 23
#define OK_BUTTON_PIN 24
#define ESTOP_BUTTON_PIN 25

#define QIK_BAUD_RATE 115200
#define FULL_BREAK 127
#define SWITCH_POLL_US 1000

#define DT_INTERVAL_MS 50  //Should be long relative to the math and has PID implications old used ~10 ms?

#define DEFAULT_PATH_ID 1

enum State {RUN, CFG, FAIL, IDLE, UNCAL};
enum FailReason {ESTOP, SOFTLIM, LIMITS, CALFAIL};

double pidXOut, pidXSet, pidXIn;
double pidYOut, pidYSet, pidYIn;

PID xPID = PID(&pidXIn, &pidXOut, &pidXSet, 2,5,1, DIRECT);  //TODO sort out constants
PID yPID = PID(&pidYIn, &pidYOut, &pidYSet, 2,5,1, DIRECT);

PololuQik2s12v10 Qik = PololuQik2s12v10(QIK_RESET_PIN, QIK_ERROR_PIN);

LCD lcddisplay = LCD();

Encoder xQuad = Encoder(X_QUADA_PIN, X_QUADB_PIN);
Encoder yQuad = Encoder(Y_QUADA_PIN, Y_QUADB_PIN);


Button negLimX = Button(NEG_X_LIM_PIN, HIGH, LOW);  //pin, pullup, active state (tripped()/pressed()/active() when)
Button negLimY = Button(NEG_Y_LIM_PIN, HIGH, LOW);
Button posLimX = Button(POS_X_LIM_PIN, HIGH, LOW);
Button posLimY = Button(POS_Y_LIM_PIN, HIGH, LOW);

Button okButton = Button(OK_BUTTON_PIN, ?, ?);
Button estopButton = Button(ESTOP_BUTTON_PIN, ?, ?);

Setting menuitem = Setting(0, NUM_MENU_ITEMS);  //Use a Setting as a menu selection tracker numpaths+numsettings integer vals
unsigned int speedSetting=0;

vec2d_t lowerLim = {10,10};  //TODO Syntax
vec2d_t upperLim = {51000,63000};

vec2d_t currentvel;
vec2d_t currentpos;

State currentstate = UNCAL;

int32_t max_y_pos, max_x_pos;
bool showcontinue = false;
bool calibrated = false;


void posvelupdater() {
    quad_t x=xQuad.read();
    quad_t y=yQuad.read();
    currentvel.x = (x-currentpos.x) / DT_INTERVAL;
    currentvel.y = (y-currentpos.y) / DT_INTERVAL;
    currentpos.x = x;
    currentpos.y = y;
}

void buttonupdater() {
    negLimX.update();
    posLimX.update();
    negLimY.update();
    posLimy.update();
    okButton.update();
    estopButton.update();
}


int setup(void) {
    
    //Configure IO
    negLimX.init();
    negLimY.init();
    posLimX.init();
    posLimY.init();
    okButton.init();
    estopButton.init();

    LCD.init();
    LCD.display('Booting...');
    
    //Connect to Qik
    Qik.init(115200);


    //Start monitoring buttons and limits
    Timer8.attachInterrupt(buttonupdater).start(SWITCH_POLL_US);
    
    //Start computing speed
    Timer7.attachInterrupt(posvelupdater).start(DT_INTERVAL_MS);

    ///configure pid objects
    pidXSet=0.0;
    pidXIn=0.0;
    pidYSet=0.0;
    pidYIn=0.0;
    xPID.SetOutputLimits(-127, 127);
    yPID.SetOutputLimits(-127, 127);
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
    
    //Enter uncalibrated state
    enterUncal();
}


int main(void) {
    
    
    if (estopButton.pressedSinceLastCall()) enterFail(ESTOP);
    failIfLimits();
    
    switch(currentstate){
        case FAIL:
            while(True) {delay(100);};  //We're done until a reset
            
        case UNCAL:
            if (okButton.pressedSinceLastCall()) {
                calbrated = calibrate();
                if (calbrated)
                    enterIdleFromUncal();
                else
                    enterFail(CALFAIL);
            }
            break;
            
        case CFG:
            setting.update(knobQuad.read());
            displayCfgScreen();
            if (userbutton.hasbeenpressed())
                enterIdleFromCfg(pathpaused);
            break;
            
        case IDLE:
            // menuitem.max is NUM_SETTINGS+NUM_PATHS or that +1 for showing continue?
            int knobv=knobQuad.read();
            knobQuad.write(knobv % menuitem.knobstep);
            menuitem.update(knobv);
            unsigned int selectionID = menuitem.value;
            
            displayIdleScreen();
            
            if (okButton.pressedSinceLastCall()){
                if (selectionIsSetting(selectionID))
                    enterConfig(itemID2settingID(selectionID));
                else if (selectionIsPath(selectionID)){
                    &currentpath = getPath(itemID2pathID(selectionID));
                    currentpath.reset();
                    enterRun();
                } else
                    enterRun();
            }
            break;

        case RUN:

            speedSetting.update(knobPos);
            displayRunScreen();
            
            if (goalExpectedBeforeNextLoop() & currentpath.isMore())
                if (currentpath.isPausePoint())
                    enterIdleFromRun(true);
                else
                    currentpath.stepForward();
            else if (currentpath.goalIsBehind(curentposition)) {  //Will never return true right after a reset()
                if (currentpath.isEnd()) {
                    enterIdleFromRun(false);
                    //Should show continue? or next path or something as a convenience if currentpath->nextpath implemented
                    break;
                } else if (currentpath.isPausePoint())
                    enterIdleFromRun(true);
                else
                    currentpath.stepForward();
            }
            
            if (userbutton.hasbeenpressed()) {
                enterIdleFromRun(true);
                break;
            }
            
            vsetpoint = currentpath.vTowardGoal(speedSetting.value);  //Get velocity vector toward path destination
            
            xSetpoint = vsetpoint.x;
            ySetpoint = vsetpoint.y;
            
            xPID.compute();
            yPID.compute();
            Qik.setM0Speed(xPIDSpeed);
            Qik.setM0Speed(yPIDSpeed);
            
            break;
    }
}


inline void selectionIsSetting(int selection) {
    return (selection > 0)
}

str itemID2name(itemID){
    if (selectionIsSetting(itemID))
        return settings[itemID2setting(itemID)].name;
    if (selectionIsPath(itemID))
        return paths[itemID2path(itemID)].name;
    return 'Continue';
}

inline unsigned int itemID2settingID(unsigned int itemID) {return itemID;}
inline unsigned int itemID2pathID(unsigned int itemID) {return itemID-NUM_SETTINGS;}
inline bool selectionIsSetting(itemID) {return itemID < NUM_SETTINGS;}
inline bool selectionIsPath(itemID) {return (itemID >= NUM_SETTINGS) && (itemID < (NUM_SETTINGS+NUM_PATHS));}


void enterRun(void) {
    currentstate=RUN;
    knobQuad.write(0);
    displayRunScreen();
    if (!calibrated || softLimitsTripped() || hardLimitsTripped()) {
        enterFail('Failed to go');
        return;
    }
}

bool softLimitsTripped() {
    return ((xQuad.read()<lowerLim.x | xQuad.read()<lowerLim.y)|
            (yQuad.read()>upperLim.x | yQuad.read()>upperLim.y));
}
    
bool hardLimitsTripped() {
    return (negLimX.tripped() | posLimX.tripped() | posLimY.tripped() | negLimY.tripped());
}
    
display(message){
    //write message to display
}


Path* getPath(id) {
    if (id<0) id=0;
    if (id>=NUM_PATHS) id = NUM_PATHS-1;
    return *paths[id]  //TODO elsewhere/somewhere the path needs to have current primed
}

void failIfLimits(){
    if (hardLimitsTripped()) enterFail('Hard Limit Fault');
    if (softLimitsTripped()) enterFail('Soft Limit Fault');
}
    
void enterUncal(){
    currentstate=UNCAL;
    display("Must Calibrate. Go?");
}

void enterIdleFromUncal(){
    currentstate=IDLE;
    knobQuad.write(0);
    menuitem.max = NUM_SETTINGS+NUM_PATHS;
    displayIdleScreen();
}

void enterConfig(unsigned int settingID){
    currentstate=CFG;
    knobQuad.write(0);
    if (settingID>=NUM_SETTINGS) settingID=NUM_SETTINGS-1;
    &setting = *settings[settingID]; //TODO is that the right pointer approach
    displayCfgScreen();
}

void enterIdleFromCfg(bool showcontinue){
    currentstate=IDLE;
    knobQuad.write(0);
    if (showcontinue) {
        menuitem.max = NUM_SETTINGS+NUM_PATHS+1;
        menuitem.value = menuitem.max;
    } else {
        menuitem.max = NUM_SETTINGS+NUM_PATHS;
        menuitem.value = NUM_SETTINGS;
    }
    displayIdleScreen();
}

void enterIdleFromRun(bool showcontinue){
    Qik.setM0Brake(FULL_BREAK);
    
    currentstate=IDLE;
    knobQuad.write(0);
    
    
    if (currentpath.isEnd())()  // pathPoint==END??
        currentpath=getPath(currentpath.nextPathID)
    else if (goalispathstart()) //TODO
        currentpath.point=0;//???
    //else
    //  do nothing
    
    if (showcontinue) {
        menuitem.max = NUM_SETTINGS+NUM_PATHS+1;
        menuitem.value = menuitem.max;
    } else {
        menuitem.max = NUM_SETTINGS+NUM_PATHS;
        menuitem.value = currentpath.nextPathID;
    }
    displayIdleScreen();
}

void displayIdleScreen() {
    itemID2name(menuitem.value);
    // Selected Path, speed, Go?/Slew?/Continue?
    if currentpath.pathPoint==UNSTARTED:
        //slew
    if currentpath.pathPoint==START:
        //go
    else:
        //continue
}


void enterFail(FailReason reason){
    switch (reason) {
        case ESTOP:
            LCD.display(F("E-Stop Pressed"));
            break;
        case SOFTLIM:
            LCD.display(F("Soft Limits"));
            break;
        case LIMITS:
            LCD.display(F("Hard Limits"));
            break;
        case CALFAIL:
            LCD.display(F("Calibration Failed"));
            break;
        default:
            LCD.display(F("Unspecified Failure"));
            break;
    }
    currentstate=FAIL;
    ehalt();
    display(reasonmsg);
}

void ehalt() {
    if (Qik.errorFlag)
        Qik.reset();
    Qik.setM0Brake(FULL_BREAK);
    Qik.setM1Brake(FULL_BREAK);
}

    
void calibrate() {
    
    //Calibrate the y-axis
    if(!yNegLim.tripped()) {
        Qik.setM1Speed(-CALIBRATE_SPEED);
        while(!yNegLim.tripped());
        Qik.setM1Brake(FULL_BREAK);
    }
    yPos.write(0);
    
    Qik.setM1Speed(CALIBRATE_SPEED);
    while(!yPosLim.tripped());
    Qik.setM1Brake(FULL_BREAK);
    
    max_y_pos=yPos.read();
    goto_y_pos(max_y_pos/2);
    Qik.setM1Brake(FULL_BREAK);
    
    //Calibrate the x axis
    if(!xNegLim.tripped()) {
        Qik.setM0Speed(-CALIBRATE_SPEED);
        while(!xNegLim.tripped());
        Qik.setM0Brake(FULL_BREAK);
    }
    xPos.write(0);

    Qik.setM0Speed(CALIBRATE_SPEED);
    while(!xPosLim.tripped());
    Qik.setM0Brake(FULL_BREAK);
    
    stage.max_x_pos=xPos.read();
    
    goto_x_pos(stage.max_x_pos/2);
    Qik.setM0Brake(FULL_BREAK);
    
    return !Qik.errorFlag
    
    
    //max_x: ~51314
    //max_y: ~63996
    
    /*
    polisher_x_offset[0]=stage.max_x_pos/20L;    //~2105void calibrate() {
    polisher_x_offset[1]=(long)(6.7*((float)stage.max_x_pos)/11.0);//16L*stage.max_x_pos/20L;//19L*stage.max_x_pos/20L; //~49337
    for (int i=0;i<NUM_PASSES;i++) {
        int32_t temp=lround(2.0*(float)stage.max_y_pos/3.0 + 5.0*COUNTS_PER_MM - //(5*COUNTS_PER_MM = fine tuning)
                            (float)i * (2.0*LEM_SIZE+COUNTS_PER_MM+PUCK_SIZE)-2.0*COUNTS_PER_MM);
        polisher_y_offset[i]=(temp < stage.max_y_pos) ? temp:stage.max_y_pos/2L;
    }
    */
}





