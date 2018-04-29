#include "PolishPath.h"

PolishPath::PolishPath() {
    _i=0;
    percentComplete=0;
    timeLeft=0;
    last=(vec2d_t){0,0};
    autonextpath=false;
    skipstartpause=false;
}

bool PolishPath::isStart() {
    return _i==0;
}

bool PolishPath::isEnd() {
    return !isMore();
}

bool PolishPath::isPausePoint(){
    ///True if goal is the first point
    return (isStart() && !skipstartpause) || isEnd();
}

bool PolishPath::goalIsBehind(vec2d_t pos) {
    // does pos line beneath the axis perpendicular to the directed line from previousGoal to currentGoal that pases through currentGoal. Previous goal is taken to be the actual position at time of path start, in case of small numbers goalIsBehind will return True.
    
    //(y2-y1)(y-y2)-(x1-x2)(x-x2)
    return (current.y-last.y)*(pos.y-current.y) - (last.x-current.x)*(pos.x-current.x) >= 0;
}

void PolishPath::stepForward(double speed) {
    if (!isMore()) return;
    double frac;
    _i++;
    last=current;
    current=computePathPos();
    frac =  (_i>1) ? (_i-1.0)/finali : 0;
    percentComplete = round(frac*100.0);
    timeLeft = (1-frac)*pathLength/(speed*60000);
}

void PolishPath::reset(vec2d_t pos) {
    //Reset the path to its start
    _i=0;
    last=pos;
    skipstartpause=false;
    current=computePathPos();
}

vec2d_t PolishPath::vTowardGoal(vec2d_t pos, double speed){
    //return the velocity vector toward the currentGoal from the present position
    
    vec2d_t ret={0,0};
    
    double dx = current.x-pos.x;
    double dy = current.y-pos.y;
    double mag;
    
//    if (abs(dx) <= POSITION_TOLERANCE)
//        dx = 0.0;
//    if (abs(dy) <= POSITION_TOLERANCE)
//        dy = 0.0;
    
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


vec2d_t PolishPath::computePathPos() {
    return finalpos;
}

bool PolishPath::isMore() {
    return _i==0;
}

#define PASS_OFFSET (2*halfamp+PUCK_SIZE)
#define IFU_PASS_OFFSET (2*halfamp+20*COUNTS_PER_MM)

LemPath::LemPath(char namestr[], vec2dint_t maxpos, char pass,
                 double halfamp, double xskew, bool ifu): PolishPath() {
    //The y extrema is PUCK_SIZE/2+halfamp so start pos should be based on that
    double offset = ifu ? IFU_PASS_OFFSET:PASS_OFFSET;
    pathpass=pass;
    if (pass==0) {
        startpos.x=maxpos.x/20.0 + 2.25*COUNTS_PER_IN;    //~2105
        startpos.y=0.6666667*maxpos.y;
        finalpos.x=maxpos.x*.94;  //NB we only end based on X position
        dir=1;
    } else if (pass==1) {
        startpos.x=maxpos.x*.94;  //~49337
        startpos.y=0.6666667*maxpos.y - offset;
        finalpos.x=maxpos.x/20.0 + 2.25*COUNTS_PER_IN;  //NB we only end based on X position
        dir=-1;
    } else {
        startpos.x=maxpos.x/20.0 + 2.25*COUNTS_PER_IN;
        startpos.y=0.6666667*maxpos.y - 2*offset;
        finalpos.x=maxpos.x*.94;
        dir=1;
    }
    
    
    if (startpos.y>maxpos.y) startpos.y=maxpos.y/2L;
    if (startpos.y<0) startpos.y=maxpos.y/2L;
    finalpos.y=0;
    
    lemsize=halfamp;//LEM_SIZE;  //in counts
    xstretch=xskew; //0; //in counts
    
    //68-150 over the range of possible sizes allwos even 1.6 PID updates at 75% max speed and just
    // over 1 at full
    //(150-68)/6.03*(lemsize-.5inches)+68
    unsigned int period=round(13.59867*(lemsize-2069.79))+68;
    if ((period%2)==1) period--; //even periods only
    period = max(period,44);
    awavenum = TWO_PI/period;
    
    //each loop is about loopwid wide the path is finalpos.x-startpos.x long
    // there are period steps in each so final i is about
    // period*(finalpos.x-startpos.x)/loopwid
    double loopwid = (lemsize + xstretch)*0.439696;
    nlem = abs(finalpos.x-startpos.x)/loopwid;

    //From Mathmatica, this is about correct for parameter combinations that make sense
    //tight to stretched (1.69249 to 1.69093) * pi ~ 5.3146637
    pathLength=5.3146637*lemsize*nlem;
    finali=round(period*nlem);
    
    strcpy(name, namestr); //6 char
    name[6]='\0';
    
    current=startpos;
    xstretch = (xstretch+lemsize)*dir/14.2899; //go ahead and precompute

}

vec2d_t LemPath::computePathPos() {
    vec2d_t next=startpos;
    //A=LemLongAxisHalfAmplitude
    //B=4sqrt(2)ArcCos(-sqrt(2/3)) ~ 14.2899
    //A Cos(t)/(Sin2(t)+1) , A(Cos(t)Sin(t)/(Sin2(t)+1) + t/B)
    // Path length over 2pi is ~= 5/3 a
    double t=_i * awavenum;
    double st=sin(t);
    double ct=lemsize*cos(t)/(st*st+1.0);
    next.x+=st*ct + t*xstretch;
    next.y+=ct;
    return next;
}

void LemPath::print() {
    SerialUSB.printf(F("%s: %d/%d  More/PP: %d/%d  Start/End: %d/%d\n"),
                     name, _i, finali, isMore(), isPausePoint(),
                     isStart(), isEnd());
    SerialUSB.printf(F("Next: %d   NLem: %g   Wave#: %g\n"),
                     nextpath, nlem, awavenum);
    SerialUSB.printf(F("Size: %g  XStr: %g  Points: %d  Length: %d  AutoC:%d\n"),
                     lemsize, xstretch, finali, pathLength, autonextpath);
    SerialUSB.printf(F("Start: %5g,%5g  End: %5g,%5g\n"),
                     startpos.x, startpos.y, finalpos.x, finalpos.y);
}

void PolishPath::print() {
    SerialUSB.printf(F("%s: %d/%d  More/PP: %d/%d  Start/End: %d/%d  AutoC: %d\n"),
                     name, _i, finali, isMore(), isPausePoint(),
                     isStart(), isEnd(), autonextpath);
}

bool LemPath::isMore() {
    //Is the current goal is the final positon
    if (_i==finali)
        return false;
    if ((current.x>finalpos.x && dir>0) || (current.x<finalpos.x && dir<0)) {
        SerialUSB.println(F("Reached end of Lem path"));
        _i=finali;
        return false;
    }
    return true;
}

LinePath::LinePath(char namestr[], vec2d_t start, vec2d_t end) : PolishPath() {

    current = startpos = start;
    finalpos = end;
    strcpy(name, namestr); //6 char
    name[7]='\0';

    finali=1;

    double dx=(end.x-start.x);
    double dy=(end.y-start.y);
    pathLength=sqrt(dx*dx+dy*dy);
}

PointPath::PointPath(char namestr[], vec2d_t point) : LinePath(namestr, point, point) {
    finali=0;
    pathLength=0;
}

bool PointPath::isMore() {
    return false;
}
