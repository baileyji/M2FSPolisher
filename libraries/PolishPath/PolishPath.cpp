#include "PolishPath.h"

//PolishPath::PolishPath() {
//    percentComplete=0;
//}

bool PolishPath::isStart() {
    return _i==0;
}

bool PolishPath::isEnd() {
    return !isMore();
}

bool PolishPath::isPausePoint(){
    ///True if goal is the first point
    return _i==0 || isEnd();
}

bool PolishPath::goalIsBehind(vec2d_t pos) {
    // does pos line beneath the axis perpendicular to the directed line from previousGoal to currentGoal that pases through currentGoal. Previous goal is taken to be the actual position at time of path start, in case of small numbers goalIsBehind will return True.
    return false;
}

void PolishPath::stepForward(double speed) {
    if (!isMore())  return;
    double frac;
    _i++;
    last=current;
    current=computePathPos();
    frac =  (_i>1) ? (_i-1.0)/finali : 0;
    percentComplete = round(frac*100.0);
    timeLeft = round((1-frac)*pathLength/(speed*60000));
}

void PolishPath::reset(vec2d_t pos) {
    //Reset the path to its start
    _i=0;
    last=pos;
    current=computePathPos();
}

vec2d_t PolishPath::vTowardGoal(vec2d_t pos, double speed){
    //return the velocity vector toward the currentGoal from the present position
    
    vec2d_t ret={0,0};
    
    double dx = current.x-pos.x;
    double dy = current.y-pos.y;
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


vec2d_t PolishPath::computePathPos() {
    return finalpos;
}

bool PolishPath::isMore() {
    return _i==0;
}


LemPath::LemPath(vec2d_t maxpos, unsigned int pass): PolishPath()
{
    _i=0;
    if (pass==1) {
        startpos.x=maxpos.x/20.0;    //~2105
        startpos.y=0.6666667*maxpos.y + 3.0*COUNTS_PER_MM;
        finalpos.x=maxpos.x*0.6091;  //NB we only end based on X position
        finalpos.y=0;
        dir=1;
        strcpy(name, "Lem+  "); //6 char
    } else {
        startpos.x=maxpos.x*0.6091;  //~49337
        startpos.y=0.6666667*maxpos.y + 3.0*COUNTS_PER_MM - (2*LEM_SIZE+COUNTS_PER_MM+PUCK_SIZE);

        finalpos.x=maxpos.x/20.0;  //NB we only end based on X position
        finalpos.y=0;
        dir=-1;
        strcpy(name, "Lem-  "); //6 char
    }
    if (startpos.y>maxpos.y) startpos.y=maxpos.y/2L;
    memset(name, '\0', sizeof(name));
    last.x=0;
    last.y=0;   //last goal
    current=startpos;
    percentComplete=0;
    finali=LEM_PERIOD*NLEM;
    pathLength=1.66667*LEM_SIZE*NLEM;
    timeLeft=0;
}

vec2d_t LemPath::computePathPos() {
    vec2d_t next=startpos;
    //A=LemLongAxisHalfAmplitude
    //B=4sqrt(2)ArcCos(-sqrt(2/3)) ~ 14.2899
    //A Cos(t)/(Sin2(t)+1) , A(Cos(t)Sin(t)/(Sin2(t)+1) + t/B)
    // Path length over 2pi is ~= 5/3 a
    double st=sin(_i * TWO_PI_DIV_LEM_PERIOD);
    double ct=cos(_i * TWO_PI_DIV_LEM_PERIOD)/(st*st+1.0);
    next.x+=LEM_SIZE*(st*ct + dir*_i/14.2899);
    next.y+=LEM_SIZE*ct;
    return next;
}

bool LemPath::isMore() {
    //Is the current goal is the final positon
    if (_i==finali)
        return false;
    if (abs(current.x-finalpos.x)<POSITION_TOLERANCE) {
        _i=finali;
        return false;
    }
    return true;
}

LinePath::LinePath(vec2d_t start, vec2d_t end) : PolishPath()
{
    _i=0;
    startpos=start;
    finalpos=end;
    strcpy(name, "A Line"); //6 char
    memset(name, '\0', sizeof(name));
    last.x=0;
    last.y=0;   //last goal
    current=startpos;
    percentComplete=0;
    finali=1;
    timeLeft=0;
    double dx=(end.x-start.x);
    double dy=(end.y-start.y);
    pathLength=sqrt(dx*dx+dy*dy);
}

//vec2d_t LinePath::computePathPos() {
//    return finalpos;
//}
//
//bool LinePath::isMore() {
//    return false;
//}

