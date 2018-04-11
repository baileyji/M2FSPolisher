#ifndef vec2d_t_h
#define vec2d_t_h
typedef struct VECTOR2D_DOUBLE {
    double x;
    double y;
} vec2d_t;
#endif

#define POSITION_TOLERANCE 238


#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559

#define NLEM 30  //approximate number of lemniscates in path
#define LEM_SIZE  0.5/0.00024157
#define LEM_PERIOD 44.0   //dist between points is ~ 5/3*LEM_SIZE/LEM_PERIOD
                          //Ideally  5/3*LEM_SIZE/LEM_PERIOD [cts] > speedSetting*10 [cts/update]
                          //max doable speed is 5/3*0.5/0.00024157/PERIOD*10/10.6667
                          // 32 points <= 101% MAXSPEED
                          // 44 points <=~75% MAXSPEED
                          // 76 points <= 43% MAXSPEED

#define COUNTS_PER_MM 1/0.00613100
#define PUCK_SIZE 2.0/0.00024157
#define TWO_PI_DIV_LEM_PERIOD TWO_PI/LEM_PERIOD



#ifndef PolishPath_h
#define PolishPath_h

#include <Arduino.h>


class PolishPath {
public:
    
//    PolishPath();
    
    uint8_t percentComplete;
    uint8_t nextpath;
    char name[7];
    vec2d_t last;
    vec2d_t current;
    vec2d_t startpos;
    vec2d_t finalpos;
    unsigned int timeLeft;
    unsigned int pathLength;

    //Return the velocity vector toward the currentGoal from the present position
    vec2d_t vTowardGoal(vec2d_t pos, double speed);

    bool isEnd();   //!isMore
    bool isStart();
    
    /*
     Does pos line beneath the axis perpendicular to the directed line from previousGoal to currentGoal that pases through currentGoal. Previous goal is taken to be the actual position at time of path start, in case of small numbers goalIsBehind will return True.
    */
    bool goalIsBehind(vec2d_t pos);
    
    void stepForward(double speed); // previousGoal-> currentGoal, currentGoal-> nextGoal
    void reset(vec2d_t pos);  //Reset the path to its start
    bool isPausePoint(); ///True if goal is the first point

    //Virtual
    virtual vec2d_t computePathPos();
    virtual bool isMore();  //Is the current goal the final positon

    
    
protected:
    unsigned int _i;
    unsigned int finali;
    
};


class LemPath : public PolishPath {
    public:
        LemPath(vec2d_t maxpos, unsigned int pass);
        virtual bool isMore();  //Is the current goal the final positon
        virtual vec2d_t computePathPos();

    protected:
        int dir;

};

class LinePath : public PolishPath {
    public:
        LinePath(vec2d_t start, vec2d_t end);
//        virtual bool isMore();  //Is the current goal the final positon
//        virtual vec2d_t computePathPos();
};



#endif

