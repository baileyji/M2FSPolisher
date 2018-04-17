#include <Arduino.h>

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

//#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559

#define LEM_SIZE  0.5/0.00024157
#define COUNTS_PER_IN 1/0.00024157
#define COUNTS_PER_MM 1/0.00613100
#define PUCK_SIZE 2.0/0.00024157

#ifndef PolishPath_h
#define PolishPath_h

class PolishPath {
    public:
        PolishPath();
        uint8_t percentComplete;
        uint8_t nextpath;
        char name[7];
        vec2d_t last;
        vec2d_t current;
        vec2d_t startpos;
        vec2d_t finalpos;
        unsigned int timeLeft;
        unsigned int pathLength;
        unsigned int finali;
    
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

    
};


class LemPath : public PolishPath {
    public:
        LemPath(char namestr[], vec2dint_t maxpos, char pass, double xskew, double halfamp); // in units of counts!
        virtual bool isMore();  //Is the current goal the final positon
        virtual vec2d_t computePathPos();
        double awavenum;  //angualr wave number 2pi/period
        double nlem;
        int dir;
        double lemsize;
        double xstretch;

};

class LinePath : public PolishPath {
    public:
        LinePath(char namestr[], vec2d_t start, vec2d_t end);
};

class PointPath : public LinePath {
    public:
        PointPath(char namestr[], vec2d_t start);
        virtual bool isMore();  //Is the current goal the final positon
};


#endif

