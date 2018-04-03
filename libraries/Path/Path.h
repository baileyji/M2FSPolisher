#ifndef Path_h
#define Path_h

#include <Arduino.h>
class Path {
    public:
        str name;
        vec2_t vTowardGoal(vec2_t pos, double speed); //return the velocity vector toward the currentGoal from the present position
        bool isMore();  //Is the current goal the final positon
        bool isEnd();   //!isMore
        bool goalIsBehind(vec2_t pos); // does pos line beneath the axis perpendicular to the directed line from previousGoal to currentGoal that pases through currentGoal. Previous goal is taken to be the actual position at time of path start, in case of small numbers goalIsBehind will return True.
        void stepForward(); // previousGoal-> currentGoal, currentGoal-> nextGoal
        void reset();  //Reset the path to its start
        bool isPausePoint(); ///True if goal is the first point
    
    protected:
        unsigned int _i;
        vec2_t last;
        vec2_t current;
};


#endif
