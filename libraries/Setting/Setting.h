#ifndef Setting_h
#define Setting_h

#include <Arduino.h>

class Setting
{
    public:
        Setting(String settingname, double initV, double minV, double maxV, int knobstepV);
    
        void update(int knobdelta);
        void setMin(double minV);
        void setMax(double maxV);
    
        String name;
        double value; //the Setting value
        double min;
        double max;
        int knobstep;
};

#endif
