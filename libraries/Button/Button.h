#ifndef Button_h
#define Button_h

#include <Arduino.h>

class Button
{
  public:
    Button(unsigned char pin, highlow_t enablepullup, highlow_t activestate);
  private:
    _activehigh = true;
    _pressingEdgeDetected = false;
    
  public:
    void update(); //call always
    bool active();
    bool pressedSinceLastCall();
};

#endif
