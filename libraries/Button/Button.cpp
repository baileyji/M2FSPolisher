#include "Button.h"

Button::Button(unsigned char pin, uint8_t enablepullup, uint8_t activestate)
{
   _pin = resetPin;
   _errorPin = errorPin;
   errorFlag = false;
   attachInterrupt(digitalPinToInterrupt(pin), &ButtonsClass::button_ISR, CHANGE);
}

bool Button::pressedSinceLastCall()
{
    bool pressedSinceLastCall() {
        if (pressingEdgeDetected) {
            pressingEdgeDetected=false;
            return true;
        } else
            return false;
}

