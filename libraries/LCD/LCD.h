#ifndef LCD_h
#define LCD_h
#include <Arduino.h>

class LCD : public Serial1
{
public:
    LCD();
    
    void init();
    
    void clearScreen();
    void selectLineOne();
    void selectLineTwo();
    void moveCursorRightOne();
    void moveCursorLeftOne();
    void scrollRight();
    void scrollLeft();
    void turnDisplayOff();
    void turnDisplayOn();
    void underlineCursorOn();
    void underlineCursorOff();
    void boxCursorOn();
    void boxCursorOff();
    void toggleSplash();
    void backlight(unsigned int brightness);
    
    
};

#endif /* LCD_hpp */
