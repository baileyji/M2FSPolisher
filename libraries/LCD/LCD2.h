#ifndef LCD2_h
#define LCD2_h
#include <Arduino.h>

#define DISPLAYLEN 16

class LCD2 {
    public:
        LCD2(Stream *);
    
        void init();
    
        void clearScreen();
        void clear();
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
        void backlight(char brightness);
    
        void saveSplash();
        void bootbaudreset();
        void setbaud9600();
    
        void write(char c);
    
        void display(char cstr[]);
        void print(char cstr[]);
        void print2(char cstr[]);
    
        #ifdef F // check to see if F() macro is available
        void display(const __FlashStringHelper *str);
        void print(const __FlashStringHelper *str);
        void print2(const __FlashStringHelper *str);
        #endif
    
        Stream* serial;
};

#endif /* LCD2_hpp */
