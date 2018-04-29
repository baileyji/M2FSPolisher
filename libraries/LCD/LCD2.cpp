#include "LCD2.h"


LCD2::LCD2(Stream* serialptr)
{
    serial=serialptr;
}

void LCD2::init()
{
    
    delay(550);  //500 required but not enough
    turnDisplayOn();
    backlight(15);
    underlineCursorOff();
    boxCursorOff();
    clearScreen();
}

void LCD2::clearScreen()
{
    //clears the screen, you will use this a lot!
    serial->write(0xFE);
    serial->write(0x01);
}

void LCD2::selectLineOne()
{
    //puts the cursor at line 0 char 0.
    serial->write(0xFE); //command flag
    serial->write(128); //position
}

void LCD2::selectLineTwo()
{
    //puts the cursor at line 0 char 0.
    serial->write(0xFE); //command flag
    serial->write(192); //position
}

void LCD2::moveCursorRightOne()
{
    //moves the cursor right one space
    serial->write(0xFE); //command flag
    serial->write(20); // 0x14
}

void LCD2::moveCursorLeftOne()
{
    //moves the cursor left one space
    serial->write(0xFE); //command flag
    serial->write(16); // 0x10
}

void LCD2::scrollRight()
{
    //same as moveCursorRightOne
    serial->write(0xFE); //command flag
    serial->write(20); // 0x14
}

void LCD2::scrollLeft()
{
    //same as moveCursorLeftOne
    serial->write(0xFE); //command flag
    serial->write(24); // 0x18
}

void LCD2::turnDisplayOff()
{
    //this tunrs the display off, but leaves the backlight on.
    serial->write(0xFE); //command flag
    serial->write(8); // 0x08
}

void LCD2::turnDisplayOn()
{
    //this turns the dispaly back ON
    serial->write(0xFE); //command flag
    serial->write(12); // 0x0C
    delay(500); // behavior seems odd without
}

void LCD2::underlineCursorOn()
{
    //turns the underline cursor on
    serial->write(0xFE); //command flag
    serial->write(14); // 0x0E
}


void LCD2::underlineCursorOff()
{
    //turns the underline cursor off
    serial->write(0xFE); //command flag
    serial->write(12); // 0x0C
}

void LCD2::boxCursorOn()
{
    //this turns the box cursor on
    serial->write(0xFE); //command flag
    serial->write(13); // 0x0D
}

void LCD2::boxCursorOff()
{
    //this turns the box cursor off
    serial->write(0xFE); //command flag
    serial->write(12); // 0x0C
}

void LCD2::toggleSplash()
{
    //this toggles the spalsh screenif off send this to turn onif on send this to turn off
    serial->write(0x7C); //command flag = 124 dec
    serial->write(9); // 0x09
}

void LCD2::saveSplash()
{
    serial->write(0x7C);
    serial->write(0x0A);
}

void LCD2::bootbaudreset() {
    serial->write(0x12);
}

void LCD2::setbaud9600() {
    serial->write(124);
    serial->write(0x0D);
}

void LCD2::backlight(char brightness)// 0 = OFF, 29 = Fully ON, everything inbetween = varied brightnbess
{
    char bri;
    bri = (brightness>29) ? 157 : brightness+128;
        
    //this function takes an int between 128-157 and turns the backlight on accordingly
    serial->write(0x7C); //NOTE THE DIFFERENT COMMAND FLAG = 124 dec
    serial->write(bri); // any value between 128 and 157 or 0x80 and 0x9D
    delay(50);  //Odd behavior with future calls w/o a delay here
}


void LCD2::write(char c)
{
    serial->write(c);
}

void LCD2::display(char cstr[])
{
    clearScreen();
    selectLineOne();
    serial->print(cstr);
}

void LCD2::print(char cstr[])
{
    serial->print(cstr);
}

void LCD2::print2(char cstr[])
{
    selectLineTwo();
    serial->print(cstr);
}

void LCD2::print(const __FlashStringHelper *str)
{
    serial->print(str);
}

void LCD2::print2(const __FlashStringHelper *str)
{
    selectLineTwo();
    serial->print(str);
}

void LCD2::display(const __FlashStringHelper *str)
{
    clearScreen();
    selectLineOne();
    serial->print(str);
}


void LCD2::clear()
{
    clearScreen();
    selectLineOne();
}


