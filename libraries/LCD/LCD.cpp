#include "LCD.h"

LCD::LCD() : Serial1()
{
    0;
}

void LCD::init()
{
    begin(9600);
    turnDisplayOn();
    backlight(29);
    underlineCursorOn();
    clearScreen();
}

void LCD::clearScreen()
{
    //clears the screen, you will use this a lot!
    write(0xFE);
    write(0x01);
}

void LCD::selectLineOne()
{
    //puts the cursor at line 0 char 0.
    write(0xFE); //command flag
    write(128); //position
}

void LCD::selectLineTwo()
{
    //puts the cursor at line 0 char 0.
    write(0xFE); //command flag
    write(192); //position
}

void LCD::moveCursorRightOne()
{
    //moves the cursor right one space
    write(0xFE); //command flag
    write(20); // 0x14
}

void LCD::moveCursorLeftOne()
{
    //moves the cursor left one space
    write(0xFE); //command flag
    write(16); // 0x10
}

void LCD::scrollRight()
{
    //same as moveCursorRightOne
    write(0xFE); //command flag
    write(20); // 0x14
}

void LCD::scrollLeft()
{
    //same as moveCursorLeftOne
    write(0xFE); //command flag
    write(24); // 0x18
}

void LCD::turnDisplayOff()
{
    //this tunrs the display off, but leaves the backlight on.
    write(0xFE); //command flag
    write(8); // 0x08
}

void LCD::turnDisplayOn()
{
    //this turns the dispaly back ON
    write(0xFE); //command flag
    write(12); // 0x0C
}

void LCD::underlineCursorOn()
{
    //turns the underline cursor on
    write(0xFE); //command flag
    write(14); // 0x0E
}


void LCD::underlineCursorOff()
{
    //turns the underline cursor off
    write(0xFE); //command flag
    write(12); // 0x0C
}

void LCD::boxCursorOn()
{
    //this turns the box cursor on
    write(0xFE); //command flag
    write(13); // 0x0D
}

void LCD::boxCursorOff()
{
    //this turns the box cursor off
    write(0xFE); //command flag
    write(12); // 0x0C
}

void LCD::toggleSplash()
{
    //this toggles the spalsh screenif off send this to turn onif on send this to turn off
    write(0x7C); //command flag = 124 dec
    write(9); // 0x09
}

void LCD::backlight(unsigned int brightness)// 0 = OFF, 29 = Fully ON, everything inbetween = varied brightnbess
{
    brightness = (brightness>29) ? 29:brightness;
        
    //this function takes an int between 128-157 and turns the backlight on accordingly
    write(0x7C); //NOTE THE DIFFERENT COMMAND FLAG = 124 dec
    write(brightness+128); // any value between 128 and 157 or 0x80 and 0x9D
}


////-------------------------------------------------------------------------------------------
//void scrollingMarquee()
//{
//    //This function scroll text across the screen on both lines
//    clearScreen(); // it's always good to clear the screen before movonh onto a new print
//    for(int j = 0; j < 17; j++)
//    {
//        selectLineOne();
//        for(int i = 0; i < j;i++)
//            moveCursorRightOne();
//        print("SPARK");
//        selectLineTwo();
//        for(int i = 0; i < j;i++)
//            moveCursorRightOne();
//        print(" FUN");
//        delay(500); // you must have a delay, otherwise the screen will print and clear before you can see the text
//        clearScreen();
//    }
//}
////-------------------------------------------------------------------------------------------
//void counter()
//{
//    //this function prints a simple counter that counts to 10
//    clearScreen();
//    for(int i = 0; i <= 10; i++)
//    {
//        print("Counter = ");
//        print(i, DEC);
//        delay(500);
//        clearScreen();
//    }
//}
////-------------------------------------------------------------------------------------------
//void tempAndHumidity()
//{
//    //this function shows how you could read the data from a temerature and humidity
//    //sensro and then print that data to the SerLCD.
//
//    //these could be varaibles instead of static numbers
//    float tempF = 77.0;
//    float tempC = 25.0;
//    float humidity = 67.0;
//
//    clearScreen();
//    selectLineOne();
//    LCD.print(" Temp = ");
//    LCD.print((long)tempF, DEC);
//    LCD.print("F ");
//    LCD.print((long)tempC, DEC);
//    LCD.print("C");
//    selectLineTwo();
//    LCD.print(" Humidity = ");
//    LCD.print((long)humidity, DEC);
//    LCD.print("%");
//    delay(2500);
//}
////-------------------------------------------------------------------------------------------
//void backlight()
//{
//    //this function shows the different brightnesses to which the backlight can be set
//    clearScreen();
//    for(int i = 128; i < 158; i+=2)// 128-157 are the levels from off to full brightness
//    {
//        backlight(i);
//        delay(100);
//        LCD.print("Backlight = ");
//        LCD.print(i, DEC);
//        delay(500);
//        clearScreen();
//    }
//}
////-------------------------------------------------------------------------------------------
//void cursors()
//{
//    //this function shows the different cursors avaiable on the SerLCD
//    clearScreen();
//
//    boxCursorOn();
//    LCD.print("Box On");
//    delay(1500);
//    clearScreen();
//
//    boxCursorOff();
//    LCD.print("Box Off");
//    delay(1000);
//    clearScreen();
//
//    underlineCursorOn();
//    LCD.print("Underline On");
//    delay(1500);
//    clearScreen();
//
//    underlineCursorOff();
//    LCD.print("Underline Off");
//    delay(1000);
//    clearScreen();
//}

