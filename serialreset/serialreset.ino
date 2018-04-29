#include <LCD2.h>





LCD2 lcddisplay = LCD2(&Serial3);
void setup() {
  // put your setup code here, to run once:
    Serial3.begin(9600);
    lcddisplay.init();
//    lcddisplay.toggleSplash();
//    lcddisplay.bootbaudreset();
    lcddisplay.init();
    lcddisplay.display("Hello World");
    delay(500);
}

bool setsplash = false;
void loop() {
  // put your main code here, to run repeatedly:

//    if (!setsplash) {
//      lcddisplay.display("M2FS Polisher v2");
//      lcddisplay.selectLineTwo();
//      lcddisplay.print("----LCD Boot----");
//      delay(100);
//      lcddisplay.saveSplash();    
//      delay(5000);
//      setsplash=true;
//    }

    lcddisplay.display("M2FS Polisher v2");
    lcddisplay.selectLineTwo();
    lcddisplay.print("!---LCD Boot---!");

  delay(2500);
    lcddisplay.display("1 2 3 4 5 6 7 8!");
    lcddisplay.selectLineTwo();
    lcddisplay.print("8 7 6 5 4 3 2 1!");

  delay(2500);

    lcddisplay.display(F("8 7 6 5 4 3 2 1!1 2 3 4 5 6 7 8!"));
  delay(2500);


}
