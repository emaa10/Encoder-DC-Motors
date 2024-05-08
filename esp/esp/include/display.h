#include "TM1637.h"

int8_t displayData[] = {0x00,0x00,0x00,0x00};

#define CLK 23//pins definitions for TM1637 and can be changed to other ports
#define DIO 22
TM1637 tm1637(CLK,DIO);

void initialiseDisplay(){
    tm1637.set();
    tm1637.init();
    tm1637.point(POINT_OFF);

    displayData[0] = 0;
    displayData[1] = 0;
    displayData[2] = 0;
    displayData[3] = 0;

    tm1637.display(displayData);
}

void displayInteger(int number){
    displayData[1] = (number / 100) % 10;
    displayData[2] = (number / 10) % 10;
    displayData[3] = number % 10;

    tm1637.display(displayData);
}

void displayRandom(){
    for(int i = 0; i < 4; i++){
        displayData[i] = random(0, 9);
    }
    tm1637.display(displayData);
    delay(600);
}