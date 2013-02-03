// Date and time functions using a DS1307 RTC connected via I2C and Wire lib

#include <Wire.h>
#include "CLCD.h"
#include "RTClib.h"

CLCD lcd(0x06,20,4);
RTC_DS1307 RTC;

void setup () {
  Wire.begin();
  lcd.init();
  lcd.clear();
  RTC.begin();

  if (! RTC.isrunning()) {
    lcd.print("RTC is NOT running!");
  } 
  else {
    RTC.adjust(DateTime(__DATE__, __TIME__)); // set the RTC to the date & time this sketch was compiled
  }
}

void loop () {
  DateTime now = RTC.now();
  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.print(now.year(), DEC);
  lcd.print('/');
  lcd.print(now.month(), DEC);
  lcd.print('/');
  lcd.print(now.day(), DEC);
  lcd.print(' ');
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  lcd.print(now.second(), DEC);
  lcd.cursor_on();
  delay(300);
}


