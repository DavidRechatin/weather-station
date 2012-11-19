String v = "0.10"; 
/* Weather station v0.10 by David Réchatin  
 
 Weather station with web connexion
 
https://github.com/DavidRechatin/weather-station
 
 This software is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 The circuit:
 Arduino MEGA 2650 R3
 
 Sensor : TEMT6000
 * VCC connection of the sensor attached to +5V
 * GND connection of the sensor attached to ground
 * SIG connection of the sensor attached to analog pin A8
 
 Sensor : RHT03 (alias DHT22)
 * Pin1 connection of the sensor attached to +5V
 * Pin2 connection of the sensor attached to digital pin 30 with 1K resistor to +5V
 * Pin3 connection of the sensor attached to ground
 * Pin4 connection of the sensor attached to ground
 
 Sensor : SHT15
 * VCC  connection of the sensor attached to +5V
 * DATA connection of the sensor attached to digital pin 28
 * SCK  connection of the sensor attached to digital pin 29
 * GND  connection of the sensor attached to ground
 
 Sensor : BMP085
 * VNI  connection of the sensor attached to +5V
 * SLC connection of the sensor attached to digital pin 21 > I2C bus
 * SDA  connection of the sensor attached to digital pin 20 > I2C bus
 * GND  connection of the sensor attached to ground
 
 Sensor : wind speed
 * Pin2 connection of the sensor attached to +5V
 * Pin3 connection of the sensor attached to digital pin 2 > interrupt 0 with 10K resistor to ground
 
 Sensor : wind direction
 * Pin1 connection of the sensor attached to analog pin A9 with 10K resistor to +5V
 * Pin4 connection of the sensor attached to ground
 
 Sensor : rain gauge
 * Pin2 connection of the sensor attached to +5V
 * Pin3 connection of the sensor attached to digital pin 3 > interrupt 1 with 10K resistor to ground
 
 LED working in progress in loop
 * to digital pin 40
 
 Button
 * Pin1 connection of the button attached to +5V
 * Pin2 connection of the button to digital pin 18 > interrupt 5 with 10K resistor to ground
 
 LCD display 16x2 (Hitachi 44780 compatible)
 * LCD RS     pin to digital pin 22
 * LCD Enable pin to digital pin 23
 * LCD Data 4 pin to digital pin 24
 * LCD Data 5 pin to digital pin 25
 * LCD Data 6 pin to digital pin 26
 * LCD Data 7 pin to digital pin 27
 * LCD R/W pin to ground
 * LCD VO pin to potentiometer 10K between +5V and ground  (contrast)
 
 http://www.rechatin.com/
 
 created 31 oct 2012
 by David Réchatin  
 
 v0.1 31/10/2012 : test RHT03 sensor (temperature + humdity) and TEMT6000 sensor (luminosity)
 v0.2 01/11/2012 : add LCD display    
 v0.3 01/11/2012 : add SHT15 sensor (temperature + humdity)  
 v0.4 01/11/2012 : add BMP085 sensor (temperature + pressure)
 v0.5 01/11/2012 : add button for select display information 
 v0.6 02/11/2012 : add windvane 
 + begin wind speed with irq_windspeed() and led wind sensor interrupt
 + begin rain gauge with irq_raingauge() and led rain gauge sensor interrupt 
 v0.7 06/11/2012 : add altiide of BMP085 sensor 
 v0.8 15/11/2012 : wind speed calculation with number of closure in each loop
 v0.9 15/11/2012 : wind speed advanced calculation 
 v0.10 15/11/2012 : rain height calculation 
 
 
 
 todo list :
 - define the pin numbers with const
 - BMP085 pressure error

 */

// include the library code:
#include <Wire.h>
#include <LiquidCrystal.h>    // require by LCD display
#include <dht.h>              // require by RHT03
#include <SHT1x.h>            // require by SHT15
#include <Adafruit_BMP085.h>  // require by BMP085
#include <SPI.h>              // require by ethernet shield
#include <Ethernet.h>         // require by ethernet shield

// initialize LCD display
LiquidCrystal lcd(22, 23, 24, 25, 26, 27); // initialize the library with the numbers of the interface pins

// initialize RHT03 (alias DHT22)
dht DHT;
#define DHT22_PIN 30

// initialize SHT15
#define rhtDataPin  28
#define rhtClockPin 29
SHT1x sht1x(rhtDataPin, rhtClockPin);

// initialize BMP085
Adafruit_BMP085 bmp;

// initialize ethernet shield
byte mac[] = { 
  0x90, 0xA2, 0xDA, 0x0D, 0x80, 0xF3 };  // MAC address printed on a sticker on the shield
IPAddress server(192,168,0,230);   // IP address of server
EthernetClient client;   // Initialize the Ethernet client library

// Variable declaration
String debug = "" ; // for debuging
int disp_state = 0; // use by interrup 5 : function irq_button()
String lcd1 = "" ; // line 1 of LCD display
String lcd2 = "" ; // line 2 of LCD display
char* tmp_char = "" ;  // temporary variable for increase code lisibilty
String tmp_string = "" ;  // temporary variable for increase code lisibilty
unsigned long time = 0 ; // time
int i = 0; // for loop 'for'

float TEMT6000_l = 0 ;   // luminosity
float RTH03_t = 0 ;    // temperature
float RTH03_h = 0 ;    // humdity
float SHT15_t = 0 ;    // temperature
float SHT15_h = 0 ;    // humdity
float BMP085_t = 0 ;   // temperature
float BMP085_p = 0 ;   // pressure
float BMP085_a = 0 ;   // altitude
String windVane = "" ;  // wind direction
float windSpeed = 0 ;  // wind speed
volatile int countWindSpeed = 0 ;  // count wind speed contact
unsigned long firstContactTime = 0 ; // // first time of wind speed interupt
volatile unsigned long lastContactTime = 0 ; // // last time of wind speed interupt
float rainHeight = 0 ;  // height of rain
volatile int countRain = 0 ;  // count rain gauge contact

// Parameters declaration
const byte nMenu = 7 ; // number of menu on LCD display
const int waitLoop = 3000 ; // time (in milli second) of wait in loop cycle

// ###############################################################################
// BEGIN FUNCTIONS

// =============================================================
// DISPLAY DATA
// generate data and send it to lcd display
void display_data()
{  
  // -------------------------------------
  // Generate display data
  tmp_char = "0000";
  switch (disp_state) { // state of information display
  case 0: //
    lcd1 = "Station meteo";
    lcd2 = "version " + v; 
    break;
  case 1: 
    lcd1 = "Temperature *C"; 
    dtostrf(RTH03_t,2,1,tmp_char);  // conv. float to string
    lcd2 = tmp_char; 
    lcd2 += " ";
    dtostrf(SHT15_t,2,1,tmp_char);  // conv. float to string
    lcd2 += tmp_char;  
    lcd2 += " ";
    dtostrf(BMP085_t,2,1,tmp_char);  // conv. float to string
    lcd2 += tmp_char;   
    break;
  case 2: 
    lcd1 = "Humidite %";
    dtostrf(RTH03_h,2,1,tmp_char);  // conv. float to string
    lcd2 = tmp_char;  
    lcd2 += " ";
    dtostrf(SHT15_h,2,1,tmp_char);  // conv. float to string
    lcd2 += tmp_char;  
    break;
  case 3: 
    lcd1 = "Pression mBar";
    dtostrf(BMP085_p,2,0,tmp_char);  // conv. float to string
    lcd2 = tmp_char;   
    break;
  case 4: 
    lcd1 = "Luminosite %";    
    dtostrf(TEMT6000_l,2,2,tmp_char);  // conv. float to string
    lcd2 = tmp_char;   
    break;
  case 5: 
    lcd1 = "Vent";
    dtostrf(windSpeed,2,1,tmp_char);  // conv. float to string
    lcd2 = tmp_char;
    lcd2 += " km/h >> " + windVane;
    break;
  case 6: 
    lcd1 = "Pluie";
    dtostrf(rainHeight,2,2,tmp_char);  // conv. float to string
    lcd2 = tmp_char;
    lcd2 += " mm";
    break;
  default: 
    disp_state = 0; // security...
  }

  // -------------------------------------
  // Send data to LCD display
  lcd.clear();  // clear and set the cursor to column 0, line 1
  lcd.print(lcd1);
  lcd.setCursor(0, 1);  // (col,row) > set the cursor to column 0, line 2
  lcd.print(lcd2);  

  // SEND DATA to Serial
  //Serial.println("-----------------------");
  // Serial.println("## " + debug + " ##");    // for debuging
  //Serial.println(lcd1);
  //Serial.println(lcd2);
}

// =============================================================
// CALCULATION WIND DIRECTION
String getWindVane() {
#define NUMDIRS 16   // number of values of wind direction
  float windvaneVal[NUMDIRS] = {
    73.5, 87.5, 108.5, 155, 215, 266.5, 348, 436, 533, 617.5, 668.5, 746, 809, 860, 918.5, 1023               }; // median values measured at the analog input  
  char *windvaneStr[NUMDIRS] = {
    "ONO","OSO", "O","NNO", "NO", "NNE", "N", "SSO", "SO", "ENE", "NE", "SSE", "S", "ESE", "SE", "E"               };  // wind directions
  float windvaneDeg[NUMDIRS] = {
    0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5               }; // degre direction  
  unsigned int val; // analog input value
  byte x;  // index

    val = analogRead(9);  // read analog input pin 9
  for (x=0; x<NUMDIRS; x++) {   // search index
    if (windvaneVal[x] >= val)  // in windvaneVal with a value below the median value higher
      break;
  }
  // return the wind direction using index
  tmp_string = String(windvaneStr[x]);
  return String(windvaneStr[x]) ; 
}

// =============================================================
// CALCULATION WIND SPEED
float getWindSpeed() {    
  float calc = 0 ;
  if (countWindSpeed == 0) // if we haven't contact
  {  
    calc = 0; // no wind
  } 
  else   // we have contact (one or more)
  { 
    // calculation of speed 
    calc = countWindSpeed / (float(lastContactTime - firstContactTime) / 1000) * 1.2 ; // impulses / time * 2,4 km/h per second / 2 switch closure per revolution
    firstContactTime = lastContactTime;  // stores the time of last contact
    countWindSpeed = 0;
  }  
  return calc; 
}

// =============================================================
// CALCULATION RAIN GAUGE
float getRainHeight() {    
  float calc = 0 ;  
  calc = countRain * 0.2794 ; 
  //countRain = 0;
  return calc;
}

// =============================================================
// BUTTON INTERUPTION
void irq_button() // interrupt 5
{
  disp_state++;   // next state of display
  if (disp_state == nMenu) {  // state no exist
    disp_state=0;  // loop to state 0
  }
  display_data();  // generate data and send it to lcd display
}

// =============================================================
// WIND SPEED SENSOR INTERUPTION
void irq_windspeed() // interrupt 0
{
  countWindSpeed++; 
  lastContactTime = millis();
}

// =============================================================
// RAIN GAUDE SENSOR INTERUPTION
void irq_raingauge() // interrupt 1
{
  countRain++; 
}

// END FUNCTIONS
// ###############################################################################



// ###############################################################################
// BEGIN SETUP
void setup()
{
  Serial.begin(9600);
  //Serial.println("Setup begin");
  pinMode(40, OUTPUT); // define pin of LED reading sensors

  // interruption by button
  attachInterrupt (5, irq_button, FALLING);
  // interruption by wind wensor
  attachInterrupt (0, irq_windspeed, FALLING);
  // interruption by rain gauge
  attachInterrupt (1, irq_raingauge, FALLING);

  // LCD Display
  lcd.begin(16, 2);   

  // BMP085
  if (!bmp.begin()) {
    //Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {
    }
  }

  /*
  // Ethernet shield
   Serial.println("Begin Ethernet shield...");
   // start the Ethernet connection:
   if (Ethernet.begin(mac) == 0) {
   Serial.println("Failed to configure Ethernet using DHCP");
   // no point in carrying on, so do nothing forevermore:
   for(;;)
   ;
   }
   */

  //Serial.println("Setup end");
}
// END SETUP
// ###############################################################################




// ###############################################################################
// BEGIN LOOP
void loop()
{
  digitalWrite(40, HIGH);   // turn ON the LED working in progress in loop

  // initialize variable
  debug = "";

  // =============================================================
  // READ SENSORS DATA

  // read data > TEMT6000
  TEMT6000_l = float(analogRead(8))*100/1023;       // read analog input pin 8

  // read data  > RHT03 (alias DTH22)
  int chk = DHT.read22(DHT22_PIN);
  /*
  switch (chk)
   {
   case DHTLIB_OK:  
   debug = "DHT22 > OK";
   break;
   case DHTLIB_ERROR_CHECKSUM:
   debug = "DHT22 > Checksum error !";
   break;
   case DHTLIB_ERROR_TIMEOUT:
   debug = "DHT22 > Time out error !";
   break;
   default:
   debug = "DHT22 > Unknown error !";
   break;
   }
   */
  RTH03_t = DHT.temperature;
  RTH03_h = DHT.humidity;

  // read data  > STH15
  SHT15_t = sht1x.readTemperatureC(); // it's slow !!
  SHT15_h = sht1x.readHumidity(); // it's slow !!

  // read data  > BMP085
  BMP085_t = bmp.readTemperature();
  BMP085_p = bmp.readPressure()/100;   // 1 mbar = 100 Pa 

  // read data > wind direction
  windVane = getWindVane();  

  // read data > wind speed
  windSpeed = getWindSpeed(); 

  // read data > rain height
  rainHeight = getRainHeight(); 

  // =============================================================
  // DISPLAY DATA
  display_data();

  // =============================================================
  // OTHER TREATMENTS
  digitalWrite(40, LOW);   // turn OFF the LED working in progress in loop
  delay(waitLoop);   // wait
}

// END LOOP
// ###############################################################################



//
// END OF FILE
//
























