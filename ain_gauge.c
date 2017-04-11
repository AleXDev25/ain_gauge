// All in one gauge for car and other vehicles
//
// Version 0.1
//
// By Alexander B.
// 2017


#include <U8glib.h>
#include <Bounce2.h>
#include <EEPROM2.h>
#include <OneWire.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 

//---------------------------------------------

#define KEY 5
#define PIN_DS 9
#define PIN_LED_BUSY 
#define DS3231_I2C_ADDRESS 0x68
#define SD_CS_PIN 10

#define latchPin 8
#define clockPin 7
#define dataPin 6

#define jp1 14
#define jp2 15

#define v_in 16
#define l_in 17

#define logo_altezza_width 128
#define logo_altezza_height 17
static unsigned char logo_altezza_bits[] U8G_PROGMEM = {
  0x00, 0x7F, 0x00, 0x1F, 0xE0, 0xFF, 0x7F, 0xFF, 0x3F, 0xFE, 0xFF, 0xF8, 
  0xFF, 0x03, 0xF8, 0x03, 0x80, 0xFF, 0x80, 0x3F, 0xC0, 0xFF, 0x7F, 0xFF, 
  0x3F, 0xFE, 0xFF, 0xF8, 0xFF, 0x03, 0xFC, 0x07, 0x80, 0xFF, 0x80, 0x1F, 
  0xC0, 0xFF, 0x3F, 0xFF, 0x3F, 0xFE, 0xFF, 0xF8, 0xFF, 0x03, 0xFC, 0x07, 
  0xC0, 0xFF, 0x01, 0x3F, 0xE0, 0xFF, 0x7F, 0xFF, 0x3F, 0xFF, 0xFF, 0xF8, 
  0xFF, 0x03, 0xFE, 0x0F, 0xC0, 0xFF, 0x81, 0x1F, 0x00, 0xF8, 0x01, 0x3F, 
  0x00, 0x00, 0x7F, 0x00, 0xFC, 0x01, 0xFE, 0x0F, 0xE0, 0xFF, 0x81, 0x3F, 
  0x00, 0xF8, 0x01, 0x3F, 0x00, 0x80, 0x7F, 0x00, 0xFE, 0x01, 0xFF, 0x0F, 
  0xE0, 0xF7, 0x83, 0x1F, 0x00, 0xF8, 0x01, 0xFF, 0x1F, 0xC0, 0x3F, 0x00, 
  0xFF, 0x00, 0xBF, 0x1F, 0xF0, 0xF3, 0x83, 0x1F, 0x00, 0xF8, 0x01, 0xFF, 
  0x0F, 0xE0, 0x0F, 0x80, 0x7F, 0x80, 0x9F, 0x1F, 0xF8, 0xF3, 0x83, 0x1F, 
  0x00, 0xF8, 0x01, 0xFF, 0x0F, 0xF0, 0x0F, 0xC0, 0x1F, 0xC0, 0x9F, 0x1F, 
  0xF8, 0xFF, 0x87, 0x1F, 0x00, 0xF8, 0x00, 0xFF, 0x0F, 0xF8, 0x03, 0xE0, 
  0x1F, 0xC0, 0xFF, 0x3F, 0xF8, 0xFF, 0x87, 0x3F, 0x00, 0xF8, 0x01, 0x3F, 
  0x00, 0xFC, 0x03, 0xF0, 0x0F, 0xE0, 0xFF, 0x3F, 0xFC, 0xFF, 0x87, 0x1F, 
  0x00, 0xFC, 0x00, 0x3F, 0x00, 0xFE, 0x01, 0xF8, 0x07, 0xE0, 0xFF, 0x3F, 
  0xFE, 0xFF, 0x8F, 0x1F, 0x00, 0xFC, 0x01, 0x3F, 0x00, 0xFF, 0x00, 0xFC, 
  0x03, 0xF0, 0xFF, 0x7F, 0xFE, 0xE0, 0x8F, 0xFF, 0x1F, 0xF8, 0x81, 0xFF, 
  0x1F, 0xFF, 0x7F, 0xFC, 0xFF, 0xF1, 0x07, 0x7F, 0x7E, 0xC0, 0x9F, 0xFF, 
  0x1F, 0xFC, 0x00, 0xFF, 0x1F, 0xFF, 0x7F, 0xFC, 0xFF, 0xF1, 0x03, 0xFE, 
  0x7F, 0xC0, 0xDF, 0xFF, 0x1F, 0xFC, 0x81, 0xFF, 0x1F, 0xFF, 0x7F, 0xFC, 
  0xFF, 0xFD, 0x03, 0xFE, 0x3F, 0x80, 0xDF, 0xFF, 0x1F, 0xFC, 0x81, 0xFF, 
  0x9F, 0xFF, 0x7F, 0xFE, 0xFF, 0xFD, 0x01, 0xFC, };

//----------------------Обьекты-----------------------

Bounce sw = Bounce();
OneWire  ds(PIN_DS);
Adafruit_BMP085 bmp;

//----------------------Переменные-----------------------

bool bmp_error,sd_error;
uint8_t page = 0;
float engine_temp,ext_temp,int_temp,pressure = 0;

byte data1;
byte data2;
byte dataArray1[8];
byte dataArray2[8];

int DISP_CONFIG[11]={0,1,2,3,4,5,6,7,8,9,10}; // 0-Volt 1-water 2-oil 3-in.temp 4-ext.temp 5-press 6-RPM 7-SPEED 8-DATE 9-AFR

//---------------------Многозадачность------------------------

unsigned long SENS_prevMillis = 0; 
const long SENS_interval = 3000;
unsigned long ENG_SENS_prevMillis = 0; 
const long ENG_SENS_interval = 200;
unsigned long SD_prevMillis = 0; 
const long SD_interval = 2000;

//---------------------Sensor ardess------------------------

byte engine[8] = {0x28,0xFF,0x8F,0xDA,0xA4,0x15,0x03,0x6F};
byte external[8] = {0x28,0xFF,0x53,0x81,0xA4,0x15,0x03,0x5C};

//---------------------------------------------
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readVolt(){
  float v = 0.0;
  for(int i=0; i<100; i++){
    v+=analogRead(v_in);
  }
  v=v/100;
  return mapf(v,0.0,1023.0,0.0,15.5);
}

float readLambda(){
  float l = 0.0;
  for(int i=0; i<100; i++){
    l+=analogRead(l_in);
  }
  l=l/100;
  return mapf(l,0.0,254.0,19.7,9.0);
}

//---------------------TIME------------------------

byte bcdToDec(byte val){             // Convert binary coded decimal to normal decimal numbers
  return( (val/16*10) + (val%16) );
}

void readDS3231time(byte *second,byte *minute,byte *hour,byte *dayOfWeek,byte *dayOfMonth,byte *month,byte *year){
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7); 
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

String getTime(){
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  String sec = String (second);
  String mi = String (minute);
  String hr = String (hour);
  String dd = String (dayOfMonth);
  String mm = String (month);
  String yy = String (year);

  String dataStr = String (dd+'/'+mm+'/'+yy+' '+hr+':'+mi+':'+sec);

  return dataStr;
}

//-------------------DISPLAY FUNC.--------------------------

void water_temp(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("WATER TEMP");
  u8g.setPrintPos(x+64,y);
  u8g.println(engine_temp,1);
}

void oil_temp(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("OIL TEMP");
  u8g.setPrintPos(x+64,y);
  u8g.println(75.23,1);
}

void in_temp(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("IN.TEMP");
  u8g.setPrintPos(x+64,y);
  u8g.println(int_temp,1);
}

void ex_temp(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("EXT.TEMP");
  u8g.setPrintPos(x+64,y);
  u8g.println(ext_temp,1);  
}

void atm_pressure(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("ATM.PRESS");
  u8g.setPrintPos(x+64,y);
  u8g.println(pressure,1);
}

void oil_pressure(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("OIL PRESS");
  u8g.setPrintPos(x+64,y);
  u8g.println(8.0,1);
}

void rpm(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("RPM");
  u8g.setPrintPos(x+64,y);
  u8g.println(658);
}

void spd(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("SPEED");
  u8g.setPrintPos(x+64,y);
  u8g.println(37);
}

void date(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("DATE");
  u8g.setPrintPos(x+25,y);
  u8g.println(getTime());
}

void afr(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("AFR");
  u8g.setPrintPos(x+25,y);
  u8g.println(readLambda(),1);
}

void volt(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("VOLT");
  u8g.setPrintPos(x+25,y);
  u8g.println(readVolt(),1);
}

//-------------------DISPLAY CASE--------------------------

void case_upper(uint8_t disp_conf){ // case for 128x0-32
  uint8_t x=2;
  uint8_t y=14;

  switch (disp_conf){
    case 0:
    volt(x,y);
    break;
    case 1:
    water_temp(x,y);
    break;
    case 2:
    oil_temp(x,y);
    break;
    case 3:
    in_temp(x,y);
    break;
    case 4:
    ex_temp(x,y);
    break;
    case 5:
    atm_pressure(x,y);
    break;
    case 6:
    oil_pressure(x,y);
    break;
    case 7:
    spd(x,y);
    break;
    case 8:
    rpm(x,y);
    break;
    case 9:
    date(x,y);
    break;
    case 10:
    afr(x,y);
    break;
  }
}

void case_bottom(uint8_t disp_conf){ // case for 128x32-64
  uint8_t x=2;
  uint8_t y=46;

  switch (disp_conf){
    case 0:
    volt(x,y);
    break;
    case 1:
    water_temp(x,y);
    break;
    case 2:
    oil_temp(x,y);
    break;
    case 3:
    in_temp(x,y);
    break;
    case 4:
    ex_temp(x,y);
    break;
    case 5:
    atm_pressure(x,y);
    break;
    case 6:
    oil_pressure(x,y);
    break;
    case 7:
    spd(x,y);
    break;
    case 8:
    rpm(x,y);
    break;
    case 9:
    date(x,y);
    break;
    case 10:
    afr(x,y);
    break;
  }
}
//---------------------------------------------

float GetTemp(byte *sensor){
  byte data[12];
  ds.reset();
  ds.select(sensor);
  ds.write(0x44, 1);
  delay(50); 
  ds.reset();
  ds.select(sensor);
  ds.write(0xBE);

  for (byte i = 0; i < 10; i++){ 
    data[i] = ds.read();
  }
  int tmp=(data[1] << 8) | data[0];
  float temp =  (float)tmp / 16.0;
  return temp;
}

//---------------------------------------------

void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {

  int i=0;
  int pinState;
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, OUTPUT);

  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);

  for (i=7; i>=0; i--)  {
    digitalWrite(myClockPin, 0);

    if ( myDataOut & (1<<i) ) {
      pinState= 1;
    }
    else {  
      pinState= 0;
    }

    digitalWrite(myDataPin, pinState);
    digitalWrite(myClockPin, 1);
    digitalWrite(myDataPin, 0);
  }
  digitalWrite(myClockPin, 0);
}

void blinkAll(int n, int d) {
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin, 0);
  shiftOut(dataPin, clockPin, 0);
  digitalWrite(latchPin, 1);
  delay(200);
  for (int x = 0; x < n; x++) {
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, 255);
    shiftOut(dataPin, clockPin, 255);
    digitalWrite(latchPin, 1);
    delay(d);
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, 0);
    shiftOut(dataPin, clockPin, 0);
    digitalWrite(latchPin, 1);
    delay(d);
  }
}

void testLed(uint8_t d) {
  dataArray1[0] = 0x01; //00000001
  dataArray1[1] = 0x02; //00000010
  dataArray1[2] = 0x04; //00000100
  dataArray1[3] = 0x08; //00001000
  dataArray1[4] = 0x10; //00010000
  dataArray1[5] = 0x20; //00100000
  dataArray1[6] = 0x40; //01000000
  dataArray1[7] = 0x80; //10000000

  dataArray2[0] = 0x01; //00000001
  dataArray2[1] = 0x02; //00000010
  dataArray2[2] = 0x04; //00000100
  dataArray2[3] = 0x08; //00001000
  dataArray2[4] = 0x10; //00010000
  dataArray2[5] = 0x20; //00100000
  dataArray2[6] = 0x40; //01000000
  dataArray2[7] = 0x80; //10000000

  for (int j = 0; j < 8; j++) {
    data1 = dataArray1[j];
    digitalWrite(latchPin, 0);    
    shiftOut(dataPin, clockPin, 0);   
    shiftOut(dataPin, clockPin, data1);
    digitalWrite(latchPin, 1);
    delay(d);
  }
  for (int j = 0; j < 8; j++) {
    data2 = dataArray2[j];    
    digitalWrite(latchPin, 0);    
    shiftOut(dataPin, clockPin, data2);
    shiftOut(dataPin, clockPin, 0);
    digitalWrite(latchPin, 1);
    delay(d);
  }
  for (int j = 7; j >= 0; j--) {
    data2 = dataArray2[j];    
    digitalWrite(latchPin, 0);    
    shiftOut(dataPin, clockPin, data2);
    shiftOut(dataPin, clockPin, 0);
    digitalWrite(latchPin, 1);
    delay(d);
  }
  for (int j = 7; j >= 0; j--) {
    data1 = dataArray1[j];
    digitalWrite(latchPin, 0);    
    shiftOut(dataPin, clockPin, 0);   
    shiftOut(dataPin, clockPin, data1);
    digitalWrite(latchPin, 1);
    delay(d);
  }  
}

//---------------------------------------------

void setup(){
  //Serial.begin(9600);   //for debugging
  u8g.setRot180();
  
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(KEY, INPUT_PULLUP);
  sw.attach(KEY); 
  sw.interval(5);

  //EEPROM_read(0, page); 

  if (!bmp.begin()) bmp_error=true;
  if (!SD.begin(SD_CS_PIN)) sd_error=true;
  
  delay(100);
  blinkAll(1,300);
  testLed(50);
  delay(200);
  blinkAll(3,200);

  u8g.firstPage();
  do {
    u8g.drawXBMP( 0, 20, logo_altezza_width, logo_altezza_height, logo_altezza_bits);
  } while(u8g.nextPage());
  delay(700);
}

//---------------------------------------------

void loop(){
  unsigned long currentMillis = millis();

  //testLed(100);
  
  sw.update();
  if (sw.fell()) {
    page+=1;
  }
  if(page>5) page=0;

  //EEPROM_write(0, page);

  if (currentMillis - ENG_SENS_prevMillis >= ENG_SENS_interval){
    ENG_SENS_prevMillis = currentMillis;
    engine_temp = GetTemp(engine);    
    }
  if (currentMillis - SENS_prevMillis >= SENS_interval){
    SENS_prevMillis = currentMillis;
    ext_temp = GetTemp(external); 
    int_temp = bmp.readTemperature();
    pressure = bmp.readPressure() / 133.3;  
    }
  if (currentMillis - SD_prevMillis >= SD_interval){
    SD_prevMillis = currentMillis;
    String dataString = String (getTime()+"  "+engine_temp+','+ext_temp+','+int_temp+','+pressure);
    File dataFile = SD.open("data.log", FILE_WRITE);
    if (dataFile) {      
      dataFile.println(dataString);
      dataFile.close();
      
      //Serial.println(dataString);
    }    
    else {
      //Serial.println("error opening data.log");
    }
  }

  u8g.firstPage();
  do {
    switch(page){

        case 0:
        case_upper(DISP_CONFIG[0]);
        case_bottom(DISP_CONFIG[1]);
        break;

        case 1:
        case_upper(DISP_CONFIG[2]);
        case_bottom(DISP_CONFIG[3]);
        break;

        case 2:
        case_upper(DISP_CONFIG[4]);
        case_bottom(DISP_CONFIG[5]);
        break;

        case 3:
        case_upper(DISP_CONFIG[6]);
        case_bottom(DISP_CONFIG[7]);
        break;

        case 4:
        case_upper(DISP_CONFIG[8]);
        case_bottom(DISP_CONFIG[9]);
        break;

        case 5:
        case_upper(DISP_CONFIG[10]);
        case_bottom(DISP_CONFIG[0]);
        break;

        default:
        case_upper(DISP_CONFIG[0]);
        case_bottom(DISP_CONFIG[1]);
        break;
     }
  } while(u8g.nextPage());     
}

