// All in one gauge for car and other vehicles
//
// Version 0.2
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
//#define PIN_LED_BUSY 
#define DS3231_I2C_ADDRESS 0x68
#define SD_CS_PIN 10

#define latchPin 8
#define clockPin 7
#define dataPin 6

//#define oilSens

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

//----------------------Objects-----------------------

Bounce sw = Bounce();
OneWire  ds(PIN_DS);
Adafruit_BMP085 bmp;

//----------------------Var-----------------------

bool bmp_error,sd_error,serial_enable;
uint8_t page,serial_cmd = 0;
float engine_temp,oil_temp,ext_temp,int_temp,pressure,egt = 0;

uint8_t DISP_CONFIG[12]={0,1,2,3,4,5,6,7,8,9,10,11}; // 0-Volt 1-water temp 2-oil temp 3-in.temp 4-ext.temp 5-atm press 6-oil press 7-SPEED 8-RPM 9-DATE 10-AFR 11-EGT
uint8_t LED_CONFIG[2]={0,1}; // 0-AFR 1-RPM

//---------------------Multitask------------------------

unsigned long SENS_prevMillis = 0; 
const long SENS_interval = 3000;
unsigned long ENG_SENS_prevMillis = 0; 
const long ENG_SENS_interval = 200;
unsigned long SD_prevMillis = 0; 
const long SD_interval[3] = {1000,2000,5000};

//---------------------Sensor ardess------------------------

byte engine[8] = {0x28,0xFF,0x8F,0xDA,0xA4,0x15,0x03,0x6F};
byte external[8] = {0x28,0xFF,0x53,0x81,0xA4,0x15,0x03,0x5C};
//byte oil[] = {0x,0x,0x,0x,0x,0x,0x,0x};

//---------------------MAF Function fo float------------------------
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//---------------------Read analog sensor functions------------------------

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

void WATER_TEMP(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("WATER TEMP");
  u8g.setPrintPos(x+64,y);
  u8g.println(engine_temp,1);
}

void OIL_TEMP(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("OIL TEMP");
  u8g.setPrintPos(x+64,y);
  u8g.println(oil_temp,1);
}

void IN_TEMP(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("IN.TEMP");
  u8g.setPrintPos(x+64,y);
  u8g.println(int_temp,1);
}

void EX_TEMP(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("EXT.TEMP");
  u8g.setPrintPos(x+64,y);
  u8g.println(ext_temp,1);  
}

void ATM_PRESSURE(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("ATM.PRESS");
  u8g.setPrintPos(x+64,y);
  u8g.println(pressure,1);
}

void OIL_PRESSURE(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("OIL PRESS");
  u8g.setPrintPos(x+64,y);
  u8g.println(8.0,1);
}

void RPM(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("RPM");
  u8g.setPrintPos(x+64,y);
  u8g.println(658);
}

void SPD(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("SPEED");
  u8g.setPrintPos(x+64,y);
  u8g.println(37);
}

void rtc_date(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("DATE");
  u8g.setPrintPos(x+25,y);
  u8g.println(getTime());
}

void AFR(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("AFR");
  u8g.setPrintPos(x+25,y);
  u8g.println(readLambda(),1);
}

void VOLT(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("VOLT");
  u8g.setPrintPos(x+25,y);
  u8g.println(readVolt(),1);
}

void EGT(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_6x13);
  u8g.setPrintPos(x,y);
  u8g.println("EGT");
  u8g.setPrintPos(x+25,y);
  u8g.println(egt,1);
}

//-------------------DISPLAY CASE--------------------------

void case_upper(uint8_t disp_conf){ // case for 128x0-32
  uint8_t x=2;
  uint8_t y=14;

  switch (disp_conf){
    case 0:
    VOLT(x,y);
    break;
    case 1:
    WATER_TEMP(x,y);
    break;
    case 2:
    OIL_TEMP(x,y);
    break;
    case 3:
    IN_TEMP(x,y);
    break;
    case 4:
    EX_TEMP(x,y);
    break;
    case 5:
    ATM_PRESSURE(x,y);
    break;
    case 6:
    OIL_PRESSURE(x,y);
    break;
    case 7:
    SPD(x,y);
    break;
    case 8:
    RPM(x,y);
    break;
    case 9:
    rtc_date(x,y);
    break;
    case 10:
    AFR(x,y);
    break;
    case 11:
    EGT(x,y);
    break;
  }
}

void case_bottom(uint8_t disp_conf){ // case for 128x32-64
  uint8_t x=2;
  uint8_t y=46;

  switch (disp_conf){
    case 0:
    VOLT(x,y);
    break;
    case 1:
    WATER_TEMP(x,y);
    break;
    case 2:
    OIL_TEMP(x,y);
    break;
    case 3:
    IN_TEMP(x,y);
    break;
    case 4:
    EX_TEMP(x,y);
    break;
    case 5:
    ATM_PRESSURE(x,y);
    break;
    case 6:
    OIL_PRESSURE(x,y);
    break;
    case 7:
    SPD(x,y);
    break;
    case 8:
    RPM(x,y);
    break;
    case 9:
    rtc_date(x,y);
    break;
    case 10:
    AFR(x,y);
    break;
    case 11:
    EGT(x,y);
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

//------------------LED CONTROL---------------------------

void blinkAll(int n, int d) {
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin, MSBFIRST, 0);
  shiftOut(dataPin, clockPin, MSBFIRST, 0);
  digitalWrite(latchPin, 1);
  delay(100);
  for (int x = 0; x < n; x++) {
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, MSBFIRST, 255);
    shiftOut(dataPin, clockPin, MSBFIRST, 255);
    digitalWrite(latchPin, 1);
    delay(d);
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, MSBFIRST, 0);
    shiftOut(dataPin, clockPin, MSBFIRST, 0);
    digitalWrite(latchPin, 1);
    delay(d);
  }
}
  
void testLed(uint8_t d) {
  uint16_t data = 0;

  data = 1;
  for(byte i=0; i<15; i++){
    data=data << 1;
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, MSBFIRST, (data >> 8));
    shiftOut(dataPin, clockPin, MSBFIRST, data);
    digitalWrite(latchPin, 1);
    delay(d);
  }
    for(byte i=0; i<15; i++){
    data=data >> 1;
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, MSBFIRST, (data >> 8));
    shiftOut(dataPin, clockPin, MSBFIRST, data);
    digitalWrite(latchPin, 1);
    delay(d);
  }
}

void bar_led(int in_data, int max_data){
  uint8_t cur_data = map(in_data,0,max_data,0,16);
  uint16_t data = 0;

  if(cur_data>0 && cur_data<2) data = 1;
  if(cur_data>1 && cur_data<=16){
    data = 1;
    for(byte i=0; i<cur_data-1; i++){
      data=(data << 1) + 1;
    }
  } 
  
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin, MSBFIRST, (data >> 8));
  shiftOut(dataPin, clockPin, MSBFIRST, data);
  digitalWrite(latchPin, 1);
}

void dot_led(int in_data, int max_data){
  uint8_t cur_data = map(in_data,0,max_data,0,16);  
  uint16_t data = 0;

  if(cur_data>0 && cur_data<2) data = 1;
  if(cur_data>1 && cur_data<=16){
    data = 1;
    for(byte i=0; i<cur_data-1; i++){
      data=data << 1;
    }
  } 
  
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin, MSBFIRST, (data >> 8));
  shiftOut(dataPin, clockPin, MSBFIRST, data);
  digitalWrite(latchPin, 1);
}

//-------------------SETUP--------------------------

void setup(){
  Serial.begin(9600);
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
  delay(900);
}

//-----------------LOOP----------------------------

void loop(){
  unsigned long currentMillis = millis();
  
  sw.update();
  if (sw.fell()) {
    page+=1;
  }
  if(page>5) page=0;

  //EEPROM_write(0, page);

//--------------------Serial interface------------------------  

//  if (Serial.available() > 0) { 
//        char bufer[] = Serial.read();
//
//
//    }


//--------------------Data from sensors-------------------------

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

//--------------------Datalogger-------------------------

  if (currentMillis - SD_prevMillis >= SD_interval[1]){
    SD_prevMillis = currentMillis;
    String dataString = String (getTime()+"  "+engine_temp+','+ext_temp+','+int_temp+','+pressure);
    File dataFile = SD.open("data.log", FILE_WRITE);
    if (dataFile) {      
      dataFile.println(dataString);
      dataFile.close();
      
      //Serial.println(dataString);
    }    
    else {
      sd_error=true;
      //Serial.println("error opening data.log");
    }
  }

//---------------------Display out------------------------

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
        case_bottom(DISP_CONFIG[11]);
        break;

        default:
        case_upper(DISP_CONFIG[0]);
        case_bottom(DISP_CONFIG[1]);
        break;
     }
  } while(u8g.nextPage());     
}

