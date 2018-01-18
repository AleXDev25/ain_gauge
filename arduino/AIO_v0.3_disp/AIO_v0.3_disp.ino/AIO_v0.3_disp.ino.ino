// Display chip
//
// All in one gauge for car and other vehicles
//
// Version 0.3
//
// By Alexander B.
// 2017


#include <U8glib.h>
#include <Bounce2.h>
//#include <EEPROM2.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


//---------------------------------------------

#define KEY 5
//#define PIN_LED_BUSY 
#define DS3231_I2C_ADDRESS 0x68
#define SD_CS_PIN 10

#define latchPin 8
#define clockPin 7
#define dataPin 6

//#define oilSens

#define jp1 14
#define jp2 15

#define l_in A2

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

//-----------------------Create objects----------------------
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 
Bounce sw = Bounce(); 

//----------------------Var-----------------------

bool bmp_error,sd_error,serial_enable;
uint8_t page =0;

uint8_t DISP_CONFIG[12]={0,1,2,3,4,5,6,7,8,9,10,11}; // 0-Volt 1-water temp 2-oil temp 3-oil press 4-AFR 5-EGT 6-RPM 7-SPEED 8-in.temp 9-ext.temp 10-atm press 11-DATE
uint8_t LED_CONFIG[2]={0,1}; // 0-AFR 1-RPM

struct DATA_STRUCTURE{
  float voltage;
  float water_temp;
  float oil_temp;
  float oil_press;
  float egt;
  uint8_t rpm_val;
  uint8_t speed_val;  
  float int_temp;
  float ext_temp;  
  float atm_pressure;  
}__attribute__ ((packed));
DATA_STRUCTURE data;

//---------------------Multitask------------------------

unsigned long SD_prevMillis = 0; 
const long SD_interval[3] = {1000,2000,5000};

//---------------------MAF Function fo float------------------------
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//---------------------Read analog sensor functions------------------------

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

String getTime(int opt){
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  String sec = String (second);
  String mi = String (minute);
  String hr = String (hour);
  String dd = String (dayOfMonth);
  String mm = String (month);
  String yy = String (year);

  String allStr = String (dd+'/'+mm+'/'+yy+' '+hr+':'+mi+':'+sec);
  String dateStr = String (dd+'/'+mm+'/'+yy);
  String timeStr = String (hr+':'+mi+':'+sec);

  switch(opt){
    case 0:   
    return allStr;
    break;

    case 1:   
    return dateStr;
    break;

    case 2:   
    return timeStr;
    break;
  }
}

//-------------------DISPLAY FUNC.--------------------------

void WATER_TEMP(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("WATER");
  u8g.setPrintPos(x,y+12);
  u8g.print("TEMP");
  u8g.setFont(u8g_font_freedoomr25n);
  if(data.water_temp<10) u8g.setPrintPos(x+76,y+18);
  else if(data.water_temp<0) u8g.setPrintPos(x+58,y+18);
  else u8g.setPrintPos(x+58,y+18);
  u8g.print(data.water_temp,1);
}

void OIL_TEMP(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("OIL");
  u8g.setPrintPos(x,y+12);
  u8g.print("TEMP");
  u8g.setFont(u8g_font_freedoomr25n);
  if(data.oil_temp<10) u8g.setPrintPos(x+76,y+18);
  else if(data.oil_temp>99) u8g.setPrintPos(x+40,y+18);
  else if(data.oil_temp<0) u8g.setPrintPos(x+58,y+18);
  else u8g.setPrintPos(x+58,y+18);
  u8g.print(data.oil_temp,1);
}

void IN_TEMP(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("IN.");
  u8g.setPrintPos(x,y+12);
  u8g.print("TEMP");
  u8g.setFont(u8g_font_freedoomr25n);
  if(data.int_temp<10) u8g.setPrintPos(x+76,y+18);
  else if(data.int_temp<0) u8g.setPrintPos(x+58,y+18);
  else u8g.setPrintPos(x+58,y+18);
  u8g.print(data.int_temp,1);
}

void EX_TEMP(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("EXT.");
  u8g.setPrintPos(x,y+12);
  u8g.print("TEMP");
  u8g.setFont(u8g_font_freedoomr25n);
  if(data.ext_temp<10) u8g.setPrintPos(x+76,y+18);
  else if(data.ext_temp<0) u8g.setPrintPos(x+58,y+18);
  else u8g.setPrintPos(x+58,y+18);
  u8g.print(data.ext_temp,1); 
}

void ATM_PRESSURE(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("ATM.");
  u8g.setPrintPos(x,y+12);
  u8g.print("PRESS");
  u8g.setFont(u8g_font_freedoomr25n);
  u8g.setPrintPos(x+39,y+18);
  u8g.print(data.atm_pressure,1);
}

void OIL_PRESSURE(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("OIL");
  u8g.setPrintPos(x,y+12);
  u8g.print("PRESS");
  u8g.setFont(u8g_font_freedoomr25n);
  if(data.oil_press<10) u8g.setPrintPos(x+76,y+18);
  else u8g.setPrintPos(x+58,y+18);
  u8g.print(data.oil_press,1);
}

void RPM(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("RPM");
  u8g.setFont(u8g_font_freedoomr25n);
  if(data.rpm_val<100) u8g.setPrintPos(x+86,y+18);
  if(data.rpm_val>=100) u8g.setPrintPos(x+68,y+18);
  if(data.rpm_val>=1000)u8g.setPrintPos(x+50,y+18);
  u8g.print(data.rpm_val);
}

void SPD(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("SPEED");
  u8g.setFont(u8g_font_freedoomr25n);
  if(data.speed_val<10) u8g.setPrintPos(x+104,y+18);
  if(data.speed_val>=10) u8g.setPrintPos(x+86,y+18);
  if(data.speed_val>=100)u8g.setPrintPos(x+68,y+18);
  u8g.print(data.speed_val);
}

void rtc_date(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("DATE");
  u8g.setPrintPos(x+40,y);
  u8g.print(getTime(1));
  u8g.setPrintPos(x+40,y+14);
  u8g.print(getTime(2));
  // u8g.print("04.05.2017");
  // u8g.setPrintPos(x+40,y+14);
  // u8g.print("14:32");
}

void AFR(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("AFR");
  u8g.setFont(u8g_font_freedoomr25n);
  if(readLambda()<10) u8g.setPrintPos(x+76,y+18);
  else u8g.setPrintPos(x+58,y+18);
  u8g.print(readLambda(),1);
}

void VOLT(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("VOLT");
  u8g.setFont(u8g_font_freedoomr25n);
  if(data.voltage<10) u8g.setPrintPos(x+76,y+18);
  else u8g.setPrintPos(x+58,y+18);
  u8g.print(data.voltage,1);
}

void EGT(uint8_t x, uint8_t y){
  u8g.setFont(u8g_font_7x14Br);
  u8g.setPrintPos(x,y);
  u8g.print("EGT");
  u8g.setFont(u8g_font_freedoomr25n);
  u8g.setPrintPos(x+58,y+18);
  u8g.print(data.egt,1);
}

//-------------------DISPLAY CASE--------------------------
// 0-Volt 1-water temp 2-oil temp 3-oil press 4-AFR 5-EGT 6-RPM 7-SPEED 8-in.temp 9-ext.temp 10-atm press 11-DATE
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
    OIL_PRESSURE(x,y);
    break;
    case 4:
    AFR(x,y);
    break;
    case 5:
    EGT(x,y);
    break;
    case 6:
    RPM(x,y);
    break;
    case 7:
    SPD(x,y);
    break;
    case 8:
    IN_TEMP(x,y);
    break;
    case 9:
    EX_TEMP(x,y);
    break;
    case 10:
    ATM_PRESSURE(x,y);
    break;
    case 11:
    rtc_date(x,y);
    break;
  }
}
// 0-Volt 1-water temp 2-oil temp 3-oil press 4-AFR 5-EGT 6-RPM 7-SPEED 8-in.temp 9-ext.temp 10-atm press 11-DATE
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
    OIL_PRESSURE(x,y);
    break;
    case 4:
    AFR(x,y);
    break;
    case 5:
    EGT(x,y);
    break;
    case 6:
    RPM(x,y);
    break;
    case 7:
    SPD(x,y);
    break;
    case 8:
    IN_TEMP(x,y);
    break;
    case 9:
    EX_TEMP(x,y);
    break;
    case 10:
    ATM_PRESSURE(x,y);
    break;
    case 11:
    rtc_date(x,y);
    break;
  }
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
  data.voltage = 0;
  data.water_temp = 0;
  data.oil_temp = 0;
  data.oil_press = 0;
  data.egt = 0;
  data.rpm_val = 0;
  data.speed_val = 0;  
  data.int_temp = 0;
  data.ext_temp = 0;  
  data.atm_pressure = 0; 
  
//  Serial.begin(9600);
  u8g.setRot180();
  
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(jp1, INPUT_PULLUP);
  pinMode(jp2, INPUT_PULLUP);
  pinMode(KEY, INPUT_PULLUP);
  sw.attach(KEY); 
  sw.interval(5);

  //EEPROM_read(0, page); 

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

  // do {
  //   blinkAll(3,100);
  // } while (!obd.init());
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



//--------------------Datalogger-------------------------

 if (currentMillis - SD_prevMillis >= SD_interval[1]){
   SD_prevMillis = currentMillis;
   String dataString = String (getTime(0)+"  "+data.water_temp+','+data.ext_temp+','+data.int_temp+','+data.atm_pressure);
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
 
//---------------------Led out------------------------

  //bar_led(rpm_val, 9000);
//---------------------Display out------------------------

  u8g.firstPage();
  do {
    u8g.drawHLine(0,32,129);
    
    switch(page){
    // 0-Volt 1-water temp 2-oil temp 3-oil press 4-AFR 5-EGT 6-RPM 7-SPEED 8-in.temp 9-ext.temp 10-atm press 11-DATE
        case 0:
        case_upper(DISP_CONFIG[4]);
        case_bottom(DISP_CONFIG[0]);
        break;

        case 1:
        case_upper(DISP_CONFIG[2]);
        case_bottom(DISP_CONFIG[3]);
        break;

        case 2:
        case_upper(DISP_CONFIG[6]);
        case_bottom(DISP_CONFIG[7]);
        break;

        case 3:
        case_upper(DISP_CONFIG[8]);
        case_bottom(DISP_CONFIG[9]);
        break;

        case 4:
        case_upper(DISP_CONFIG[10]);
        case_bottom(DISP_CONFIG[11]);
        break;

        case 5:
        case_upper(DISP_CONFIG[1]);
        case_bottom(DISP_CONFIG[0]);
        break;
     }
  } while(u8g.nextPage());     
}

