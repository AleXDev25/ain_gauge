// Main chip
//
// All in one gauge for car and other vehicles
//
// Version 0.3
//
// By Alexander B.
// 2017

#include <OneWire.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <OBD.h>

//---------------------------------------------

#define PIN_DS 9
#define PIN_LED_BUSY 
#define SD_CS_PIN 10

#define latchPin 8
#define clockPin 7
#define dataPin 6

//#define oilSens

#define jp1 14
#define jp2 15

#define v_in 16
#define l_in 17


//-----------------------Create objects----------------------
Adafruit_BMP085 bmp;
COBD obd;
OneWire  ds(PIN_DS);  

//----------------------Var-----------------------
static const char cmds = {"ATIB96\nATIIA13\nATSH8113F1\nATSPA4\nATSW00"};

bool bmp_error,/*sd_error,*/serial_enable;
/*volatile */uint8_t page/*,serial_cmd = 0*/;
float engine_temp,oil_temp,ext_temp,int_temp,atm_pressure,egt,oil_press = 0;
int rpm_val,speed_val = 0;


//---------------------Multitask------------------------

unsigned long SENS_prevMillis = 0; 
const long SENS_interval = 3000;
unsigned long ENG_SENS_prevMillis = 0; 
const long ENG_SENS_interval = 200;
//unsigned long SD_prevMillis = 0; 
//const long SD_interval[3] = {1000,2000,5000};

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


//-------------------SETUP--------------------------

void setup(){
//  Serial.begin(9600);
  
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  //obd.begin();


  //attachInterrupt(0, pagesw, FALLING);


  if (!bmp.begin()) bmp_error=true;
//  if (!SD.begin(SD_CS_PIN)) sd_error=true;  

}

//-----------------LOOP----------------------------

void loop(){
  unsigned long currentMillis = millis();  
   

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

    rpm_val = 4500;
    speed_val = 105;

    //obd.readPID(PID_RPM, rpm_val);
    //obd.readPID(PID_SPEED, speed_val);
    }
  if (currentMillis - SENS_prevMillis >= SENS_interval){
    SENS_prevMillis = currentMillis;
    ext_temp = GetTemp(external); 
    int_temp = bmp.readTemperature();
    atm_pressure = bmp.readPressure() / 133.3;  
    }

//--------------------Datalogger-------------------------

//  if (currentMillis - SD_prevMillis >= SD_interval[1]){
//    SD_prevMillis = currentMillis;
//    String dataString = String (getTime(0)+"  "+engine_temp+','+ext_temp+','+int_temp+','+pressure);
//    File dataFile = SD.open("data.log", FILE_WRITE);
//    if (dataFile) {      
//      dataFile.println(dataString);
//      dataFile.close();
//      
//      //Serial.println(dataString);
//    }    
//    else {
//      sd_error=true;
//      //Serial.println("error opening data.log");
//    }
//  }

    
}

