#include <Adafruit_INA219.h>
#include <LiquidCrystal.h>
#include <CayenneLPP.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <Wire.h>
#include "DHT.h"

Adafruit_INA219 ina219;

template <class T, size_t N> constexpr size_t len(const T(&)[N]) { return N; }

// This defines the LCD wiring to the DIGITALpins
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize time related variables
unsigned long ts = 0UL, tm = 0UL, tstamp = 0UL, interval = 600UL;

// For accurate Time reading, use Arduino Real Time Clock
static uint32_t last_time = 0; // RTC
 
// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
#define DHTA 22
#define DHTB 23
#define DHTTYPE DHT22
#define TIME_HEADER  "T"

// TDS variables
#define TdsPin A8
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,temperature = 25, tds=0, ec=0;

// PH variables
#define PhPin A9
float calibration_value = 21.34 - 0.7;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;

// INA219 variables
uint32_t currentFrequency;
float PowSec[60], PowMin[60], PowSecAve=0, PowMinAve=0, mWh=0, loadV=0;
int PowSecInd=0, PowMinInd=0;

//init somemore variables
const int relay1=24, relay2=25, relay3=26, relay4=27;
bool timer=false, aut=false, change=false, invalid=false;
int onPeriod=0, offPeriod=0, h=0;
int tThreshold=0, hThreshold=0, pumpElapse=0, nextSwitch=0, line=1, idl=0;
float hA, hB, tA, tB, pH=0, rec[12]={0,0,0,0,0,0,0,0,0,0,0,0};
DHT dhtA(DHTA, DHTTYPE);
DHT dhtB(DHTB, DHTTYPE);

// read the buttons
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor 
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1500) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 195)  return btnUP; 
 if (adc_key_in < 380)  return btnDOWN; 
 if (adc_key_in < 500)  return btnLEFT; 
 if (adc_key_in < 900)  return btnSELECT;   
 return btnNONE;  // when all others fail, return this...
}

void pumpTimer(){
  if(timer==true || (pumpElapse!=0 && digitalRead(relay1)==true)){
    pumpElapse++;
    if ((digitalRead(relay1)==false && pumpElapse >= offPeriod) || (digitalRead(relay1)==true && pumpElapse >= onPeriod)){
      pumpElapse=0;
      digitalWrite(relay1,!digitalRead(relay1));
      change=true;
    }
  }
}

void PHSensor(){
  for(int i=0;i<10;i++) { 
   buffer_arr[i]=analogRead(PhPin);
   delay(30);
  }
  for(int i=0;i<9;i++){
   for(int j=i+1;j<10;j++){
     if(buffer_arr[i]>buffer_arr[j]){
       temp=buffer_arr[i];
       buffer_arr[i]=buffer_arr[j];
       buffer_arr[j]=temp;
     }
   }
  }
  avgval=0;
  for(int i=2;i<8;i++)
  avgval+=buffer_arr[i];
  float volt=(float)avgval*5.0/1024/6; 
  pH = -5.70 * volt + calibration_value;
}

void TdsMeter(){
  while (true){
    static unsigned long analogSampleTimepoint = millis();
    
    if(millis()-analogSampleTimepoint > 40U){ //every 40 milliseconds,read the analog value from the ADC{
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsPin); //read the analog value and store into the buffer
      analogBufferIndex++;
      if(analogBufferIndex == SCOUNT) analogBufferIndex = 0;
    }
    
    static unsigned long printTimepoint = millis();
    
    if(millis()-printTimepoint > 800U){
      printTimepoint = millis();
      
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++) analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF/ 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient; //temperature compensation
      
      tds = (133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      ec = 1.12*tds;
      break;
    }
  }
}

int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++) bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++){
    for (i = 0; i < iFilterLen - j - 1; i++){
      if (bTab[i] > bTab[i + 1]){
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  
  if ((iFilterLen & 1) > 0) bTemp = bTab[(iFilterLen - 1) / 2];
  else bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  
  return bTemp;
}

void HTSensor(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
//  hA = int(map(dhtA.readHumidity(), 35, 81, 38, 79));
//  if (hA > 100)hA=100;
//  hB = int(map(dhtB.readHumidity(), 35, 76, 38, 75));
//  if (hB > 100)hB=100;
  hA = dhtA.readHumidity();
  if (hA > 100)hA=100;
  hB = dhtB.readHumidity();
  if (hB > 100)hB=100;
  // Read temperature as Celsius (the default)
  tA = dhtA.readTemperature();
  tB = dhtB.readTemperature();

  // Compute heat index in Celsius (isFahreheit = false)
//  float hicA = dhtA.computeHeatIndex(tA, hA, false);
//  float hicB = dhtB.computeHeatIndex(tB, hB, false);

  if(aut==true){
    if (hA<=hThreshold || hB<=hThreshold || tA>=tThreshold || tB>=tThreshold) timer=true;
    else timer=false;
  }  
  else timer=false;
}

void variablesManager(){
  if (now() >= ts+1){
    idl++;
    ts = now();
    if (PowSecInd < 60){
      PowSec[PowSecInd] = ina219.getPower_mW();
      PowSecInd++;
    }
    else if (PowSecInd >= 60){
      for(int i=0;i<60;i++){
        PowSecAve = PowSecAve + PowSec[i];
      }
      PowSecAve = PowSecAve/60;
      PowSecInd = 0;
    }
  }
  if (now() >= tm+60){
    if(timer==true) pumpElapse++;
    tm = now();
    if (PowMinInd < 60){
      PowMin[PowMinInd] = PowSecAve;
      PowMinInd++;
    }
    else if (PowMinInd >= 60){
      for(int i=0;i<60;i++){
        PowMinAve = PowMinAve + PowMin[i];
      }
      mWh = PowMinAve/60;
      PowMinInd = 0;
    }
  }
  if (now() >= tstamp+interval){
    HTSensor();
	  TdsMeter();
    PHSensor();
    float shuntvoltage = ina219.getShuntVoltage_mV();
    float busvoltage = ina219.getBusVoltage_V(); 
    loadV = busvoltage + (shuntvoltage / 1000);
    change = true;
    tstamp = now();
  }
}

void kypdBtn(){  
 lcd_key = read_LCD_buttons();  // read the buttons
  if(lcd_key==btnLEFT){ 
    idl=0;
    delay(200);
  }
  if(lcd_key==btnRIGHT){
    idl=0;
    delay(200);
  }
  if(lcd_key==btnSELECT){
    digitalWrite(bl,HIGH);
    lcd.display();
    idl=0;
    delay(200);
  }
  if(lcd_key==btnUP){
    line--;
    if(line<1)line=8;
    idl=0;
    delay(200);
  }
  if(lcd_key==btnDOWN){
    line++;
    if(line>8)line=1;
    idl=0;
    delay(200);
  }
  if(idl>=60){
    digitalWrite(bl,LOW);
    lcd.noDisplay();
    line=1;
  }
}

time_t requestSync()
{
  CayenneLPP lpp (160);
  lpp.reset();
  lpp.addDigitalInput(0, 0);
  lpp.addDigitalInput(0, 0);
  Serial.write(lpp.getBuffer(),lpp.getSize());
  Serial.println();
  delay(250);
  return 0; // the time will be sent later in response to serial mesg
}

void serComm(){
  CayenneLPP lpp (160);
  lpp.reset();
    
  if (change==true){
    lpp.reset();
    if (rec[0]!=int(digitalRead(relay1))){
      lpp.addDigitalInput(relay1,digitalRead(relay1));
      lpp.addDigitalInput(relay1,digitalRead(relay1));
      rec[0]=int(digitalRead(relay1));
    }
    else if (rec[1]!=int(digitalRead(relay2))){
      lpp.addDigitalInput(relay2,digitalRead(relay2));
      lpp.addDigitalInput(relay2,digitalRead(relay2));
      rec[1]=int(digitalRead(relay2));
    }
    else if (rec[2]!=int(digitalRead(relay3))){
      lpp.addDigitalInput(relay3,digitalRead(relay3));
      lpp.addDigitalInput(relay3,digitalRead(relay3));
      rec[2]=int(digitalRead(relay3));
    }
    else if (rec[3]!=int(digitalRead(relay4))){
      lpp.addDigitalInput(relay4,digitalRead(relay4));
      lpp.addDigitalInput(relay4,digitalRead(relay4));
      rec[3]=int(digitalRead(relay4));
    }
    else{
      lpp.addDigitalInput(0, 0);
      if (rec[4]!=hA){
        lpp.addRelativeHumidity(51,hA);
        rec[4]=hA;
      }
      if (rec[5]!=tA){
        lpp.addTemperature(52,tA);
        rec[5]=tA;
      }
      if (rec[6]!=hB){
        lpp.addRelativeHumidity(53,hB);
        rec[6]=hB;
      }
      if (rec[7]!=tB){
        lpp.addTemperature(54,tB);
        rec[7]=tB;
      }
      if (rec[8]!=tds){
        lpp.addAnalogInput(55,tds);
        rec[8]=tds;
      }
      if (rec[9]!=pH){
        lpp.addAnalogInput(57,pH);
        rec[9]=pH;
      }
      if (rec[10]!=mWh){
        lpp.addAnalogInput(58,mWh);
        rec[10]=mWh;
      }
      if (rec[11]!=loadV){
        lpp.addAnalogInput(59,loadV);
        rec[11]=loadV;
      }
    }
    
    Serial.write(lpp.getBuffer(),lpp.getSize());
    Serial.println();
    change=false;
    tstamp = now();
    delay(250);
  }
  if (Serial.available()){
    if(Serial.find(TIME_HEADER)) {
       unsigned long pctime;
       pctime = Serial.parseInt();
       setTime(pctime);
       adjustTime(28800);
    }

    else{
      byte buff[Serial.available()];
      int buffSize = Serial.readBytes(buff, Serial.available());
      
      StaticJsonDocument<512> dnLinkBuff;
      JsonArray dnLink = dnLinkBuff.to<JsonArray>();
      lpp.decode(buff, buffSize, dnLink);
  
      for (JsonVariant dataKeys : dnLink){
        int channel = dataKeys["channel"];
        if (channel==100) invalid=bool(dataKeys["value"]);
        else if (channel == 101){
          aut=bool(dataKeys["value"]);
          change=true;
        }
        else if ((channel == relay1+100) && aut==false){
          digitalWrite(relay1, bool(dataKeys["value"]));
          change=true;
        }
        else if ((channel == relay2+100) && aut==false){
          digitalWrite(relay2, bool(dataKeys["value"]));
          change=true;
        }
        else if ((channel == relay3+100) && aut==false){
          digitalWrite(relay3, bool(dataKeys["value"]));
          change=true;
        }
        else if ((channel == relay4+100) && aut==false){
          digitalWrite(relay4, bool(dataKeys["value"]));
          change=true;
        }
        else if (channel==106) onPeriod=int(dataKeys["value"]);
        else if (channel==107) offPeriod=int(dataKeys["value"]);
        else if (channel==108) hThreshold=int(dataKeys["value"]);
        else if (channel==109) tThreshold=int(dataKeys["value"]);
      }
      
      dnLinkBuff.clear();
      dnLink.clear();
      delay(250);
    }
  }
  if (invalid==true){
    lpp.reset();
    lpp.addDigitalInput(0, 0);
    lpp.addDigitalInput(relay1,rec[0]);
    lpp.addDigitalInput(relay2,rec[1]);
    lpp.addDigitalInput(relay3,rec[2]);
    lpp.addDigitalInput(relay4,rec[3]);
    lpp.addRelativeHumidity(51,rec[4]);
    lpp.addTemperature(52,rec[5]);
    lpp.addRelativeHumidity(53,rec[6]);
    lpp.addTemperature(54,rec[7]);
    lpp.addAnalogInput(55,rec[8]);
    lpp.addAnalogInput(57,rec[9]);
    lpp.addAnalogInput(58,rec[10]);
    lpp.addAnalogInput(59,rec[11]);
    
    Serial.write(lpp.getBuffer(),lpp.getSize());
    Serial.println();
    invalid=false;
    delay(250);
  }
}

void setup() {
  Serial.begin (115200);
  ina219.begin();
  lcd.begin(16, 2);
  dhtA.begin();
  dhtB.begin();
  pinMode(TdsPin,INPUT);
  pinMode(PhPin,INPUT);
  pinMode(bl,OUTPUT);
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);
  pinMode(relay3,OUTPUT);
  pinMode(relay4,OUTPUT);
  digitalWrite(bl,HIGH);
  digitalWrite(relay1,LOW);
  digitalWrite(relay2,LOW);
  digitalWrite(relay3,LOW);
  digitalWrite(relay4,LOW);
  if (timeStatus() == timeNotSet){
    setSyncProvider( requestSync);
    setSyncInterval(3600);
  }
}

void loop()
{
  kypdBtn();
  variablesManager();
  if (aut==true) pumpTimer();
  serComm();
  
// Update LCD Display
// Print TIME in Hour, Min, Sec + AM/PM
  lcd.setCursor(0,0);
  lcd.print("Time ");
  if(hour()>12)h=hour()-12;
  else h=hour();
  if(h<10)lcd.print("0");// always 2 digits
  lcd.print(h);
  lcd.print(":");
  if(minute()<10)lcd.print("0");
  lcd.print(minute());
  lcd.print(":");
  if(second()<10)lcd.print("0");
  lcd.print(second());

  if(isAM()==true) lcd.print(" AM");
  else lcd.print(" PM");

  if(line==1){
    lcd.setCursor(0,1);// for Line 2
    if(aut==true){
      if(!digitalRead(relay1)){
        lcd.print("AutoPump:OFF ~");
        nextSwitch=(-1)*(pumpElapse-offPeriod);
      }
      else if(digitalRead(relay1)){
        lcd.print("AutoPump:ON  ~");
        nextSwitch=(-1)*(pumpElapse-onPeriod);
      }
      if(timer==true || (pumpElapse!=0 && digitalRead(relay1)==true)){
        if(nextSwitch<10)lcd.print("0");
        lcd.print(nextSwitch);
      }
      else if(timer==false && digitalRead(relay1)==false){
        lcd.print("--");
      }
    }
    else if(aut==false){
      if(!digitalRead(relay1)){
        lcd.print("Pump:OFF     ~--");
      }
      else if(digitalRead(relay1)){
        lcd.print("Pump:ON      ~--");
      }
    }
  }
  else if(line==2){
    lcd.setCursor(0,1);// for Line 2
    lcd.print("hA:");
    if(hA<10)lcd.print("0");
    lcd.print(int(hA));
    lcd.print("% tA:");
    if(tA<10)lcd.print("0");
    lcd.print(int(tA));
    lcd.print((char)223);
    lcd.print("C     ");
  }
  else if(line==3){
    lcd.setCursor(0,1);// for Line 2
    lcd.print("hB:");
    if(hB<10)lcd.print("0");
    lcd.print(int(hB));
    lcd.print("% tB:");
    if(tB<10)lcd.print("0");
    lcd.print(int(tB));
    lcd.print((char)223);
    lcd.print("C     ");
  }
  else if(line==4){
    lcd.setCursor(0,1);// for Line 2
    lcd.print("tds:");
    if(tds<10)lcd.print("0");
    lcd.print(tds);
    lcd.print(" ppm    ");
  }
  else if(line==5){
    lcd.setCursor(0,1);// for Line 2
    lcd.print("ec:");
    if(ec<10)lcd.print("0");
    lcd.print(ec);
    lcd.print(" us/cm   ");
  }
  else if(line==6){
    lcd.setCursor(0,1);// for Line 2
    lcd.print("ph:");
    if(pH<10)lcd.print("0");
    lcd.print(pH);
    lcd.print("          ");
  }
  else if(line==7){
    lcd.setCursor(0,1);// for Line 2
    lcd.print("Power:");
    if(mWh<10)lcd.print("0");
    if(mWh<100){
      lcd.print(mWh);
      lcd.print("mW/h     ");
    }
    else{
      lcd.print(mWh/1000);
      lcd.print("W/h     ");
    }
  }
  else if(line==8){
    lcd.setCursor(0,1);// for Line 2
    lcd.print("Voltage:");
    if(loadV<10)lcd.print("0");
    lcd.print(loadV);
    lcd.print("V   ");
  }
// Loop end
}
