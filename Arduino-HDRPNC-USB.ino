#include <Adafruit_INA219.h>
#include <LiquidCrystal.h>
#include <CayenneLPP.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <Wire.h>
#include "DHT.h"

Adafruit_INA219 ina219;
OneWire  ds(19);

template <class T, size_t N> constexpr size_t len(const T(&)[N]) { return N; }

// This defines the LCD wiring to the DIGITALpins
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize time related variables
unsigned long ts = 0UL, tm = 0UL, tstamp = 0UL, interval = 900UL;

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
float averageVoltage = 0, tds=0, ec=0;

// PH variables
#define PhPin A9
float calibration_value = 21.34 - 0.7;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;

// INA219 variables
uint32_t currentFrequency;
float PowSec[60], PowMin[60], PowSecAve=0, PowMinAve=0, mWh=0, Wh=0, kWh=0, loadV=0;
int PowSecInd=0, PowMinInd=0;

//init somemore variables
const int relay1=24, relay2=25, relay3=26, relay4=27, waterLvl=18;
bool aut=true, change=false, invalid=false;
int h=0, manual=0, pumpElapse=0, fertA_Elapse=0, fertB_Elapse=0, fertC_Elapse=0;
int tThreshold=0, hThreshold=0, TDSMaxOffset=0, TDSMinOffset=0, pumpIdle=0, line=1, idl=0;
float hA, hB, tA, tB, pH=0, rec[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
DHT dhtA(DHTA, DHTTYPE);
DHT dhtB(DHTB, DHTTYPE);

// read the buttons
int read_LCD_buttons(){
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
  float waterTemp = WaterTemperature();
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
      float compensationCoefficient=1.0+0.02*(waterTemp-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
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

float WaterTemperature(){
  byte i;
  byte present = 0;
  byte type_s;
  byte data[9];
  byte addr[8];
  
  if (ds.search(addr)) {
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);

    delay(1000);

    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }

    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    float waterTemp = (float)raw / 16.0;
    return waterTemp;
  }
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
}

void pumpControl(){
  if (fertA_Elapse>0){
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, HIGH);
    digitalWrite(relay3, LOW);
    digitalWrite(relay4, LOW);
  }
  else if (fertB_Elapse>0){
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);
    digitalWrite(relay3, HIGH);
    digitalWrite(relay4, LOW);
  }
  else if (fertC_Elapse>0){
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);
    digitalWrite(relay3, LOW);
    digitalWrite(relay4, HIGH);
  }
  else if (pumpElapse>0){
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, LOW);
    digitalWrite(relay3, LOW);
    digitalWrite(relay4, LOW);
    pumpIdle=0;
  }
  
  if (pumpElapse<=0) digitalWrite(relay1, LOW);
  if (fertC_Elapse<=0) digitalWrite(relay4, LOW);
  if (fertB_Elapse<=0) digitalWrite(relay3, LOW);
  if (fertA_Elapse<=0) digitalWrite(relay2, LOW);

  if (rec[1]!=int(digitalRead(relay1)) || rec[2]!=int(digitalRead(relay2)) || rec[3]!=int(digitalRead(relay3)) || rec[4]!=int(digitalRead(relay4))) change = true;
}

void variablesManager(){
  if (now() >= ts+1){
    idl++;
    if (!digitalRead(relay1)){
      if (pumpIdle>=7200) pumpElapse=300;
      else pumpIdle++;
    }
    
    if (fertA_Elapse>0) fertA_Elapse--;
    else if (fertB_Elapse>0) fertB_Elapse--;
    else if (fertC_Elapse>0) fertC_Elapse--;
    else if (pumpElapse>0) pumpElapse--;
    
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
    ts = now();
  }
  if (now() >= tm+60){
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
      if(mWh > 100){
        Wh = mWh / 1000;
        mWh = 0;
      }
      if(Wh > 100){
        kWh = Wh / 1000;
        Wh = 0;
      }
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
    line=1;
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
  if(lcd_key==btnRIGHT){
    aut=!aut;
    if(aut==true) manual=0;
    delay(200);
  } 
  if(lcd_key==btnLEFT){
    if(aut==false) manual++;
    delay(200);
    if (manual>4 || aut==true) manual=0;
    if (manual==0){
      pumpElapse=0;
      fertA_Elapse=0;
      fertB_Elapse=0;
      fertC_Elapse=0;
    }
    else if (manual==1){
      pumpElapse=300;
      fertA_Elapse=0;
      fertB_Elapse=0;
      fertC_Elapse=0;
    }
    else if (manual==2){
      pumpElapse=0;
      fertA_Elapse=20;
      fertB_Elapse=0;
      fertC_Elapse=0;
    }
    else if (manual==3){
      pumpElapse=0;
      fertA_Elapse=0;
      fertB_Elapse=20;
      fertC_Elapse=0;
    }
    else if (manual==4){
      pumpElapse=0;
      fertA_Elapse=0;
      fertB_Elapse=0;
      fertC_Elapse=20;
    }
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
  while (!Serial.available());
  if(Serial.find(TIME_HEADER)) {
    unsigned long pctime;
    pctime = Serial.parseInt();
    setTime(pctime);
    adjustTime(28800);
    
//    Serial.print("mssg");
//    Serial.print("Time: ");
//    Serial.print(pctime);
//    Serial.println();
  }
}

void serComm(){
  CayenneLPP lpp (160);
  lpp.reset();
    
  if (change==true){
    lpp.reset();
    lpp.addDigitalInput(0, 0);
    lpp.addDigitalInput(58,digitalRead(waterLvl));
    if (rec[0]!=int(aut)){
      lpp.addDigitalInput(1,aut);
      rec[0]=int(aut);
    }
    if (rec[1]!=int(digitalRead(relay1))){
      lpp.addDigitalInput(relay1,digitalRead(relay1));
      rec[1]=int(digitalRead(relay1));
    }
    if (rec[2]!=int(digitalRead(relay2))){
      lpp.addDigitalInput(relay2,digitalRead(relay2));
      rec[2]=int(digitalRead(relay2));
    }
    if (rec[3]!=int(digitalRead(relay3))){
      lpp.addDigitalInput(relay3,digitalRead(relay3));
      rec[3]=int(digitalRead(relay3));
    }
    if (rec[4]!=int(digitalRead(relay4))){
      lpp.addDigitalInput(relay4,digitalRead(relay4));
      rec[4]=int(digitalRead(relay4));
    }
    if (rec[5]!=hA){
      lpp.addRelativeHumidity(51,hA);
      rec[5]=hA;
    }
    if (rec[6]!=tA){
      lpp.addTemperature(52,tA);
      rec[6]=tA;
    }
    if (rec[7]!=hB){
      lpp.addRelativeHumidity(53,hB);
      rec[7]=hB;
    }
    if (rec[8]!=tB){
      lpp.addTemperature(54,tB);
      rec[8]=tB;
    }
    if (rec[9]!=tds){
      lpp.addAnalogInput(55,tds);
      rec[9]=tds;
    }
    if (rec[10]!=pH){
      lpp.addAnalogInput(57,pH);
      rec[10]=pH;
    }
    if (rec[11]!=loadV){
      lpp.addAnalogInput(59,loadV);
      rec[11]=loadV;
    }
    if (rec[12]!=mWh){
      lpp.addAnalogInput(60,mWh);
      rec[12]=mWh;
    }
    if (rec[13]!=Wh){
      lpp.addAnalogInput(61,Wh);
      rec[13]=Wh;
    }
    if (rec[14]!=kWh){
      lpp.addAnalogInput(62,kWh);
      rec[14]=kWh;
    }
    
    Serial.write(lpp.getBuffer(),lpp.getSize());
    Serial.println();
    change=false;
    tstamp = now();
    delay(250);
  }
  if (Serial.available()){
    if (Serial.find("D")){
      byte buff[Serial.available()];
      int buffSize = Serial.readBytes(buff, Serial.available());
      
//      Serial.print("mssg");
//      Serial.print("Data Received: ");
//      for (unsigned int i = 0; i < buffSize; i++){
//        Serial.print("0x");
//        Serial.print(buff[i], HEX);
//        Serial.print(" ");
//      }
//      Serial.println();
      
      byte dataBuff[buffSize-1];
      int dataSize=buffSize-1;
      for (unsigned int i = 1; i < buffSize; i++){
        dataBuff[i-1] = buff[i];
      }
    
      DynamicJsonDocument dnLinkBuff(4096);
      JsonArray dnLink = dnLinkBuff.to<JsonArray>();
      lpp.decode(buff, buffSize, dnLink);

//      Serial.print("mssg");
//      Serial.print("dnLink: ");
//      serializeJson(dnLink, Serial);
//      Serial.println();
    
      for (JsonVariant dataKeys : dnLink){
        int channel = dataKeys["channel"];
        if (channel==100) invalid=bool(dataKeys["value"]);
        else if (channel == 101){
          aut=bool(dataKeys["value"]);
          change=true;
        }
        else if (channel == relay1+100 && bool(dataKeys["value"])==true){
          manual=1;
          pumpElapse=90;
          change=true;
        }
        else if (channel == relay2+100 && bool(dataKeys["value"])==true){
          manual=2;
          fertA_Elapse=20;
          change=true;
        }
        else if (channel == relay3+100 && bool(dataKeys["value"])==true){
          manual=3;
          fertB_Elapse=20;
          change=true;
        }
        else if (channel == relay4+100 && bool(dataKeys["value"])==true){
          manual=4;
          fertC_Elapse=20;
          change=true;
        }
        else if (channel==106){
          pumpElapse=int(dataKeys["value"]);
          change=true;
        }
        else if (channel==107){
          fertA_Elapse=int(dataKeys["value"]);
          change=true;
        }
        else if (channel==108){
          fertB_Elapse=int(dataKeys["value"]);
          change=true;
        }
        else if (channel==109){
          fertC_Elapse=int(dataKeys["value"]);
          change=true;
        }
      }
      
//      Serial.print("mssg");
//      Serial.print("pumpElapse: ");
//      Serial.print(pumpElapse);
//      Serial.print("  fertElapse: ");
//      Serial.print(fertElapse);
//      Serial.println();
      
      dnLinkBuff.clear();
      dnLink.clear();
      delay(250);
    }
  }
  if (invalid==true){
    lpp.reset();
    lpp.addDigitalInput(0, 0);
    lpp.addDigitalInput(1, rec[0]);
    lpp.addDigitalInput(relay1,rec[1]);
    lpp.addDigitalInput(relay2,rec[2]);
    lpp.addDigitalInput(relay3,rec[3]);
    lpp.addDigitalInput(relay4,rec[4]);
    lpp.addRelativeHumidity(51,rec[5]);
    lpp.addTemperature(52,rec[6]);
    lpp.addRelativeHumidity(53,rec[7]);
    lpp.addTemperature(54,rec[8]);
    lpp.addAnalogInput(55,rec[9]);
    lpp.addAnalogInput(57,rec[10]);
    lpp.addDigitalInput(58,digitalRead(waterLvl));
    lpp.addAnalogInput(59,rec[11]);
    lpp.addAnalogInput(60,rec[12]);
    lpp.addAnalogInput(61,rec[13]);
    lpp.addAnalogInput(62,rec[14]);
    
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
  pinMode(waterLvl, INPUT);
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
  pumpControl();
  variablesManager();
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
    if(!digitalRead(relay1) && !digitalRead(relay2) && !digitalRead(relay3) && !digitalRead(relay4)){
      lcd.print("PumpOff Auto:");
      if (aut==true) lcd.print("ON~");
      else lcd.print("OFF");
    }
    else if(digitalRead(relay1)){
      lcd.print("MainPump: ON~");
      if(pumpElapse<100)lcd.print("0");
      if(pumpElapse<10)lcd.print("0");
      lcd.print(pumpElapse);
    }
    else if(digitalRead(relay2) && !digitalRead(relay3) && !digitalRead(relay4)){
      lcd.print("Fert A: ON   ~");
      if(fertA_Elapse<10)lcd.print("0");
      lcd.print(fertA_Elapse);
    }
    else if(!digitalRead(relay2) && digitalRead(relay3) && !digitalRead(relay4)){
      lcd.print("Fert B: ON   ~");
      if(fertB_Elapse<10)lcd.print("0");
      lcd.print(fertB_Elapse);
    }
    else if(!digitalRead(relay2) && !digitalRead(relay3) && digitalRead(relay4)){
      lcd.print("Fert C: ON   ~");
      if(fertC_Elapse<10)lcd.print("0");
      lcd.print(fertC_Elapse);
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
    else if(mWh>100){
      lcd.print(Wh);
      lcd.print("W/h     ");
    }
    else if(Wh>100){
      lcd.print(kWh);
      lcd.print("W/h      ");
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
