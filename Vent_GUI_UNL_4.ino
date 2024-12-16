#include <Arduino.h>
#include <UTFT.h>
#include <URTouch.h>
#include <UTFT_Buttons.h>
#include "compute.h"

const uint8_t ADC_sequencer_size = 4; // MUST match the number of input in ADC_Sample->begin()
uint32_t adcBuffer[ADC_sequencer_size];

int count = 0;
#define sampleSize 100
#define conv 300.0


//Pins
#define STOP_SWITCH1      14
#define STOP_SWITCH2      15
#define FLOW_PIN          A2
#define KE_25_PIN         A0
#define CO2_PIN           A6
#define MPXHZ_PIN         A1
#define IN1_PIN           9
#define IN2_PIN           8
#define EN_PIN            10
#define BUZ_PIN           11

//ADXL Pins
int RawMin = 0;
int RawMax = 4096;
const int xInput = A3;
const int yInput = A4;
const int zInput = A5;

int xRaw, yRaw, zRaw;
float xAccel, yAccel, zAccel, volt; 
bool calibrated = false;
volatile bool enabled = false;
float hyteresis = 0.0, Rmax = 0.0, Rmin = 6000.0, Rref = 0.0, Rn = 0.0;
bool counted = true;
uint32_t bmpTime[5];
uint32_t beatTimer = 0, kount = 0, Counted = 0;

extern uint8_t BigFont[];
extern uint8_t SmallFont[];
extern uint8_t SevenSegNumFont[];

extern unsigned short plug[0x528];
extern unsigned short lung[0x3B10];
extern unsigned short battery[0x4D8];

#define MAXSAMPLE     100

#define LH_BOX_X1     10
#define LH_BOX_X2     160

#define FLOW_MIN      0.68          //0.5
#define FLOW_MAX      4.32          //4.5


UTFT           myGLCD(SSD1963_800ALT,38,39,40,41);
URTouch        myTouch(6,5,4,3,2);
UTFT_Buttons   myButtons(&myGLCD, &myTouch);

//Button Returns
int pressed_button, BPM_hi, BPM_lw, IE_hi, IE_lw, weight_hi, weight_lw, MotorEN, Modes, Events, NextPage;
int BPM_val = 12,     IE_val = 1,         weight_val = 15;

//Read Variable val
int peepVal = 0,      peepVal_lw = 0,     peepVal_hi = 0;
int flowVal = 0,      flowVal_lw = 0,     flowVal_hi = 0;
int bpmVal = 0,       bpmVal_lw = 0,      bpmVal_hi = 0;
int co2Val = 0,       co2Val_lw = 0,      co2Val_hi = 0;
int o2Val = 0,        o2Val_lw = 0,       o2Val_hi = 0;

int               motorspeed_adc                  =         0;
bool              adjusted                        =         true;
volatile int      pos                             =         0;
volatile uint32_t pollTimer                       =         0;
volatile uint32_t waitTime                        =         500;

#define Times  750 //1500 is max for compresion


volatile uint32_t timers[]                         =        {Times, Times}; //{90, 100, 1000, 100};

int pin1[] = {HIGH, LOW};
int pin2[] = {LOW, HIGH};

//int pin1[] = {HIGH, HIGH, LOW, HIGH};
//int pin2[] = {LOW, HIGH, HIGH, HIGH};

void TC3_Handler(void)
{
  TC_GetStatus(TC1, 0);
  motorHandler();
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK/128/frequency;
  TC_SetRA(tc, channel, rc/2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}


void handleDrawStatic()
{
  myGLCD.setBackColor(0x0000);

  myGLCD.setColor(0x0861);
  myGLCD.fillRoundRect(180, 50, 580, 260);
  myGLCD.fillRoundRect(600, 50, 790, 380);
  
  int j = 50;
  for(int i = 0; i < 4; i++, j += 110)
    myGLCD.fillRoundRect(LH_BOX_X1, j, LH_BOX_X2, (j + 99));
    
  myGLCD.fillRoundRect(180, 380, 330, 479); //290, 

  //Buttons
  myButtons.setTextFont(BigFont);
  myButtons.setButtonColors(VGA_WHITE, VGA_GRAY, 0x0861, VGA_RED, 0x016A);
  BPM_hi        = myButtons.addButton(620,  100, 40,  40, "+");
  BPM_lw        = myButtons.addButton(730,  100, 40,  40, "-");
  weight_hi     = myButtons.addButton(620,  200, 40,  40, "+");
  weight_lw     = myButtons.addButton(730,  200, 40,  40, "-");
  IE_hi         = myButtons.addButton(620,  300, 40,  40, "+");
  IE_lw         = myButtons.addButton(730,  300, 40,  40, "-");
  
  myButtons.setTextFont(SmallFont);
  MotorEN       = myButtons.addButton(600,  390, 90,  30, "Disabled");
  Modes         = myButtons.addButton(700,  390, 85,  30, "Modes");
  Events        = myButtons.addButton(600,  430, 90,  30, "Events");
  NextPage      = myButtons.addButton(700,  430, 85,  30, "Next Page");
  
  myButtons.drawButtons();

  //Bitmap logos
  myGLCD.drawBitmap(420, 300, 169, 160, lung);
  myGLCD.drawBitmap(720, 5, 40, 31, battery);
  //myGLCD.drawBitmap(759, 5, 33, 40, plug);

  //Print Text
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(VGA_WHITE);
  myGLCD.setBackColor(0x0861);
  myGLCD.print("07:58 PM", 15, 15);
  myGLCD.print("78%", 700, 15);
  myGLCD.print("BPM", 625, 75);
  myGLCD.print("WEIGTH", 625, 175);
  myGLCD.print("IE", 625, 275);
  myGLCD.print("bpm", 670, 125);
  myGLCD.print("kg", 675, 225);
  myGLCD.print("ratio", 690, 325);
  
  myGLCD.print("PEEP", 20, 50);
  myGLCD.print("FLOW", 20, 164);
  myGLCD.print("BREATH", 20, 275);
  myGLCD.print("FiO2", 20, 389);
  myGLCD.print("CO2", 190, 389);
  
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(0x528A);
  myGLCD.setBackColor(0x0861);
  myGLCD.print("cmH20", 100, 130);
  myGLCD.print("m3/min", 100, 239);
  myGLCD.print("BPM", 100, 348);
  myGLCD.print("%", 100, 457);
  myGLCD.print("PPM", 260, 457);
  /*
  myGLCD.print("25", 20, 110);
  myGLCD.print("18", 20, 130);
  myGLCD.print("21", 20, 239);
  myGLCD.print("15", 20, 219);
  myGLCD.print("12", 20, 348);
  myGLCD.print("10", 20, 328);
  myGLCD.print("3", 20, 437);
  myGLCD.print("0", 20, 457);
  */

  myGLCD.print("20", 200, 90);
  myGLCD.print("15", 200, 130);
  myGLCD.print("10", 200, 170);
  myGLCD.print("05", 200, 210);

  myGLCD.print("25", 540, 240);
  myGLCD.print("20", 460, 240);
  myGLCD.print("15", 380, 240);
  myGLCD.print("10", 300, 240);
  myGLCD.print("05", 220, 240);

  for(int y = 80; y < 240; y += 20)
  {
    for(int x = 220; x < 560; x += 10 ) //200, 100
      myGLCD.drawPixel(x, y);
  }

  for(int x = 220; x < 560; x += 20)
  {
    for(int y = 80; y < 240; y += 10 ) //200, 100
      myGLCD.drawPixel(x, y);
  }  
/*
  myGLCD.setFont(BigFont);
  myGLCD.setColor(VGA_BLUE); 
  myGLCD.setBackColor(0x0861); 
  //myGLCD.print("55", 675, 105);
  myGLCD.printNumI(speed_val, 670, 105, 3, '0');
  myGLCD.print("14", 675, 205);
  myGLCD.print("21", 675, 305);

  myGLCD.setFont(SevenSegNumFont);
  myGLCD.setColor(VGA_WHITE);
  myGLCD.setBackColor(0x0861);
  myGLCD.print("22", 85, 60);
  myGLCD.print("19", 85, 169);
  myGLCD.print("12", 85, 278);
  myGLCD.print("1", 85, 387);
*/
  
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(VGA_RED);
  myGLCD.setBackColor(0x0000);
  myGLCD.print("PATIENT STATS", 180, 280);

  myGLCD.setFont(SmallFont);
  myGLCD.setColor(VGA_WHITE);
  myGLCD.setBackColor(0x0000);
  myGLCD.print("IE ratio: ", 180, 300);
  myGLCD.print("IN time(s): ", 180, 320);
  myGLCD.print("EX time(s): ", 180, 340);
  myGLCD.print("Health: ", 180, 360);

  myGLCD.print("STABLE ", 310, 360);  
}

void handleDrawVariables()
{
  myGLCD.setFont(BigFont);
  myGLCD.setColor(VGA_BLUE); 
  myGLCD.setBackColor(0x0861); 
  myGLCD.printNumI(BPM_val, 660, 105, 4, '0');
  myGLCD.printNumI(weight_val, 660, 205, 4, '0');
  myGLCD.printNumI(IE_val, 670, 305, 3, '0');  

  myGLCD.setFont(SmallFont);
  myGLCD.setColor(0x528A);
  myGLCD.setBackColor(0x0861);
  myGLCD.printNumI(peepVal_hi, 20, 110, 2, '0');  //PEEP
  myGLCD.printNumI(peepVal_lw, 20, 130, 2, '0');
  myGLCD.printNumI(flowVal_hi, 20, 239, 2, '0');  //FLOW
  myGLCD.printNumI(flowVal_lw, 20, 219, 2, '0');
  myGLCD.printNumI(bpmVal_hi, 20, 348, 2, '0');   //BPM
  myGLCD.printNumI(bpmVal_lw, 20, 328, 2, '0');
  myGLCD.printNumI(o2Val_hi, 20, 437, 2, '0');   //O2
  myGLCD.printNumI(o2Val_lw, 20, 457, 2, '0');
  myGLCD.printNumI(co2Val_hi, 190, 437, 2, '0');   //O2
  myGLCD.printNumI(co2Val_lw, 190, 457, 2, '0');

  myGLCD.setFont(SevenSegNumFont);
  myGLCD.setColor(VGA_WHITE);
  myGLCD.setBackColor(0x0861);
  myGLCD.printNumI(peepVal, 60, 65, 3, '0');      //PEEP
  myGLCD.printNumI(flowVal, 60, 175, 3, '0');     //FLOW
  myGLCD.printNumI(bpmVal, 60, 283, 3, '0');      //BPM
  myGLCD.printNumI(o2Val, 60, 392, 3, '0');       //O2
  myGLCD.printNumI(co2Val, 230, 392, 3, '0');     //CO2

  myGLCD.setFont(SmallFont);
  myGLCD.setColor(VGA_WHITE);
  myGLCD.setBackColor(0x0000);
  myGLCD.printNumI(IE_val, 310, 300,  2, '0');
  myGLCD.printNumF(timers[1]/1000., 2, 310, 320, '.', 2);
  myGLCD.printNumF(timers[0]/1000., 2, 310, 340, '.', 2);
  
}

void setup() 
{
  Serial.begin(115200);
  pinMode(FLOW_PIN,     INPUT);
  pinMode(BUZ_PIN,      INPUT);
  pinMode(KE_25_PIN,    INPUT);
  pinMode(IN1_PIN,      OUTPUT);
  pinMode(IN2_PIN,      OUTPUT);
  pinMode(EN_PIN,       OUTPUT);
  pinMode(xInput,       INPUT);
  pinMode(yInput,       INPUT);
  pinMode(zInput,       INPUT);
  pinMode(STOP_SWITCH1, INPUT_PULLUP);
  pinMode(STOP_SWITCH2, INPUT_PULLUP);
    
  analogReadResolution(12);
  analogWriteResolution(10);
  
  myGLCD.InitLCD();
  myTouch.InitTouch(LANDSCAPE);
  myTouch.setPrecision(PREC_MEDIUM);
  myGLCD.clrScr();
  handleDrawStatic();
  
  Serial.println("setting ARM");
 // HandleRestoreArm();

  //speed_val = 0;
  startTimer(TC1, 0, TC3_IRQn, 200);
  //Serial.println("ARM Aligned!");
}

void loop() 
{
  handleScreenReact();
  handleDrawVariables();
  handleMotorSpeed();
  //handleAllReadings();
  handleFlowRateReading();
  handleBPMCount();
  handleGetPressure();
  handleO2Reading();
  delay(50);
}


void handleScreenReact()
{
  if(myTouch.dataAvailable() == true)
  {
    pressed_button = myButtons.checkButtons();

    if(pressed_button == BPM_hi)
    {
      if(BPM_val < 40)
      BPM_val ++;
      adjusted = true;
    }
    else if(pressed_button == BPM_lw)
    {
      if(BPM_val > 12)
      BPM_val --;
      adjusted = true;
    }
    else if(pressed_button == IE_hi)
    {
      if(IE_val < 4)
      IE_val++;
      adjusted = true;
    }
    else if(pressed_button == IE_lw)
    {
      if(IE_val > 1)
      IE_val--;
      adjusted = true;
    }
    else if(pressed_button == weight_hi)
    {
      if(weight_val < 75)
      weight_val++;
      adjusted = true;
    }
    else if(pressed_button == weight_lw)
    {
      if(weight_val > 15)
      weight_val --;
      adjusted = true;
    }
    else if(pressed_button == MotorEN)
    {
      if(enabled)
      {
        digitalWriteDirect(IN1_PIN, 0);
        digitalWriteDirect(IN2_PIN, 0); 
        myButtons.relabelButton(MotorEN, "Disabled", true);
        enabled = false;
        HandleRestoreArm();
      }
      else
      {
        myButtons.relabelButton(MotorEN, "Enabled", true);
        enabled = true;
      }
      
      if(calibrated) calibrated = false;
      
    }
    else if(pressed_button == Modes)
    {
      
    }
    else if(pressed_button == Events)
    {
      
    }
    else if(pressed_button == NextPage)
    {
      
    }
  }
}

void handleMotorSpeed()
{
  if(adjusted)
  {//1:IE
    //float motorspeed_Volt = (0.0000005 * weight_val * (1 + IE_val)) / (1 - pow(e, -(0.7950/((1+IE_val)*BPM_val))));
    float mpp1 = (Bm*Ra) + pow(k, 2);
    float mpp2 = Ra*Jm;
    float LA = arm_len(R);
    float DV = delta_vol(R, L, LA); 
    float theta = p_vol(weight_val)/DV;
    float TAU = tau(BPM_val, IE_val);
    float omega = ((1 + IE_val) * BPM_val * theta)/60.0;
    float v = Volt(mpp1, mpp2, omega, TAU);

    timers[0] = 0;
    timers[1] = 0;
    timers[1] = round(TAU * 1000);                      //Inhalation time
    timers[0] = round(IE_val * timers[1]);              //Exhalation Time in sec
    motorspeed_adc = mapfloat(v, 0.373197, 15.5499, 400.0, 4095.0);
    //motorspeed_adc = map((v*1000), 373, 15549, 400, 4095);
    
    Serial.print(F("volt: "));
    Serial.print(v, 3);
    Serial.print(F("V, "));
    Serial.print(F("motorspeed_adc: "));
    Serial.println(motorspeed_adc);
}
  
  if(enabled)
  {    
    analogWrite(EN_PIN, motorspeed_adc); //digitalWrite(EN_PIN, !digitalRead(EN_PIN));
    if(adjusted)
    adjusted = false;
  }
  else
    analogWrite(EN_PIN, 0);

}


void motorHandler()
{
 /* 
  if(speed_val < 12)
  {
    digitalWriteDirect(IN1_PIN, 0);
    digitalWriteDirect(IN2_PIN, 0);
    return;
  }
  */

  if((digitalReadDirect(STOP_SWITCH1) == LOW || digitalReadDirect(STOP_SWITCH2) == LOW) && pos == 0)      //if a extreme end
  {
    digitalWriteDirect(IN1_PIN, 0);
    digitalWriteDirect(IN2_PIN, 0);    
    //pollTimer = 0;
  }
  else
  {
    if(enabled)
    {
      digitalWriteDirect(IN1_PIN, pin1[pos]);
      digitalWriteDirect(IN2_PIN, pin2[pos]);
    }
  }

  if(millis() - pollTimer >= timers[pos])
  {
    pollTimer = millis();
    pos++;

    if(pos > 1)
    {
      pos = 0;
    }
  }
}

inline void digitalWriteDirect(int pin, boolean val)
{
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadDirect(int pin)
{
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void handleFlowRateReading()
{
  static uint8_t p = 0;
  static uint16_t tempBuffer[6];
  uint32_t val = 0;  
  
  for(int i = 0; i < 500; i++)
  {
    val += analogRead(FLOW_PIN);
    delayMicroseconds(50);
  }

  val = val/500;
  
  //float volt = 3.3 * adcBuffer[0]/4095.;
  float volt = 3.3 * val/4095.;
  int flow = round(mapfloat(volt, FLOW_MIN, FLOW_MAX, 0.0, 200.0));  

  //Serial.print("volt: ");
  //Serial.println(volt);
  
  //Serial.print("flow: ");
  //Serial.println(flow);
  if(flow >= 0)
  flowVal = flow;
  //tempBuffer[p++] = flow;
/*
  //where o2Val is computed
  if(p == 5)
  {
    uint32_t ttAvg = 0;
    for(int t = 0; t < p; t++)
    {
      ttAvg += tempBuffer[t];
    }
    
    flowVal = ttAvg / 5;
    
    //shift bmpTime[] << 1
    p = shiftArrayLeft((float*)tempBuffer, 5, 1);

    //Serial.print("flowVal: ");
    //Serial.println(flowVal);
  }
  */
}

void handleO2Reading()
{
  static uint8_t p = 0;
  static uint16_t tempBuffer[6];
  
  uint32_t sensorValue = 0;
  for(int i = 0; i < 500; i++)
  {
    sensorValue += analogRead(KE_25_PIN);
    delayMicroseconds(50);
  }

  sensorValue = sensorValue/500;
  //tempBuffer[p++] = round(sensorValue * 0.14583);

  int o2Val_f = round(sensorValue * 0.14583);
  o2Val = constrain(o2Val_f, 0, 100);
/*
  //where o2Val is computed
  if(p == 5)
  {
  Serial.print("tempBuffer[");
  Serial.print(p-2);
  Serial.print("]: ");
  Serial.println(tempBuffer[p]);
    uint32_t ttAvg = 0;
    for(int t = 0; t < p; t++)
    {
      ttAvg += tempBuffer[t];
    }
    
    o2Val = ttAvg / 5;
    
    //shift bmpTime[] << 1
    p = shiftArrayLeft((float*)tempBuffer, 5, 1);
  }
  */
    //Serial.print("o2Val: ");
    //Serial.println(o2Val);
}


void handleGetPressure()
{
  //static uint32_t lastTimer = millis();

  //if(millis() >= (lastTimer + 1000))
  //{
    int16_t adc0;
    float v0, pres = 0.0;
    static float Ful_Pres[5] = {0.0}, temp = 0.0, atm_Pressure = 0;
    static uint16_t p = 0, q = 0;
    
    uint32_t sensorValue = 0;
    for(int i = 0; i < 500; i++)
    {
      sensorValue += analogRead(MPXHZ_PIN);
      delayMicroseconds(50);
    }
  
    sensorValue = sensorValue/500;
    v0 = (3.3/4095.*sensorValue) - 0.0115; //0.0115 offset voltage
  
    //float Perr = 0.069/(temp*0.002421*5.0);
    float Perr = 0.069/(27.7*0.002421*5.0);
    pres = ((v0/5.0) + 0.00842)/0.002421; 
    pres = pres - Perr;

    if(atm_Pressure == 0)
    atm_Pressure = pres;
 /*   
    Serial.print("sensorValue: ");
    Serial.println(sensorValue);
    Serial.print("v0: ");
    Serial.println(v0);
    Serial.print("pres: ");
    Serial.println(pres);
    */
    pres = pres - atm_Pressure;

    if(pres < 0.1) pres = 0.0;
    else pres = pres*10.197;

    //Ful_Pres[p++] = pres;
    peepVal = pres;
/*
    if(p == 5)
    {
      double ttotal = 0.0;

      for(int t = 0; t < 5; t++)
      {
        ttotal += Ful_Pres[t];
      }

      peepVal = ttotal/5.;
      
      p = shiftArrayLeft(Ful_Pres, 5, 1);
  
      Serial.print("peepVal: ");
      Serial.println(peepVal);
    }
    */
}


void handleCO2Reading()
{
  uint32_t sensorValue = 0;
  for(int i = 0; i < 500; i++)
  {
    sensorValue += analogRead(CO2_PIN);
    delayMicroseconds(50);
  }

  sensorValue = sensorValue/500;
  float current = 4000/204. * sensorValue;

  float tem = mapfloat(current, 4000, 20000, 0.0, 2000.0);
  
  if(tem > 0)
    co2Val = tem;
    
}


void handleBPMCount()
{
  static uint8_t p = 0;
  xRaw = ReadAxis(xInput);
  yRaw = ReadAxis(yInput);
  zRaw = ReadAxis(zInput);

  // Convert raw values to 'milli-Gs"
  xAccel = mapfloat((float)xRaw, RawMin, RawMax, -1650, 1650.) / conv;  
  yAccel = mapfloat((float)yRaw, RawMin, RawMax, -1650, 1650.) / conv;
  zAccel = mapfloat((float)zRaw, RawMin, RawMax, -1650, 1650.) / conv;

  Rn = compute_R(xAccel, yAccel, zAccel);
  //Serial.print(F("Rn: "));
  //Serial.println(Rn, 5);
  //counting process
  if(calibrated)
  {
    if(Rn <= Rref-hyteresis)
    {
      if(!counted)
      {
        kount++; 
        counted = true;
        bmpTime[p++] = millis() - beatTimer;
        beatTimer = millis();
        //delay(50);
      }
    }
    else if(Rn >= Rref+hyteresis)
    {
      if(counted)
      {
        counted = false;   
        //delay(250);       
      }
    }
  
    //where bpmVal is computed
    if(p == 5)
    {
      float ttAvg = 0;
      for(int t = 0; t < p; t++)
      {
        ttAvg += bmpTime[t];
      }
      ttAvg = ttAvg / 1000.;
      //Serial.print("ttAvg: ");
      //Serial.println(ttAvg);
      
      //shift bmpTime[] << 1
      p = shiftArrayLeft((float*)bmpTime, 5, 1);
      //Serial.print("new array num: ");
      //Serial.println(p);

      bpmVal = 60 * 5/ttAvg;
      //Serial.print("bpmVal: ");
      //Serial.println(bpmVal);
    }
  }

  getCalibration();
}

uint16_t shiftArrayLeft(float *_data, uint16_t _len, uint8_t pos)
{
  float temp = 0;
  for(int j = pos; j <_len; j++)
  {
    temp = _data[j];
    _data[j-pos] = temp;
  }
  return (_len-pos);
}

void getCalibration()
{
  static float cal_data[20] = {0.0};
  static int j = 0;

  if(!calibrated  && /*speed_val > 11*/ enabled)
  {
    cal_data[j++] = Rn; //compute_R(xAccel, yAccel, zAccel);
    
    //Serial.print(j);
    //Serial.print(F(": "));
    //Serial.println(cal_data[j-1]);
    
    if(j == 19)
    {
      //reset cal data
      Rmin = 6000.0;
      Rmax = 0.0;
      Rref = 0.0;
      hyteresis = 0;
      
      for(int i = 0; i < 15; i++)
      {
        float t = cal_data[i];

        if(t < Rmin && t > 0.01) Rmin = t;
        if(t > Rmax) Rmax = t;
      }

      Rref = (Rmin + Rmax)/2;
      beatTimer = millis();
      hyteresis = 0.02 * Rref;
      calibrated = true;
      Serial.print("Rmin = ");
      Serial.print(Rmin);
      Serial.print(", Rmax = ");
      Serial.print(Rmax);
      Serial.println(F(" --> Done Calibrating"));
    }
    delay(500);
  }
  else j = 0;
}

int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
    delayMicroseconds(5);
  }
  return reading/sampleSize;
}

float compute_R(float X, float Y, float Z)
{
  return sqrt((pow(X, 2)) + (pow(Y, 2)) + (pow(Z, 2)));
}


void handleMinAndMax()
{
  
}
/*
void handleAllReadings()
{
  while ( ADC_Sampler.available() ) 
  {
    count++;
    uint16_t* x = ADC_Sampler.data();
    for (int i = 0; i < ADC_sequencer_size; i++) 
    {
      adcBuffer[i] += x[i];
    }
  }

  if(count >= MAXSAMPLE)
  {
    Serial.print("\tcount: ");
    Serial.print(count);
    Serial.print('\t');
    for(int i = 0; i < ADC_sequencer_size; i++)
    {
      adcBuffer[i] = adcBuffer[i]/count;
      Serial.print(adcBuffer[i]);
      Serial.print('\t');
    }
    Serial.println();
    handleFlowRateReading();
    handleO2Reading();
    count = 0;
    memset(adcBuffer, 0, sizeof(adcBuffer));
  } 
  delay(200);
}
*/

void HandleRestoreArm()
{
  analogWrite(EN_PIN, 500);
  //digitalWriteDirect(EN_PIN, HIGH);
  while(digitalReadDirect(STOP_SWITCH1) == HIGH || digitalReadDirect(STOP_SWITCH2) == HIGH)
  {
    digitalWriteDirect(IN1_PIN, HIGH);
    digitalWriteDirect(IN2_PIN, LOW);
    delay(100);
  }
  digitalWriteDirect(IN1_PIN, HIGH);
  digitalWriteDirect(IN2_PIN, HIGH);
  digitalWriteDirect(EN_PIN, LOW);
  
}
