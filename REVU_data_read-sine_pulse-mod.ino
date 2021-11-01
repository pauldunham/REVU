//11NOV20: Remove sensor LED pin initialisation
//15FEB21 Changed to Teensy 4.1
//22FEB21 Added limit switches
//23FEB21 Added interrupts for limit switches
//26FEB21 Added audio
//8MAR21 Added data ouptut array, changed motor motion (simplified)
//10MAR21 Merged code for optophonic mode
//15MAR21 Added reset to home on startup & white sounding Optophone mode
//14JUN21 Changed counter to longer extent. Change in dpi() and End().


#include <Encoder.h>
#include <Entropy.h>
#include <TeensyThreads.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>

#define NB_LED 16
#define NB_PIXELS 27 // Pixels/sensor. Adapt to control your speed  and precision 
#define NB_PIXELS_MAX 2700

#define ANALOG_PIN A8 // Sensor Analog pin
#define SENSOR_LE 2 // CIS Latch pin 
#define SENSOR_CLK 3 // CIS Clock pin

#define motorA2 29 //Motor A2 in
#define motorA1 28 //Motor A1 in

#define ENCODER_OPTIMIZE_INTERRUPTS

//Storage table for CIS data
int tab[NB_PIXELS];

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthWaveformSine   sine3;          //xy=130,124
AudioSynthWaveformSine   sine7;          //xy=136,256
AudioSynthWaveformSine   sine11;         //xy=142,387
AudioSynthWaveformSine   sine15;         //xy=149,507
AudioSynthWaveformSine   sine1;          //xy=241,59
AudioSynthWaveform       waveform1;      //xy=241,94
AudioSynthWaveformSine   sine5;          //xy=246,191
AudioSynthWaveformSine   sine9;          //xy=252,320
AudioSynthWaveform       waveform5;      //xy=256,154
AudioSynthWaveform       waveform2;      //xy=256,228
AudioSynthWaveformSine   sine13;         //xy=256,448
AudioSynthWaveform       waveform4;      //xy=259,279
AudioSynthWaveform       waveform3;      //xy=261,358
AudioSynthWaveform       waveform6;      //xy=262,411
AudioSynthWaveform       waveform8;      //xy=263,532
AudioSynthWaveform       waveform7;      //xy=265,486
AudioSynthWaveformModulated waveformMod2;   //xy=424,129
AudioSynthWaveformModulated waveformMod1;   //xy=425,65
AudioSynthWaveformModulated waveformMod7;   //xy=425,455
AudioSynthWaveformModulated waveformMod8;   //xy=425,513
AudioSynthWaveformModulated waveformMod5;   //xy=426,326
AudioSynthWaveformModulated waveformMod6;   //xy=426,393
AudioSynthWaveformModulated waveformMod3;   //xy=427,197
AudioSynthWaveformModulated waveformMod4;   //xy=427,262
AudioMixer4              mixer2;         //xy=641,161
AudioMixer4              mixer1;         //xy=646,422
AudioOutputI2S           i2s1;           //xy=826,288
AudioConnection          patchCord1(sine3, 0, waveformMod2, 0);
AudioConnection          patchCord2(sine7, 0, waveformMod4, 0);
AudioConnection          patchCord3(sine11, 0, waveformMod6, 0);
AudioConnection          patchCord4(sine15, 0, waveformMod8, 0);
AudioConnection          patchCord5(sine1, 0, waveformMod1, 0);
AudioConnection          patchCord6(waveform1, 0, waveformMod1, 1);
AudioConnection          patchCord7(sine5, 0, waveformMod3, 0);
AudioConnection          patchCord8(sine9, 0, waveformMod5, 0);
AudioConnection          patchCord9(waveform5, 0, waveformMod2, 1);
AudioConnection          patchCord10(waveform2, 0, waveformMod3, 1);
AudioConnection          patchCord11(sine13, 0, waveformMod7, 0);
AudioConnection          patchCord12(waveform4, 0, waveformMod4, 1);
AudioConnection          patchCord13(waveform3, 0, waveformMod5, 1);
AudioConnection          patchCord14(waveform6, 0, waveformMod6, 1);
AudioConnection          patchCord15(waveform8, 0, waveformMod8, 1);
AudioConnection          patchCord16(waveform7, 0, waveformMod7, 1);
AudioConnection          patchCord17(waveformMod2, 0, mixer2, 1);
AudioConnection          patchCord18(waveformMod1, 0, mixer2, 0);
AudioConnection          patchCord19(waveformMod7, 0, mixer1, 2);
AudioConnection          patchCord20(waveformMod8, 0, mixer1, 3);
AudioConnection          patchCord21(waveformMod5, 0, mixer1, 0);
AudioConnection          patchCord22(waveformMod6, 0, mixer1, 1);
AudioConnection          patchCord23(waveformMod3, 0, mixer2, 2);
AudioConnection          patchCord24(waveformMod4, 0, mixer2, 3);
AudioConnection          patchCord25(mixer2, 0, i2s1, 0);
AudioConnection          patchCord26(mixer2, 0, i2s1, 1);
AudioConnection          patchCord27(mixer1, 0, i2s1, 1);
AudioConnection          patchCord28(mixer1, 0, i2s1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=724,576
// GUItool: end automatically generated code


/* Modes
    0 = G1 scale
    1 = G2 scale
    2 = G3 scale, "Optophonic mode" */
int freqMode = 2;

//G1, G2 & G3
float freqMatrix[3][8] = {
  {48.999, 65.406, 73.416, 82.407, 97.999, 123.471, 130.813, 164.814},
  {97.999, 130.813, 146.832, 164.814, 195.998, 246.942, 261.626, 329.628},
  {195.998, 261.626, 293.665, 329.628, 391.995, 493.883, 523.251, 659.255},
};

Encoder myEnc(34, 33);
int counter = 0;
int dpiMode = 0;
int homeState = HIGH;
int endState = HIGH;
int idG, idC, idD, idE, idg, idb, idc, ide, idLED;//thread IDs
int current_waveform = 0;
int mod_waveform = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Entropy.Initialize();

  pinMode(SENSOR_LE, OUTPUT);
  pinMode(SENSOR_CLK, OUTPUT);
  pinMode(motorA1, OUTPUT); //motor pin
  pinMode(motorA2, OUTPUT); //motor pin
  pinMode(30, INPUT_PULLUP); //home switch
  pinMode(31, INPUT_PULLUP); //outer limit switch
  pinMode(33, INPUT_PULLUP); //encoder pin
  pinMode(34, INPUT_PULLUP); //encoder pin
  pinMode(32, INPUT_PULLDOWN); //1st mode switch
  pinMode(27, INPUT_PULLDOWN); //2nd mode switch

  AudioMemory(20);
  sgtl5000_1.enable();

  for (int i = 0; i < 5; i++) {
    mixer1.gain(i, 0.5);
    mixer2.gain(i, 0.5);
  }

  attachInterrupt(31, Home, CHANGE);
  attachInterrupt(30, End, CHANGE);
  attachInterrupt(34, updateEncoder, CHANGE);
  seekHome();

  waveformMod1.amplitude(1);
  waveformMod2.amplitude(1);
  waveformMod3.amplitude(1);
  waveformMod4.amplitude(1);
  waveformMod5.amplitude(1);
  waveformMod6.amplitude(1);
  waveformMod7.amplitude(1);
  waveformMod8.amplitude(1);

  sine1.frequency(4);
  sine3.frequency(4);
  sine5.frequency(4);
  sine7.frequency(4);
  sine9.frequency(4);
  sine11.frequency(4);
  sine13.frequency(4);
  sine15.frequency(4);

  waveformMod1.frequency(freqMatrix[freqMode][0]);
  waveformMod2.frequency(freqMatrix[freqMode][1]);
  waveformMod3.frequency(freqMatrix[freqMode][2]);
  waveformMod4.frequency(freqMatrix[freqMode][3]);
  waveformMod5.frequency(freqMatrix[freqMode][4]);
  waveformMod6.frequency(freqMatrix[freqMode][5]);
  waveformMod7.frequency(freqMatrix[freqMode][6]);
  waveformMod8.frequency(freqMatrix[freqMode][7]);

  current_waveform = WAVEFORM_SINE;
  mod_waveform = WAVEFORM_PULSE;
  waveformMod1.begin(current_waveform);
  waveformMod2.begin(current_waveform);
  waveformMod3.begin(current_waveform);
  waveformMod4.begin(current_waveform);
  waveformMod5.begin(current_waveform);
  waveformMod6.begin(current_waveform);
  waveformMod7.begin(current_waveform);
  waveformMod8.begin(current_waveform);

  waveform1.frequency(4);
  waveform2.frequency(4);
  waveform3.frequency(4);
  waveform4.frequency(4);
  waveform5.frequency(4);
  waveform6.frequency(4);
  waveform7.frequency(4);
  waveform8.frequency(4);

  waveform1.pulseWidth(0.6);
  waveform2.pulseWidth(0.6);
  waveform3.pulseWidth(0.6);
  waveform4.pulseWidth(0.6);
  waveform5.pulseWidth(0.6);
  waveform6.pulseWidth(0.6);
  waveform7.pulseWidth(0.6);
  waveform8.pulseWidth(0.6);

  waveform1.begin(mod_waveform);
  waveform2.begin(mod_waveform);
  waveform3.begin(mod_waveform);
  waveform4.begin(mod_waveform);
  waveform5.begin(mod_waveform);
  waveform6.begin(mod_waveform);
  waveform7.begin(mod_waveform);
  waveform8.begin(mod_waveform);
}

void loop() {
  counter = 0;
  idLED = threads.addThread(init_LED);
  dpiMode = Entropy.random(0, 3);
  dpi(dpiMode);
  delay(500);
  whiteDetect();
  delay(5000);
}

void motorStop() { //Stop motor movement
  analogWrite(motorA1, 0);
  analogWrite(motorA2, 0);
}

void whiteDetect() { //Start/finish head shuffle. MFP looks for white detect strip for home
  delay(10);
  while (counter < 1500) {
    analogWrite(motorA1, 255);
    analogWrite(motorA2, 127);
    delay(100);
    motorStop();
  }
  delay(200);
  while (counter > 0) {
    analogWrite(motorA1, 96);
    analogWrite(motorA2, 255);
    delay(10);
    motorStop();
  }
  counter = 0;
}

void dpi(int dpiMode) {
  while (counter < 180000) {
    sgtl5000_1.volume(0.5);
    if (dpiMode == 0) {
      analogWrite(motorA1, 255);
      analogWrite(motorA2, 130);
      delay(1);
      motorStop();
    }
    else if (dpiMode == 1) {
      analogWrite(motorA1, 255);
      analogWrite(motorA2, 160);
      delay(1);
      motorStop();
    }
    else if (dpiMode == 2) {
      analogWrite(motorA1, 255);
      analogWrite(motorA2, 190);
      delay(1);
      motorStop();
    }
  }
  //sgtl5000_1.volume(0);
  threads.kill(idLED);
  delay(500);
  while (counter > 0) {
    analogWrite(motorA1, 127);
    analogWrite(motorA2, 255);
    delay(1);
    motorStop();
  }
  counter = 0;
}

void seekHome() {
  while (homeState == HIGH) {
    analogWrite(motorA1, 160);
    analogWrite(motorA2, 255);
    delay(1);
    motorStop();
    homeState = digitalRead(31);
    if (homeState == LOW) {
      counter = 0;
      delay(500);
      whiteDetect();
      delay(5000);
    }
  }
}

//Encoder Interrupt
void updateEncoder() {
  int lastState = 0;
  int currentState = digitalRead(34);
  if (currentState != lastState && currentState == 1) {
    if (digitalRead(33) != currentState) {
      counter--;
    }
    else {
      counter++;
    }
  }
  lastState = digitalRead(34);
}

//Home Limit Switch Interrupt
void Home() {
  homeState = digitalRead(31);
  if (homeState == LOW) {
    counter = 0;
  }
}

//End Limit Switch Interrupt
void End() {
  endState = digitalRead(30);
  //Serial.println(endState);
  if (endState == LOW) {
    counter = 180000;
  }
}

void LED_IR(int numled, boolean state) {
  digitalWrite(numled + 6, state); //LED number is 0 but the pin start at 6
}

void init_LED() {
  while (counter < 180000) {
    boolean flag_detect = 0;
    for (int l = 0; l < NB_LED; l++) {
      //Turn ON a sensor LED (common anode, LOW = on and HIGH = OFF)
      LED_IR(l, LOW);
      // Clear/reset the sensor
      clear_CCD();
      // Read and store the data, return 1 if something was seen
      if (read_CCD(l) == 1) {
        flag_detect = 1;
      }
      // Turn off the LED
      LED_IR(l, HIGH);
    }
    if (flag_detect == 1) {
      idG = threads.addThread(topG);
      idC = threads.addThread(C);
      idD = threads.addThread(D);
      idE = threads.addThread(E);
      idg = threads.addThread(bottomG);
      idb = threads.addThread(bottomB);
      idc = threads.addThread(bottomC);
      ide = threads.addThread(bottomE);
      delay(50);
    }
  }
}

void clear_CCD() {
  digitalWrite(SENSOR_LE, HIGH);
  digitalWrite(SENSOR_CLK, HIGH);
  digitalWrite(SENSOR_CLK, LOW);
  digitalWrite(SENSOR_LE, LOW);
  for (int i = 0; i < NB_PIXELS_MAX; i++) {
    digitalWrite(SENSOR_CLK, HIGH);
    digitalWrite(SENSOR_CLK, LOW);
  }
}

boolean read_CCD(int led_on) {
  //boolean flag_detect = 0;
  digitalWrite(SENSOR_LE, HIGH);
  digitalWrite(SENSOR_CLK, HIGH);
  digitalWrite(SENSOR_CLK, LOW);
  digitalWrite(SENSOR_LE, LOW);

  for (int i = 0; i < NB_PIXELS; i++) {
    for (int j = 0; j < NB_PIXELS_MAX; j++) {
      digitalWrite(SENSOR_CLK, HIGH);
      digitalWrite(SENSOR_CLK, LOW);
    }
    Serial.println(analogRead(ANALOG_PIN));
  }
  return analogRead(ANALOG_PIN);
}

void topG() {
  // if (analogRead(ANALOG_PIN) < 388) {
  sine1.amplitude((float)analogRead(A8) / 1023.0);
  waveform1.amplitude(1-(float)analogRead(A8) / 1023.0);
  threads.delay(Entropy.random(200, 1000));
  sine1.amplitude((float)analogRead(A8) / 1023.0);
  waveform1.amplitude(1-(float)analogRead(A8) / 1023.0);
  // }
}

void C() {
  // if ((analogRead(ANALOG_PIN) > 387) && (analogRead(ANALOG_PIN) < 476)) {
  sine3.amplitude((float)analogRead(A8) / 1023.0);
  waveform2.amplitude(1-(float)analogRead(A8) / 1023.0);
  threads.delay(Entropy.random(200, 1000));
  sine3.amplitude((float)analogRead(A8) / 1023.0);
  waveform2.amplitude(1-(float)analogRead(A8) / 1023.0);
  // }
}

void D() {
  // if ((analogRead(ANALOG_PIN) > 475) && (analogRead(ANALOG_PIN) < 563)) {
  sine5.amplitude((float)analogRead(A8) / 1023.0);
  waveform3.amplitude(1-(float)analogRead(A8) / 1023.0);
  threads.delay(Entropy.random(200, 1000));
  sine5.amplitude((float)analogRead(A8) / 1023.0);
  waveform3.amplitude(1-(float)analogRead(A8) / 1023.0);
  // }
}

void E() {
  // if ((analogRead(ANALOG_PIN) > 562) && (analogRead(ANALOG_PIN) < 651)) {
  sine7.amplitude((float)analogRead(A8) / 1023.0);
  waveform4.amplitude(1-(float)analogRead(A8) / 1023.0);
  threads.delay(Entropy.random(200, 1000));
  sine7.amplitude((float)analogRead(A8) / 1023.0);
  waveform4.amplitude(1-(float)analogRead(A8) / 1023.0);
  // }
}

void bottomG() {
  //  if ((analogRead(ANALOG_PIN) > 650) && (analogRead(ANALOG_PIN) < 739)) {
  sine9.amplitude((float)analogRead(A8) / 1023.0);
  waveform5.amplitude(1-(float)analogRead(A8) / 1023.0);
  threads.delay(Entropy.random(200, 1000));
  sine9.amplitude((float)analogRead(A8) / 1023.0);
  waveform5.amplitude(1-(float)analogRead(A8) / 1023.0);
  //  }
}

void bottomB() {
  // if ((analogRead(ANALOG_PIN) > 738) && (analogRead(ANALOG_PIN) < 826)) {
  sine11.amplitude((float)analogRead(A8) / 1023.0);
  waveform6.amplitude(1-(float)analogRead(A8) / 1023.0);
  threads.delay(Entropy.random(200, 1000));
  sine11.amplitude((float)analogRead(A8) / 1023.0);
  waveform6.amplitude(1-(float)analogRead(A8) / 1023.0);
  //  }
}

void bottomC() {
  // if ((analogRead(ANALOG_PIN) > 825) && (analogRead(ANALOG_PIN) < 914)) {
  sine13.amplitude((float)analogRead(A8) / 1023.0);
  waveform7.amplitude(1-(float)analogRead(A8) / 1023.0);
  threads.delay(Entropy.random(200, 1000));
  sine13.amplitude((float)analogRead(A8) / 1023.0);
  waveform7.amplitude(1-(float)analogRead(A8) / 1023.0);
  // }
}

void bottomE() {
  // if (analogRead(ANALOG_PIN) > 914) {
  sine15.amplitude((float)analogRead(A8) / 1023.0);
  waveform8.amplitude(1-(float)analogRead(A8) / 1023.0);
  threads.delay(Entropy.random(200, 1000));
  sine15.amplitude((float)analogRead(A8) / 1023.0);
  waveform8.amplitude(1-(float)analogRead(A8) / 1023.0);
  // }
}
