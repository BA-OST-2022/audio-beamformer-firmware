#include <SPI.h>
#include <Audio.h>
#include "AudioOutputCustomPWM.h"

#define AUDIO_PWM           0
#define AUDIO_SEL           1
#define POTI                41

#define OVERSAMPLING_RATE   2

#define SIGNAL_FREQ         1000
#define CARRIER_FREQ        40000
#define SAMPLE_RATE         (CARRIER_FREQ * OVERSAMPLING_RATE)

#define SPI_SCLK            27
#define SPI_MOSI            26
#define SPI_CS              38


AudioInputUSB usb;
//AudioOutputCustomPWM pwm(AUDIO_PWM, AUDIO_SEL);
AudioOutputI2S i2s;
AudioSynthWaveformDc dc;
AudioSynthWaveformSine sine;
AudioSynthWaveform waveform;
AudioMixer4 mixer;
//AudioConnection patchCord1(usb, 0, pwm, 0);
AudioConnection patchCord2(usb, 0, i2s, 0);
AudioConnection patchCord3(usb, 1, i2s, 1);

//AudioConnection patchCord2(waveform, 0, i2s, 0);
//AudioConnection patchCord3(waveform, 0, i2s, 1);

//AudioConnection patchCord2(dc, 0, mixer, 0);
//AudioConnection patchCord3(sine, 0, mixer, 1);

//AudioConnection patchCord4(mixer, 0, i2s, 0);
//AudioConnection patchCord5(mixer, 0, i2s, 1);

IntervalTimer timer, ramp, sigmaDelta;

void pwmUpdate(void);
void rampUpdate(void);
void sigmaDeltaUpdate(void);

static volatile float out = 0.0, val = 0.0;
static volatile float signalFreq = 1000.0;

float sinewave[OVERSAMPLING_RATE];


void setup()
{
  Serial.begin(0);
  AudioMemory(12);

  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_CS, OUTPUT);
  pinMode(AUDIO_PWM, OUTPUT);
  pinMode(AUDIO_SEL, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(POTI, INPUT);

  SPI1.setSCK(SPI_SCLK);
  SPI1.setMOSI(SPI_MOSI);
  //SPI1.setCS(SPI_CS);
  SPI1.begin();

  for(int i = 0; i < OVERSAMPLING_RATE; i++)
  {
    sinewave[i] = cosf(2 * PI * ((float) i) / (float) OVERSAMPLING_RATE);
  }

  //dc.amplitude(0.17);
  //dc.amplitude(-(1.0/32767.0));
  
  //dc.amplitude(0.5);
  //sine.amplitude(1.0);
  //sine.frequency(44000/2);

  //waveform.begin(1, WAVEFORM_SQUARE, 44100/8);
  //waveform.offset(-0.5);
  
  //analogWriteResolution(8);
  //analogWriteFrequency(AUDIO_PWM, 80000);
  //analogWriteFrequency(AUDIO_SEL, 40000);

  //analogWrite(AUDIO_PWM, 127);
  //analogWrite(AUDIO_SEL, 512);

  //timer.begin(pwmUpdate, 1000000 / SAMPLE_RATE);   // [us]
  //ramp.begin(rampUpdate, 1000000 / AUDIO_SAMPLE_RATE_EXACT);
  //sigmaDelta.begin(sigmaDeltaUpdate, 1);
}

void loop()
{
  /*
  static uint32_t t = 0; t++;
  if(t >= 1000000)
  {
    static uint32_t time = 0;
    Serial.printf("Delta-Sigma Frequency: %f\n", 1e12 / (float)(micros() - time));
    time = micros();
    t = 0;
  }
  */

  /*
  // Noise Shaping 1. Order
  static bool signal = false;
  static float error = 0.0;
  float target = error + out;
  signal = (target > 0.0);
  error = target - ((((float) signal) * 2.0) - 1.0);
  digitalWriteFast(AUDIO_PWM, signal);
  */
  
  /*
  // Noise Shaping 2. Order
  static bool signal = false;
  static float error1 = 0.0, error2 = 0.0;
  float target = out + 2.0 * error1 - error2;
  signal = (target > 0.0);
  error2 = error1;
  error1 = target - ((((float) signal) * 2.0) - 1.0);
  digitalWriteFast(AUDIO_PWM, signal);
  */

  //delayMicroseconds(1);

  /*static bool dir = false;
  static float ramp = 0.0;
  if(ramp >= 0.999) dir = false;
  if(ramp <= -0.999) dir = true;
  ramp += (dir)? 0.001 : -0.001;
  dc.amplitude(ramp);
  delay(10);*/

  uint32_t v = 32767;
  for(int i = 0; i < (1 << 12); i++)
  {
    v += analogRead(POTI);
  }
  v >>= 7;
  if(v > 32767) v = 32767;

  //static uint16_t v = 32767;
  static uint8_t interpol = 0x01;
  static bool once = true;

  if(Serial.available())
  {
    once = false;
    delay(1);
    interpol = Serial.parseInt();
    Serial.println(interpol);
    while(Serial.available()) Serial.read();
  }

  SPI1.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWriteFast(SPI_CS, 0);
  

  /*
  for(int i = 0; i < 64; i++)
  {
    SPI1.transfer(interpol);
  }
  */
  
  SPI1.transfer(v & 0xFF);
  SPI1.transfer((v >> 8) & 0xFF);
  SPI1.transfer(interpol);// & 0x07);
  digitalWriteFast(SPI_CS, 1);
  SPI1.endTransaction();
  
  
  delay(1);


  //Serial.printf("%02X%02X\n", (v >> 8) & 0xFF, v & 0xFF);
}

void sigmaDeltaUpdate(void)
{
  /*
  static bool signal = false;
  static float error = 0.0;
  float target = error + out;
  signal = (target > 0.0);
  error = target - (signal? 1.0 : -1.0);
  digitalWriteFast(AUDIO_PWM, signal);
  */
  
  static bool signal = false;
  static int16_t error = 0;
  int16_t target = error + (int16_t)(out * 16383);
  signal = (target > 0);
  error = target + (signal? -16383 : 16383);
  digitalWriteFast(AUDIO_PWM, signal);
  
}

void pwmUpdate(void)
{
  static uint32_t t = 0; t++;
  float carrier = sinewave[t % OVERSAMPLING_RATE];
  //static float carrier = 1.0; carrier *= -1.0;
  //float carrier = cosf(2.0 * PI * CARRIER_FREQ * (float) t * (1.0 / (float) SAMPLE_RATE));
  //val = cosf(2.0 * PI * signalFreq * (float) t * (1.0 / (float) SAMPLE_RATE));
  

  const float m = 1.0;
  out = carrier * (1.0 + m * val) * 0.5;
  //out = val;
  //out = carrier;
}

void rampUpdate(void)
{
  static bool dir = true;
  if(signalFreq > 3000)
  {
    dir = false;
  }
  if(signalFreq < 500)
  {
    dir = true;
  }

  static float clip = 0.0;
  //val = pwm.data[pwm.pos] * 1.0;
  //if(pwm.pos < 127) pwm.pos++;
  //if(fabs(val) > clip) clip = fabs(val);

  //Serial.printf("%f, %f\n", val, clip);
  //signalFreq += (dir) ? 0.001 : -0.001;
  //Serial.println(pwm.data[0]);
}
