#ifndef AUDIO_OUTPUT_CUSTOM_PWM_H
#define AUDIO_OUTPUT_CUSTOM_PWM_H

#include "Arduino.h"
#include "AudioStream.h"
#include "DMAChannel.h"

struct _pwm_pin_info_struct_c {
  uint8_t type;    // 0=no pwm, 1=flexpwm, 2=quad
  uint8_t module;  // 0-3, 0-3
  uint8_t channel; // 0=X, 1=A, 2=B
  uint8_t muxval;  //
};

struct _audio_info_flexpwm_c {
  IMXRT_FLEXPWM_t *flexpwm;
  _pwm_pin_info_struct_c info;
  uint8_t pin;
};

class AudioOutputCustomPWM : public AudioStream
{
  public:
    AudioOutputCustomPWM(uint8_t pinPwm, uint8_t pinSel) : AudioStream(1, inputQueueArray) {begin(pinPwm, pinSel);}
    virtual void update(void);

    volatile float data[AUDIO_BLOCK_SAMPLES];
    volatile int pos = 0;
    
  private:
    static bool update_responsibility;
    audio_block_t *inputQueueArray[1];
    static void isr(void);
    void begin(uint8_t pin1, uint8_t pin2); // FlexPWM pins only
    static audio_block_t *block;
    static DMAChannel dma[2];
    static _audio_info_flexpwm_c apins[2];
};


#endif
