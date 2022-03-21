#include "AudioOutputCustomPWM.h"

bool AudioOutputCustomPWM::update_responsibility = false;

static const uint8_t silence[2] = {0x80, 0x00};
extern uint8_t analog_write_res;
extern const struct _pwm_pin_info_struct_c pwm_pin_info[];
audio_block_t * AudioOutputCustomPWM::block = NULL;
DMAMEM __attribute__((aligned(32))) static uint16_t pwm_tx_buffer[2][AUDIO_BLOCK_SAMPLES * 2];
DMAChannel AudioOutputCustomPWM::dma[2];
_audio_info_flexpwm_c AudioOutputCustomPWM::apins[2];

FLASHMEM
void AudioOutputCustomPWM::begin(uint8_t pin1, uint8_t pin2)
{
  /*
  analogWriteResolution(8);
  const uint8_t pins[2] = {pin1, pin2};
  for (unsigned i = 0; i < 2; i++) {
    // use the existing code here:
    analogWriteFrequency(pins[i], AUDIO_SAMPLE_RATE_EXACT);
    analogWrite(pins[i], silence[i]);
    //Fill structure
    apins[i].pin = pins[i];
    apins[i].info = pwm_pin_info[apins[i].pin];
    uint8_t dmamux_source;
    if (apins[i].info.type == 1) { //only for valid flexPWM pin:
      unsigned module = (apins[i].info.module >> 4) & 3;
      unsigned submodule = apins[i].info.module & 3;
      switch (module) {
        case 0: {
            apins[i].flexpwm = &IMXRT_FLEXPWM1;
            switch (submodule) {
              case 0: dmamux_source = DMAMUX_SOURCE_FLEXPWM1_WRITE0; break;
              case 1: dmamux_source = DMAMUX_SOURCE_FLEXPWM1_WRITE1; break;
              case 2: dmamux_source = DMAMUX_SOURCE_FLEXPWM1_WRITE2; break;
              default: dmamux_source = DMAMUX_SOURCE_FLEXPWM1_WRITE3;
            }
            break;
          }
        case 1: {
            apins[i].flexpwm = &IMXRT_FLEXPWM2;
            switch (submodule) {
              case 0: dmamux_source = DMAMUX_SOURCE_FLEXPWM2_WRITE0; break;
              case 1: dmamux_source = DMAMUX_SOURCE_FLEXPWM2_WRITE1; break;
              case 2: dmamux_source = DMAMUX_SOURCE_FLEXPWM2_WRITE2; break;
              default: dmamux_source = DMAMUX_SOURCE_FLEXPWM2_WRITE3;
            }
            break;
          }
        case 2: {
            apins[i].flexpwm = &IMXRT_FLEXPWM3;
            switch (submodule) {
              case 0: dmamux_source = DMAMUX_SOURCE_FLEXPWM3_WRITE0; break;
              case 1: dmamux_source = DMAMUX_SOURCE_FLEXPWM3_WRITE1; break;
              case 2: dmamux_source = DMAMUX_SOURCE_FLEXPWM3_WRITE2; break;
              default: dmamux_source = DMAMUX_SOURCE_FLEXPWM3_WRITE3;
            }
            break;
          }
        default: {
            apins[i].flexpwm = &IMXRT_FLEXPWM4;
            switch (submodule) {
              case 0: dmamux_source = DMAMUX_SOURCE_FLEXPWM4_WRITE0; break;
              case 1: dmamux_source = DMAMUX_SOURCE_FLEXPWM4_WRITE1; break;
              case 2: dmamux_source = DMAMUX_SOURCE_FLEXPWM4_WRITE2; break;
              default: dmamux_source = DMAMUX_SOURCE_FLEXPWM4_WRITE3;
            }
          }
        }
  volatile uint16_t *valReg;
  switch (apins[i].info.channel) {
      case 0:  valReg = &apins[i].flexpwm->SM[submodule].VAL0; break;
      case 1:  valReg = &apins[i].flexpwm->SM[submodule].VAL3; break;
      default:  valReg = &apins[i].flexpwm->SM[submodule].VAL5; break;
  }
  dma[i].begin(true);
  dma[i].TCD->SADDR = &pwm_tx_buffer[i][0];
  dma[i].TCD->SOFF = 2;
  dma[i].TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  dma[i].TCD->NBYTES_MLNO = 2;
  dma[i].TCD->SLAST = -sizeof(pwm_tx_buffer[0]);
  dma[i].TCD->DOFF = 0;
  dma[i].TCD->CITER_ELINKNO = sizeof(pwm_tx_buffer[0]) / 2;
  dma[i].TCD->DLASTSGA = 0;
  dma[i].TCD->BITER_ELINKNO = sizeof(pwm_tx_buffer[0]) / 2;
  dma[i].TCD->DADDR = valReg;
  dma[i].triggerAtHardwareEvent(dmamux_source);
  if (i == 1) { //One interrupt only
    dma[i].TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
    dma[i].attachInterrupt(isr);
  }
  //set PWM-DMA-Enable
  apins[i].flexpwm->SM[submodule].DMAEN = FLEXPWM_SMDMAEN_VALDE;
  //clear inital dma data:
  uint32_t modulo = apins[i].flexpwm->SM[apins[i].info.module & 3].VAL1;
  for (unsigned j=0; j<AUDIO_BLOCK_SAMPLES * 2; j++) {
    uint32_t cval = (silence[i] * (modulo + 1)) >> analog_write_res;
    if (cval > modulo) cval = modulo;
    pwm_tx_buffer[i][j] = cval;
  }
  arm_dcache_flush_delete(&pwm_tx_buffer[i][0], sizeof(pwm_tx_buffer[0]) / 2 );
      }
  }
  dma[0].enable();
  dma[1].enable();

  
  update_responsibility = update_setup();
  //pinMode(13,OUTPUT);
  */
}
void AudioOutputCustomPWM::isr(void)
{
  /*
  dma[1].clearInterrupt();
  uint16_t *dest, *dest1;
  uint32_t saddr = (uint32_t)(dma[0].TCD->SADDR);
  if (saddr < (uint32_t)&pwm_tx_buffer[0][AUDIO_BLOCK_SAMPLES]) {
    // DMA is transmitting the first half of the buffer
    // so we must fill the second half
    dest = &pwm_tx_buffer[0][AUDIO_BLOCK_SAMPLES];
    dest1 = &pwm_tx_buffer[1][AUDIO_BLOCK_SAMPLES];
    digitalWriteFast(13, 1);
  } else {
    // DMA is transmitting the second half of the buffer
    // so we must fill the first half
    dest = &pwm_tx_buffer[0][0];
    dest1 = &pwm_tx_buffer[1][0];
    digitalWriteFast(13, 0);
  }
        const uint32_t modulo[2] = { apins[0].flexpwm->SM[apins[0].info.module & 3].VAL1, apins[1].flexpwm->SM[apins[1].info.module & 3].VAL1};
  if (block) {
    for (unsigned i=0; i < AUDIO_BLOCK_SAMPLES; i++) {
      uint32_t sample = (uint16_t)block->data[i] + 0x8000;
      
      uint32_t msb = ((sample >> 8) & 255);  // + 120 ???
      uint32_t cval0 = (msb * (modulo[0] + 1)) >> analog_write_res;
      if (cval0 > modulo[0]) cval0 = modulo[0]; // TODO: is this check correct?
      *dest++ = cval0;
      
      uint32_t lsb = sample & 255;
      uint32_t cval1 = (lsb * (modulo[1] + 1)) >> analog_write_res;
      if (cval1 > modulo[1]) cval1 = modulo[1];
      *dest1++ = cval1;
    }
    arm_dcache_flush_delete(dest, sizeof(pwm_tx_buffer[0]) / 2 );
    arm_dcache_flush_delete(dest1, sizeof(pwm_tx_buffer[1]) / 2 );
    
    AudioStream::release(block);
    block = NULL;
  } else {
    //Serial.println(".");
    // fill with silence when no data available
    uint32_t cval0 = (silence[0] * (modulo[0] + 1)) >> analog_write_res;
    if (cval0 > modulo[0]) cval0 = modulo[0];
    uint32_t cval1 = (silence[1] * (modulo[1] + 1)) >> analog_write_res;
    if (cval1 > modulo[1]) cval1 = modulo[1];
    for (unsigned i=0; i < AUDIO_BLOCK_SAMPLES / 2; i++) {
      *dest++ = cval0;
      *dest++ = cval0;
      *dest1++ = cval1;
      *dest1++ = cval1;
    }
    arm_dcache_flush_delete(dest, sizeof(pwm_tx_buffer[0]) / 2 );
    arm_dcache_flush_delete(dest1, sizeof(pwm_tx_buffer[1]) / 2 );
  }
        AudioStream::update_all();
  //digitalWriteFast(13, !digitalRead(13));
  */
}
void AudioOutputCustomPWM::update(void)
{
  audio_block_t *tblock;
  tblock = receiveReadOnly();
  if (!tblock) return;
  //__disable_irq();
  pos = 0;
  for(int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
  {
    data[i] = ((float) tblock->data[i]) / 32768.0;
  }
  //__enable_irq();
  release(tblock);
}
