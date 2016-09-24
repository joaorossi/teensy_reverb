/*  Reverb for Teensy
 *
 *  Author: Joao Rossi Filho
 *          joaorossifilho@gmail.com
 *
 *  Soon I'll make a commit with nice comments and discription
 */

#ifndef effect_reverb_
#define effect_reverb_

#include "AudioStream.h"
#include "utility/dspinst.h"
#include "math_helper.h"
#include <cmath>

#define APF1_BUF_LEN 600
#define APF2_BUF_LEN 1300
#define APF3_BUF_LEN 500

#define LPF1_BUF_LEN 1400
#define LPF2_BUF_LEN 1700
#define LPF3_BUF_LEN 1800
#define LPF4_BUF_LEN 2000

#define APF1_DLY_LEN 586
#define APF2_DLY_LEN 1239
#define APF3_DLY_LEN 485

#define LPF1_DLY_LEN 1398
#define LPF2_DLY_LEN 1637
#define LPF3_DLY_LEN 1774
#define LPF4_DLY_LEN 1947

#define LPF1_DLY_SEC 0.03171 
#define LPF2_DLY_SEC 0.03711
#define LPF3_DLY_SEC 0.04023
#define LPF4_DLY_SEC 0.04414

struct comb_apf {
  int32_t   g;
  int32_t   *buffer;
  uint32_t  buf_len;
  uint32_t  delay, rd_idx, wr_idx;
};

struct comb_lpf {
  int32_t   g1, g2, z1;
  int32_t   *buffer;
  uint32_t  buf_len;
  uint32_t  delay, rd_idx, wr_idx;
};

class AudioEffectReverb : public AudioStream
{
  public:
    AudioEffectReverb(void) : AudioStream(1, inputQueueArray)
    {
      init_comb_filters();
      clear_buffers();
      reverb_time(5.0);
    }

    virtual void update(void);

    void reverb_time(float);

  private:
    void init_comb_filters(void);
    void clear_buffers(void);

    audio_block_t *inputQueueArray[1];

    struct comb_apf apf[3];
    struct comb_lpf lpf[4];

    float   reverb_time_sec;

    float   g_flt_apf[3];
    int32_t g_q31_apf[3];

    float   g1_flt_lpf[4];
    int32_t g1_q31_lpf[4];

    float   g2_flt_lpf;
    int32_t g2_q31_lpf;

    int32_t apf1_buf[APF1_BUF_LEN];
    int32_t apf2_buf[APF2_BUF_LEN];
    int32_t apf3_buf[APF3_BUF_LEN];

    int32_t lpf1_buf[LPF1_BUF_LEN];
    int32_t lpf2_buf[LPF2_BUF_LEN];
    int32_t lpf3_buf[LPF3_BUF_LEN];
    int32_t lpf4_buf[LPF4_BUF_LEN];

    int32_t q31_buf[AUDIO_BLOCK_SAMPLES];
    int32_t sum_buf[AUDIO_BLOCK_SAMPLES];
    int32_t aux_buf[AUDIO_BLOCK_SAMPLES];
};

#endif
