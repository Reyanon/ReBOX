#ifndef RBX_FRONTEND_RESAMPLE_H
#define RBX_FRONTEND_RESAMPLE_H

#include <stdint.h>

// Resampler for audio DRC (stereo INT16, 16-tap Lanczos, 1024 phases)
// Linear interpolation reached only 31dB in-band SNR: high-frequency attenuation varied periodically as the fractional
// position drifted (crackling). 16 taps give 68dB under the same conditions. Latency is 8 samples.

#define RBX_RESAMPLE_MAX_IN 4096 // max sample pairs per call

void rbx_resample_init(void);

// Resample frames sample pairs from in at step (16.16, input advance per output sample) into out.
// Returns the number of output sample pairs. out must hold at least frames * 65536 / step + 2 pairs.
int rbx_resample(const int16_t *in, int frames, uint32_t step, int16_t *out);
int rbx_resample_fast_rint(void);   // (ADR-161: the inline rounding in use, checked equal to lrintf at init)
void rbx_resample_set_neon(int on);   // (ADR-180: 0 = the scalar loop as before, the data folder's "feaudold")
int rbx_resample_neon(void);          // (the NEON loop in use)

#endif
