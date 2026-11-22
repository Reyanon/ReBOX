// The sampled activity profiler (ADR-155, log builds): a thread reads the emulation thread's activity (rbx_prof_cur,
// rbx_prof.h) every 250 us and logs the shares every 5 seconds ("prof: ..."). Off unless the data folder has the file
// "prof" (CPU3 at the lowest priority) or "profhi" (CPU3 at a high priority, not starved by the blitter lane there;
// it takes a few us of that core each time). CPU2 when CPU3 is closed
#ifndef RBX_FRONTEND_SAMPLER_H
#define RBX_FRONTEND_SAMPLER_H

void rbx_sampler_start(void);   // at a game's start (after its threads)
void rbx_sampler_stop(void);    // at its end

#endif
