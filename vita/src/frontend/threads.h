#ifndef RBX_FRONTEND_THREADS_H
#define RBX_FRONTEND_THREADS_H

// Core assignment (docs/RULES.md ch. 3)
//   CPU0: main (V33, video, input, vsync) - main() thread
//   CPU1: sound (V35, YM2151, GA20 -> audio ring) - thread in this file
//   CPU2: render (added in M3-c)

// Pin the main thread to CPU0
void rbx_threads_pin_main(void);

// CPU3 (mask 0x80000) for user threads: closed on a stock system (only a plugin such as CoreUnlocker opens it). Asked
// once: a thread must really start and end there within a time limit, a created thread alone proves nothing
int rbx_cpu3_usable(void);
void rbx_cpu3_recheck(void);   // (after the CPU3 unlock: the next rbx_cpu3_usable tries again)
// a thread meant for CPU3: there when it is open, else the mask given
int rbx_cpu3_mask_or(int fallback_mask);

// Start the sound thread on CPU1. Call after rbx_core_init(threaded=1)
int rbx_threads_start_sound(void);

// Other boards: YM2151 synthesis worker on CPU1 (only if the core supports it, 0 = started)
int rbx_threads_start_ym(void);

// The io worker (ADR-160: a file region's reads ahead, the CV1000 NAND) for a game that wants one: CPU1 at 72
// (RBX_DATA_DIR "nandcpu3": CPU3, "nandsync" / "nandold": none). Started after the game's init, stopped before its exit
void rbx_threads_start_io(void);
void rbx_threads_stop_io(void);

// Stop and join the sound/YM threads (includes rbx_core_stop)
void rbx_threads_tick(void);   // once a frame (thread placement by load)
void rbx_threads_stop(void);

#endif
