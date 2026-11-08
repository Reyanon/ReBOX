// ReBOX kernel helper, user side (vita/kmod/rbx_kctrl.c)
#ifndef RBX_KCTRL_H
#define RBX_KCTRL_H
unsigned int reiKCtrlPsState(void); // bits 0-14 single PS presses, 15-29 double presses, 30 held
int reiKCtrlGoHome(void);           // press PS for the system once (after the app lets go of its lock)
int reiKCtrlInstallOnExit(void);    // once the app has exited: ux0:data/rebox/update/files -> the app folder
// CPU3 for the app's threads: on 1 hook (a verified firmware only), 0 release, -1 read only; out[4]: the module NID, the
// checked function's first 8 bytes, hooked. Returns 1 hooked, 0 not, negative: why not (-1 an unverified firmware)
int reiKCtrlCpu3(int on, unsigned int *out);
#endif
