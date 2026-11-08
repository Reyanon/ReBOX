#ifndef RBX_FRONTEND_STATES_H
#define RBX_FRONTEND_STATES_H

// Save state slots: 0-9, quicksave (RBX_STATE_QUICK).
// File: ux0:data/rebox/<romset>_<slot>.sav (per set)

#define RBX_STATE_SLOTS 10
#define RBX_STATE_QUICK (-1)

void rbx_states_move_old(void);   // (at start: states/ made, older .sav files moved in)
void rbx_state_path(int slot, char *buf, int len);
int rbx_state_exists(int slot);
// "slot 3" / "quick"
const char *rbx_state_slot_name(int slot);

#endif
