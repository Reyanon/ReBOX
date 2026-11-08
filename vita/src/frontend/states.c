#include "frontend/states.h"

#include <stdio.h>
#include <string.h>

#include <psp2/io/dirent.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>

#include "frontend/core_api.h"

#include "frontend/settings.h"

#define STATE_DIR RBX_DATA_DIR "states/"

// the save states in states/; older ones, in the data folder itself, moved there
void rbx_states_move_old(void)
{
	sceIoMkdir(STATE_DIR, 0777);
	const SceUID d = sceIoDopen(RBX_DATA_DIR);
	if (d < 0)
		return;
	SceIoDirent de;
	for (;;) {
		memset(&de, 0, sizeof(de));
		if (sceIoDread(d, &de) <= 0)
			break;
		const size_t l = strlen(de.d_name);
		if (l < 5 || strcmp(de.d_name + l - 4, ".sav") || SCE_S_ISDIR(de.d_stat.st_mode))
			continue;
		char from[128], to[128];
		snprintf(from, sizeof(from), RBX_DATA_DIR "%s", de.d_name);
		snprintf(to, sizeof(to), STATE_DIR "%s", de.d_name);
		sceIoRename(from, to);
	}
	sceIoDclose(d);
}

void rbx_state_path(int slot, char *buf, int len)
{
	if (slot == RBX_STATE_QUICK)
		snprintf(buf, len, STATE_DIR "%s_quick.sav", rbx_core_set_name());
	else
		snprintf(buf, len, STATE_DIR "%s_%d.sav", rbx_core_set_name(), slot);
}

int rbx_state_exists(int slot)
{
	char path[128];
	rbx_state_path(slot, path, sizeof(path));
	return rbx_core_state_exists(path);
}

const char *rbx_state_slot_name(int slot)
{
	static char buf[16];
	if (slot == RBX_STATE_QUICK)
		return "quick";
	snprintf(buf, sizeof(buf), "slot %d", slot);
	return buf;
}
