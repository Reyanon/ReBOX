// ReBOX installer: promotes the unpacked package in ux0:data/rbox_pkg (VPK files + sce_sys/package/head.bin made on the
// PC by vita/tools/vita.py) to an installed app, then exits. Run once from any registered app slot (vita.py install) for
// the first install; the steps follow VitaShell package_installer.c promoteApp (GPL-3.0, TheFloW).
// This file is part of ReBOX. GPL-2.0-or-later.
#include <psp2/kernel/processmgr.h>
#include <psp2/promoterutil.h>
#include <psp2/sysmodule.h>
#include <stdio.h>
#include <string.h>

#define PKG_DIR "ux0:data/rbox_pkg"
#define LOG "ux0:data/rbox_inst.txt"

static void note(const char *what, int rc)
{
	FILE *f = fopen(LOG, "a");
	if (f) {
		fprintf(f, "%s %08x\n", what, (unsigned)rc);
		fclose(f);
	}
}

static int load_paf(void)
{
	static uint32_t argp[] = { 0x180000, (uint32_t)-1, (uint32_t)-1, 1, (uint32_t)-1, (uint32_t)-1 };
	int result = -1;
	uint32_t buf[4];
	buf[0] = sizeof(buf);
	buf[1] = (uint32_t)&result;
	buf[2] = (uint32_t)-1;
	buf[3] = (uint32_t)-1;
	return sceSysmoduleLoadModuleInternalWithArg(SCE_SYSMODULE_INTERNAL_PAF, sizeof(argp), argp, (SceSysmoduleOpt *)buf);
}

int main(void)
{
	int rc = load_paf();
	note("paf", rc);
	rc = sceSysmoduleLoadModuleInternal(SCE_SYSMODULE_INTERNAL_PROMOTER_UTIL);
	note("promoter_module", rc);
	rc = scePromoterUtilityInit();
	note("init", rc);
	rc = scePromoterUtilityPromotePkgWithRif(PKG_DIR, 1);
	note("promote", rc);
	int res = 0;
	int r2 = scePromoterUtilityGetResult(&res);
	note("result_call", r2);
	note("result", res);
	rc = scePromoterUtilityExit();
	note("exit", rc);
	note("done", 0);
	sceKernelExitProcess(0);
	return 0;
}
