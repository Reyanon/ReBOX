// CPS-2 WIDE v1 sets, written by hand (tools/gen_games.py makes cps2.inc from MAME's own cps2.cpp, which has none):
// from the Vampire Saved project's MAME patch emu/mame-patches/0002-cps2-wide-v1.patch
// (https://github.com/DefinitelyFrenchName/VampireSaved, GPL-3.0; a patch to MAME's src/mame/capcom/cps2.cpp,
// license BSD-3-Clause, copyright-holders Paul Leaman, Andreas Naive, Nicola Salmoria, Charles MacDonald).
// vsavjw: vsavj on the WIDE profile (program 6 MB, the part above 0x0fffff plain; gfx 48 MB with a 19-bit sprite
// tile address; QSound samples 16 MB). The vsw.* members carry sentinel CRCs (the project's builds patch them):
// the loader finds them by name.
// This file is part of ReBOX. GPL-2.0-or-later.

static const rbx_rom_load cps2_roml_vsavjw[] = {
	{ RBX_ROM_LOAD, 0, "vm3j.03d", 0x0, 0x80000, 0x2a2e74a4, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vm3j.04d", 0x80000, 0x80000, 0x1c2427bc, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vm3j.05a", 0x100000, 0x80000, 0x95ce88d5, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vm3j.06b", 0x180000, 0x80000, 0x2c4297e0, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vm3j.07b", 0x200000, 0x80000, 0xa38aaae7, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vm3j.08a", 0x280000, 0x80000, 0x5773e5c9, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vm3j.09b", 0x300000, 0x80000, 0xd064f8b9, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vm3j.10b", 0x380000, 0x80000, 0x434518e9, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vsw.41", 0x400000, 0x80000, 0xdec0de41, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vsw.42", 0x480000, 0x80000, 0xdec0de42, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vsw.43", 0x500000, 0x80000, 0xdec0de43, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "vsw.44", 0x580000, 0x80000, 0xdec0de44, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 1, "vm3.13m", 0x0, 0x400000, 0xfd8a11eb, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vm3.15m", 0x2, 0x400000, 0xdd1e7d4e, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vm3.17m", 0x4, 0x400000, 0x6b89445e, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vm3.19m", 0x6, 0x400000, 0x3830fdc7, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vm3.14m", 0x1000000, 0x400000, 0xc1a28e6c, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vm3.16m", 0x1000002, 0x400000, 0x194a7304, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vm3.18m", 0x1000004, 0x400000, 0xdf9a9f47, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vm3.20m", 0x1000006, 0x400000, 0xc22fc3d9, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vsw.31m", 0x2000000, 0x400000, 0xdec0de31, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vsw.33m", 0x2000002, 0x400000, 0xdec0de33, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vsw.35m", 0x2000004, 0x400000, 0xdec0de35, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 1, "vsw.37m", 0x2000006, 0x400000, 0xdec0de37, 2, 6, 0, 0 },
	{ RBX_ROM_LOAD, 2, "vsw.z01", 0x0, 0x8000, 0xdec0de38, 1, 0, 0, 0 },
	{ RBX_ROM_CONTINUE, 2, NULL, 0x10000, 0x18000, 0, 0, 0, 0, 0 },
	{ RBX_ROM_LOAD, 2, "vsw.z02", 0x28000, 0x20000, 0xdec0de39, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 3, "vm3.11m", 0x0, 0x400000, 0xe80e956e, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 3, "vm3.12m", 0x400000, 0x400000, 0x9cd71557, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 3, "vsw.21m", 0x800000, 0x400000, 0xdec0de3a, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 3, "vsw.22m", 0xc00000, 0x400000, 0xdec0de3b, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 4, "vsavj.key", 0x0, 0x14, 0x36d28ab8, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 5, "dl-1425.bin", 0x0, 0x2000, 0xd6cf5ef5, 1, 0, 0, 0 },
	{ RBX_ROM_END, 0, NULL, 0, 0, 0, 0, 0, 0, 0 }
};
static const rbx_rom_region cps2_romr_vsavjw[] = {
	{ "maincpu", 0x600000, 0x00 },
	{ "gfx", 0x3000000, 0x00 },
	{ "audiocpu", 0x50000, 0x00 },
	{ "qsound", 0x1000000, 0x00 },
	{ "key", 0x20, 0x00 },
	{ "qsound:dsp", 0x2000, 0x00 },
	{ NULL, 0, 0 }
};
static const rbx_game cps2_wide_games[] = {
	{ "vsavjw", "vsav", "Vampire Saved (Vampire Savior Japan 970519, CPS-2 WIDE v1)", "1997", "Capcom / Vampire Saved",
	  "Capcom CPS2", "cps2wide", 0, 0, cps2_romr_vsavjw, cps2_roml_vsavjw, cps2_ipp_cps2_2p6b, cps2_ipf_cps2_2p6b,
	  cps2_ips_cps2_2p6b },
};
#define CPS2_WIDE_GAME_COUNT 1
