/*  Neo Geo hacks added on request (issue #17, ADR-232): not MAME sets. Each table is written here by hand from the hack's own
    zip (its file names, sizes and CRCs), in the layout of gen/neogeo.inc; no other emulator's source or set list is used.
    The BIOS files come from the parent chain's zips (kof98.zip or neogeo.zip), as for MAME's split sets.
    This file is part of ReBOX. GPL-2.0-or-later.
*/

// The King of Fighters '98 Anniversary Edition: an unencrypted P ROM (no kof98 protection: the neobase cartridge), 6 MB
// of P2 (banked), a fifth V ROM and twelve C ROMs (96 MB of sprites)
static const rbx_rom_load neogeo_roml_kof98ae[] = {
	{ RBX_ROM_LOAD, 0, "242-p1ae.bin", 0x0, 0x100000, 0x23a80b3e, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 0, "242-p2ae.bin", 0x100000, 0x600000, 0x99b3e5cc, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 1, "242-s1ae.bin", 0x0, 0x20000, 0xf1fee5c0, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 2, "sfix.sfix", 0x0, 0x20000, 0xc2ea0cfd, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 3, "000-lo.lo", 0x0, 0x20000, 0x5a86cff2, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 4, "sp-s2.sp1", 0x0, 0x20000, 0x9036d879, 2, 0, 1, 0 },
	{ RBX_ROM_LOAD, 5, "sm1.sm1", 0x0, 0x20000, 0x94416d67, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 6, "242-m1ae.bin", 0x0, 0x40000, 0x9ade0528, 1, 0, 0, 0 },
	{ RBX_ROM_RELOAD, 6, NULL, 0x10000, 0x40000, 0, 0, 0, 0, 0 },
	{ RBX_ROM_LOAD, 7, "242-v1.bin", 0x0, 0x400000, 0xb9ea8051, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 7, "242-v2.bin", 0x400000, 0x400000, 0xcc11106e, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 7, "242-v3.bin", 0x800000, 0x400000, 0x044ea4e1, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 7, "242-v4.bin", 0xc00000, 0x400000, 0x7985ea30, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 7, "242-v5ae.bin", 0x1000000, 0x400000, 0xafdd9660, 1, 0, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c1ae.bin", 0x0, 0x800000, 0x0319cfc9, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c2ae.bin", 0x1, 0x800000, 0x553f6714, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c3.bin", 0x1000000, 0x800000, 0x22127b4f, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c4.bin", 0x1000001, 0x800000, 0x0b4fa044, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c5ae.bin", 0x2000000, 0x800000, 0x71641718, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c6ae.bin", 0x2000001, 0x800000, 0x982ba2b3, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c7ae.bin", 0x3000000, 0x800000, 0x8d495552, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c8ae.bin", 0x3000001, 0x800000, 0x8bfc3417, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c9ae.bin", 0x4000000, 0x800000, 0x128256d1, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c91ae.bin", 0x4000001, 0x800000, 0x4bd8412d, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c92ae.bin", 0x5000000, 0x800000, 0xe893277f, 1, 1, 0, 0 },
	{ RBX_ROM_LOAD, 8, "242-c93ae.bin", 0x5000001, 0x800000, 0xf165589e, 1, 1, 0, 0 },
	{ RBX_ROM_END, 0, NULL, 0, 0, 0, 0, 0, 0, 0 }
};
static const rbx_rom_region neogeo_romr_kof98ae[] = {
	{ "cslot1:maincpu", 0x700000, 0x00 },
	{ "cslot1:fixed", 0x20000, 0x00 },
	{ "fixedbios", 0x20000, 0x00 },
	{ "spritegen:zoomy", 0x20000, 0x00 },
	{ "mainbios", 0x80000, 0x00 },
	{ "audiobios", 0x20000, 0x00 },
	{ "cslot1:audiocpu", 0x50000, 0x00 },
	{ "cslot1:ymsnd:adpcma", 0x1400000, 0x00 },
	{ "cslot1:sprites", 0x6000000, 0x00 },
	{ NULL, 0, 0 }
};

static const rbx_game neogeo_hack_games[] = {
	{ "kof98ae", "kof98", "The King of Fighters '98 Anniversary Edition (hack)", "1998", "hack", "SNK Neo Geo", "neobase", 0, 0, neogeo_romr_kof98ae, neogeo_roml_kof98ae, neogeo_ipp_neogeo, neogeo_ipf_neogeo, neogeo_ips_neogeo },
};
#define NEOGEO_HACK_GAME_COUNT 1
