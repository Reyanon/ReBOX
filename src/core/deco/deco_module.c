/*  The DECO module (rbx_module.h): Data East boards, their drivers and game tables, and the game list's systems.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/simpl156.inc"
#include "gen/hvysmsh.inc"
#include "gen/backfire.inc"
#include "gen/captaven.inc"
#include "gen/fghthist.inc"
#include "gen/dragngun.inc"
#include "gen/deco_mlc.inc"
#include "gen/supbtime.inc"
#include "gen/dietgo.inc"
#include "gen/funkyjet.inc"
#include "gen/pktgaldx.inc"
#include "gen/rohga.inc"
#include "gen/darkseal.inc"
#include "gen/vaportra.inc"
#include "gen/cbuster.inc"
#include "gen/cninja.inc"
#include "gen/boogwing.inc"
#include "gen/dassault.inc"
#include "gen/sshangha.inc"
#include "gen/dblewing.inc"
#include "gen/mirage.inc"
#include "gen/dreambal.inc"
#include "gen/lemmings.inc"
#include "gen/dec0.inc"
#include "gen/karnov.inc"
#include "gen/stadhero.inc"
#include "gen/madmotor.inc"
#include "gen/actfancr.inc"
#include "gen/dec8.inc"
#include "gen/brkthru.inc"
#include "gen/chanbara.inc"
#include "gen/compgolf.inc"
#include "gen/exprraid.inc"
#include "gen/metlclsh.inc"
#include "gen/shootout.inc"
#include "gen/sidepckt.inc"
#include "gen/tryout.inc"
#include "gen/firetrap.inc"
#include "gen/thedeep.inc"
#include "gen/pcktgal.inc"
#include "gen/tumbleb.inc"
#include "gen/btime.inc"
#include "gen/liberate.inc"
#include "gen/progolf.inc"
#include "gen/bwing.inc"
#include "gen/kchamp.inc"
#include "gen/kingofb.inc"
#include "gen/astrof.inc"
#include "gen/madalien.inc"
#include "gen/decocass.inc"
#include "gen/battlera.inc"

extern const rbx_driver rbx_drv_simpl156;
extern const rbx_driver rbx_drv_hvysmsh;
extern const rbx_driver rbx_drv_backfire;
extern const rbx_driver rbx_drv_captaven;
extern const rbx_driver rbx_drv_fghthist;
extern const rbx_driver rbx_drv_dragngun;
extern const rbx_driver rbx_drv_deco_mlc;
extern const rbx_driver rbx_drv_supbtime;
extern const rbx_driver rbx_drv_dietgo;
extern const rbx_driver rbx_drv_funkyjet;
extern const rbx_driver rbx_drv_pktgaldx;
extern const rbx_driver rbx_drv_rohga;
extern const rbx_driver rbx_drv_darkseal;
extern const rbx_driver rbx_drv_vaportra;
extern const rbx_driver rbx_drv_cbuster;
extern const rbx_driver rbx_drv_cninja;
extern const rbx_driver rbx_drv_boogwing;
extern const rbx_driver rbx_drv_dassault;
extern const rbx_driver rbx_drv_sshangha;
extern const rbx_driver rbx_drv_dblewing;
extern const rbx_driver rbx_drv_mirage;
extern const rbx_driver rbx_drv_dreambal;
extern const rbx_driver rbx_drv_lemmings;
extern const rbx_driver rbx_drv_dec0;
extern const rbx_driver rbx_drv_karnov;
extern const rbx_driver rbx_drv_stadhero;
extern const rbx_driver rbx_drv_madmotor;
extern const rbx_driver rbx_drv_actfancr;
extern const rbx_driver rbx_drv_dec8;
extern const rbx_driver rbx_drv_brkthru;
extern const rbx_driver rbx_drv_chanbara;
extern const rbx_driver rbx_drv_compgolf;
extern const rbx_driver rbx_drv_exprraid;
extern const rbx_driver rbx_drv_metlclsh;
extern const rbx_driver rbx_drv_shootout;
extern const rbx_driver rbx_drv_sidepckt;
extern const rbx_driver rbx_drv_tryout;
extern const rbx_driver rbx_drv_firetrap;
extern const rbx_driver rbx_drv_thedeep;
extern const rbx_driver rbx_drv_pcktgal;
extern const rbx_driver rbx_drv_tumbleb;
extern const rbx_driver rbx_drv_btime;
extern const rbx_driver rbx_drv_liberate;
extern const rbx_driver rbx_drv_progolf;
extern const rbx_driver rbx_drv_bwing;
extern const rbx_driver rbx_drv_kchamp;
extern const rbx_driver rbx_drv_kingofb;
extern const rbx_driver rbx_drv_astrof;
extern const rbx_driver rbx_drv_madalien;
extern const rbx_driver rbx_drv_decocass;
extern const rbx_driver rbx_drv_battlera;

static const rbx_board_tab s_boards[] = {
	{ simpl156_games, SIMPL156_GAME_COUNT, &rbx_drv_simpl156 },
	{ hvysmsh_games, HVYSMSH_GAME_COUNT, &rbx_drv_hvysmsh },
	{ backfire_games, BACKFIRE_GAME_COUNT, &rbx_drv_backfire },
	{ captaven_games, CAPTAVEN_GAME_COUNT, &rbx_drv_captaven },
	{ fghthist_games, FGHTHIST_GAME_COUNT, &rbx_drv_fghthist },
	{ dragngun_games, DRAGNGUN_GAME_COUNT, &rbx_drv_dragngun },
	{ deco_mlc_games, DECO_MLC_GAME_COUNT, &rbx_drv_deco_mlc },
	{ supbtime_games, SUPBTIME_GAME_COUNT, &rbx_drv_supbtime },
	{ dietgo_games, DIETGO_GAME_COUNT, &rbx_drv_dietgo },
	{ funkyjet_games, FUNKYJET_GAME_COUNT, &rbx_drv_funkyjet },
	{ pktgaldx_games, PKTGALDX_GAME_COUNT, &rbx_drv_pktgaldx },
	{ rohga_games, ROHGA_GAME_COUNT, &rbx_drv_rohga },
	{ darkseal_games, DARKSEAL_GAME_COUNT, &rbx_drv_darkseal },
	{ vaportra_games, VAPORTRA_GAME_COUNT, &rbx_drv_vaportra },
	{ cbuster_games, CBUSTER_GAME_COUNT, &rbx_drv_cbuster },
	{ cninja_games, CNINJA_GAME_COUNT, &rbx_drv_cninja },
	{ boogwing_games, BOOGWING_GAME_COUNT, &rbx_drv_boogwing },
	{ dassault_games, DASSAULT_GAME_COUNT, &rbx_drv_dassault },
	{ sshangha_games, SSHANGHA_GAME_COUNT, &rbx_drv_sshangha },
	{ dblewing_games, DBLEWING_GAME_COUNT, &rbx_drv_dblewing },
	{ mirage_games, MIRAGE_GAME_COUNT, &rbx_drv_mirage },
	{ dreambal_games, DREAMBAL_GAME_COUNT, &rbx_drv_dreambal },
	{ lemmings_games, LEMMINGS_GAME_COUNT, &rbx_drv_lemmings },
	{ dec0_games, DEC0_GAME_COUNT, &rbx_drv_dec0 },
	{ karnov_games, KARNOV_GAME_COUNT, &rbx_drv_karnov },
	{ stadhero_games, STADHERO_GAME_COUNT, &rbx_drv_stadhero },
	{ madmotor_games, MADMOTOR_GAME_COUNT, &rbx_drv_madmotor },
	{ actfancr_games, ACTFANCR_GAME_COUNT, &rbx_drv_actfancr },
	{ dec8_games, DEC8_GAME_COUNT, &rbx_drv_dec8 },
	{ brkthru_games, BRKTHRU_GAME_COUNT, &rbx_drv_brkthru },
	{ chanbara_games, CHANBARA_GAME_COUNT, &rbx_drv_chanbara },
	{ compgolf_games, COMPGOLF_GAME_COUNT, &rbx_drv_compgolf },
	{ exprraid_games, EXPRRAID_GAME_COUNT, &rbx_drv_exprraid },
	{ metlclsh_games, METLCLSH_GAME_COUNT, &rbx_drv_metlclsh },
	{ shootout_games, SHOOTOUT_GAME_COUNT, &rbx_drv_shootout },
	{ sidepckt_games, SIDEPCKT_GAME_COUNT, &rbx_drv_sidepckt },
	{ tryout_games, TRYOUT_GAME_COUNT, &rbx_drv_tryout },
	{ firetrap_games, FIRETRAP_GAME_COUNT, &rbx_drv_firetrap },
	{ thedeep_games, THEDEEP_GAME_COUNT, &rbx_drv_thedeep },
	{ pcktgal_games, PCKTGAL_GAME_COUNT, &rbx_drv_pcktgal },
	{ tumbleb_games, TUMBLEB_GAME_COUNT, &rbx_drv_tumbleb },
	{ btime_games, BTIME_GAME_COUNT, &rbx_drv_btime },
	{ liberate_games, LIBERATE_GAME_COUNT, &rbx_drv_liberate },
	{ progolf_games, PROGOLF_GAME_COUNT, &rbx_drv_progolf },
	{ bwing_games, BWING_GAME_COUNT, &rbx_drv_bwing },
	{ kchamp_games, KCHAMP_GAME_COUNT, &rbx_drv_kchamp },
	{ kingofb_games, KINGOFB_GAME_COUNT, &rbx_drv_kingofb },
	{ astrof_games, ASTROF_GAME_COUNT, &rbx_drv_astrof },
	{ madalien_games, MADALIEN_GAME_COUNT, &rbx_drv_madalien },
	{ decocass_games, DECOCASS_GAME_COUNT, &rbx_drv_decocass },
	{ battlera_games, BATTLERA_GAME_COUNT, &rbx_drv_battlera },
};

// the game list's systems (the frontend orders them by year): Data East's own system names (DECO 156, DECO 32, MLC,
// DECO 16), then the older boards by generation (dec0's MEC-M1 era, the 8-bit boards, the first ones) and the Cassette
// System (2026-11-04, user: DECO 156 at the bottom — the older boards had fallen into its tab)
static const char *const s_groups[] = { "DECO 156", "DECO 32", "DECO MLC", "DECO 16", "16-bit", "8-bit", "Classic",
                                        "DECO Cassette" };

static int group(const rbx_game *g)
{
	static const char *const b16[] = { "DECO dec0", "DECO Karnov", "DECO Stadium Hero", "DECO Mad Motor", "DECO Act-Fancer",
	                                   "DECO Battle Rangers" };
	static const char *const b8[] = { "DECO dec8", "DECO Break Thru", "DECO Chanbara", "DECO Competition Golf",
	                                  "DECO Express Raider", "DECO Metal Clash", "DECO Shoot Out", "DECO Side Pocket",
	                                  "DECO Tryout", "DECO Fire Trap", "DECO The Deep", "DECO Pocket Gal" };
	const char *bd = g->board;
	if (strstr(bd, "156"))
		return 0;
	if (!strncmp(bd, "DECO 32", 7))
		return 1;
	if (!strcmp(bd, "DECO MLC"))
		return 2;
	if (!strncmp(bd, "DECO 16", 7) || !strcmp(bd, "DECO Tumble Pop bootlegs"))
		return 3;
	for (int i = 0; i < (int)(sizeof(b16) / sizeof(b16[0])); i++)
		if (!strcmp(bd, b16[i]))
			return 4;
	for (int i = 0; i < (int)(sizeof(b8) / sizeof(b8[0])); i++)
		if (!strcmp(bd, b8[i]))
			return 5;
	if (!strcmp(bd, "DECO Cassette System"))
		return 7;
	return 6;
}

static int board_order(const char *board)
{
	static const char *const order[] = { "DECO Simple 156", "DECO 156 Heavy Smash", "DECO 156 Backfire", "DECO 32 Captain America", "DECO 32 Fighter's History", "DECO 32 Dragon Gun", "DECO MLC", "DECO 16 Tumble Pop", "DECO 16 Diet Go Go", "DECO 16 Funky Jet", "DECO 16 Pocket Gal Deluxe", "DECO 16 Rohga", "DECO 16 Dark Seal", "DECO 16 Vapor Trail", "DECO 16 Crude Buster", "DECO 16 Caveman Ninja", "DECO 16 Boogie Wings", "DECO 16 Thunder Zone", "DECO 16 Super Shanghai Dragon's Eye", "DECO 16 Double Wings", "DECO 16 Mirage Youjuu Mahjongden", "DECO 16 Dream Ball", "DECO 16 Lemmings", "DECO dec0", "DECO Karnov", "DECO Stadium Hero", "DECO Mad Motor", "DECO Act-Fancer", "DECO Battle Rangers", "DECO dec8", "DECO Break Thru", "DECO Chanbara", "DECO Competition Golf", "DECO Express Raider", "DECO Metal Clash", "DECO Shoot Out", "DECO Side Pocket", "DECO Tryout", "DECO Fire Trap", "DECO The Deep", "DECO Pocket Gal", "DECO Tumble Pop bootlegs", "DECO Burger Time", "DECO Liberation", "DECO Pro Golf", "DECO B-Wings", "DECO Karate Champ", "DECO King of Boxer", "DECO Astro Fighter", "DECO Mad Alien", "DECO Cassette System" };
	for (int i = 0; i < (int)(sizeof(order) / sizeof(order[0])); i++)
		if (!strcmp(order[i], board))
			return i;
	return 1000;
}

const rbx_module rbx_mod_deco = {
	"deco", "Data East",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
