/*  The Taito module (rbx_module.h): its boards' drivers and game tables, and the game list's tabs by board generation.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <stdlib.h>
#include <string.h>

#include "gen/taito_b.inc"
#include "gen/taito_x.inc"
#include "gen/taito_o.inc"
#include "gen/taito_h.inc"
#include "gen/taito_l.inc"
#include "gen/si8080.inc"
#include "gen/rastan.inc"
#include "gen/rbisland.inc"
#include "gen/opwolf.inc"
#include "gen/arkanoid.inc"
#include "gen/bublbobl.inc"
#include "gen/tnzs.inc"
#include "gen/taito_f2.inc"
#include "gen/taito_z.inc"
#include "gen/qix.inc"
#include "gen/asuka.inc"
#include "gen/volfied.inc"
#include "gen/darius.inc"
#include "gen/ninjaw.inc"
#include "gen/warriorb.inc"
#include "gen/taito_f3.inc"
#include "gen/othunder.inc"
#include "gen/topspeed.inc"
#include "gen/slapshot.inc"
#include "gen/wgp.inc"
#include "gen/halleys.inc"
#include "gen/tsamurai.inc"
#include "gen/gladiatr.inc"
#include "gen/superqix.inc"
#include "gen/exzisus.inc"
#include "gen/roundup.inc"
#include "gen/gsword.inc"
#include "gen/cyclemb.inc"
#include "gen/spdheat.inc"
#include "gen/jollyjgr.inc"
#include "gen/suprridr.inc"
#include "gen/rollrace.inc"
#include "gen/ksayakyu.inc"
#include "gen/ssrj.inc"
#include "gen/minivadr.inc"
#include "gen/marinedt.inc"
#include "gen/wyvernf0.inc"
#include "gen/bigevglf.inc"
#include "gen/changela.inc"
#include "gen/pitnrun.inc"
#include "gen/crbaloon.inc"
#include "gen/sspeedr.inc"
#include "gen/fgoal.inc"
#include "gen/grchamp.inc"
#include "gen/sbowling.inc"
#include "gen/taitosj.inc"
#include "gen/lkage.inc"
#include "gen/flstory.inc"
#include "gen/retofinv.inc"
#include "gen/kikikai.inc"
#include "gen/msisaac.inc"
#include "gen/nycaptor.inc"
#include "gen/fortylove.inc"
#include "gen/bking.inc"
#include "gen/chaknpop.inc"
#include "gen/buggychl.inc"
#include "gen/lsasquad.inc"

extern const rbx_driver rbx_drv_taito_b;
extern const rbx_driver rbx_drv_taito_x;
extern const rbx_driver rbx_drv_taito_o;
extern const rbx_driver rbx_drv_taito_h;
extern const rbx_driver rbx_drv_taito_l;
extern const rbx_driver rbx_drv_si8080;
extern const rbx_driver rbx_drv_rastan;
extern const rbx_driver rbx_drv_rbisland;
extern const rbx_driver rbx_drv_opwolf;
extern const rbx_driver rbx_drv_arkanoid;
extern const rbx_driver rbx_drv_bublbobl;
extern const rbx_driver rbx_drv_tnzs;
extern const rbx_driver rbx_drv_taito_f2;
extern const rbx_driver rbx_drv_taito_z;
extern const rbx_driver rbx_drv_qix;
extern const rbx_driver rbx_drv_asuka;
extern const rbx_driver rbx_drv_volfied;
extern const rbx_driver rbx_drv_darius;
extern const rbx_driver rbx_drv_ninjaw;
extern const rbx_driver rbx_drv_warriorb;
extern const rbx_driver rbx_drv_taito_f3;
extern const rbx_driver rbx_drv_othunder;
extern const rbx_driver rbx_drv_topspeed;
extern const rbx_driver rbx_drv_slapshot;
extern const rbx_driver rbx_drv_wgp;
extern const rbx_driver rbx_drv_halleys;
extern const rbx_driver rbx_drv_tsamurai;
extern const rbx_driver rbx_drv_gladiatr;
extern const rbx_driver rbx_drv_superqix;
extern const rbx_driver rbx_drv_exzisus;
extern const rbx_driver rbx_drv_roundup;
extern const rbx_driver rbx_drv_gsword;
extern const rbx_driver rbx_drv_cyclemb;
extern const rbx_driver rbx_drv_spdheat;
extern const rbx_driver rbx_drv_jollyjgr;
extern const rbx_driver rbx_drv_suprridr;
extern const rbx_driver rbx_drv_rollrace;
extern const rbx_driver rbx_drv_ksayakyu;
extern const rbx_driver rbx_drv_ssrj;
extern const rbx_driver rbx_drv_minivadr;
extern const rbx_driver rbx_drv_marinedt;
extern const rbx_driver rbx_drv_wyvernf0;
extern const rbx_driver rbx_drv_bigevglf;
extern const rbx_driver rbx_drv_changela;
extern const rbx_driver rbx_drv_pitnrun;
extern const rbx_driver rbx_drv_crbaloon;
extern const rbx_driver rbx_drv_sspeedr;
extern const rbx_driver rbx_drv_fgoal;
extern const rbx_driver rbx_drv_grchamp;
extern const rbx_driver rbx_drv_sbowling;
extern const rbx_driver rbx_drv_taitosj;
extern const rbx_driver rbx_drv_lkage;
extern const rbx_driver rbx_drv_flstory;
extern const rbx_driver rbx_drv_retofinv;
extern const rbx_driver rbx_drv_kikikai;
extern const rbx_driver rbx_drv_msisaac;
extern const rbx_driver rbx_drv_nycaptor;
extern const rbx_driver rbx_drv_fortylove;
extern const rbx_driver rbx_drv_bking;
extern const rbx_driver rbx_drv_chaknpop;
extern const rbx_driver rbx_drv_buggychl;
extern const rbx_driver rbx_drv_lsasquad;

static const rbx_board_tab s_boards[] = {
	{ taito_b_games, TAITO_B_GAME_COUNT, &rbx_drv_taito_b },
	{ taito_x_games, TAITO_X_GAME_COUNT, &rbx_drv_taito_x },
	{ taito_o_games, TAITO_O_GAME_COUNT, &rbx_drv_taito_o },
	{ taito_h_games, TAITO_H_GAME_COUNT, &rbx_drv_taito_h },
	{ taito_l_games, TAITO_L_GAME_COUNT, &rbx_drv_taito_l },
	{ si8080_games, SI8080_GAME_COUNT, &rbx_drv_si8080 },
	{ rastan_games, RASTAN_GAME_COUNT, &rbx_drv_rastan },
	{ rbisland_games, RBISLAND_GAME_COUNT, &rbx_drv_rbisland },
	{ opwolf_games, OPWOLF_GAME_COUNT, &rbx_drv_opwolf },
	{ arkanoid_games, ARKANOID_GAME_COUNT, &rbx_drv_arkanoid },
	{ bublbobl_games, BUBLBOBL_GAME_COUNT, &rbx_drv_bublbobl },
	{ tnzs_games, TNZS_GAME_COUNT, &rbx_drv_tnzs },
	{ taito_f2_games, TAITO_F2_GAME_COUNT, &rbx_drv_taito_f2 },
	{ taito_z_games, TAITO_Z_GAME_COUNT, &rbx_drv_taito_z },
	{ qix_games, QIX_GAME_COUNT, &rbx_drv_qix },
	{ asuka_games, ASUKA_GAME_COUNT, &rbx_drv_asuka },
	{ volfied_games, VOLFIED_GAME_COUNT, &rbx_drv_volfied },
	{ darius_games, DARIUS_GAME_COUNT, &rbx_drv_darius },
	{ ninjaw_games, NINJAW_GAME_COUNT, &rbx_drv_ninjaw },
	{ warriorb_games, WARRIORB_GAME_COUNT, &rbx_drv_warriorb },
	{ taito_f3_games, TAITO_F3_GAME_COUNT, &rbx_drv_taito_f3 },
	{ othunder_games, OTHUNDER_GAME_COUNT, &rbx_drv_othunder },
	{ topspeed_games, TOPSPEED_GAME_COUNT, &rbx_drv_topspeed },
	{ slapshot_games, SLAPSHOT_GAME_COUNT, &rbx_drv_slapshot },
	{ wgp_games, WGP_GAME_COUNT, &rbx_drv_wgp },
	{ halleys_games, HALLEYS_GAME_COUNT, &rbx_drv_halleys },
	{ tsamurai_games, TSAMURAI_GAME_COUNT, &rbx_drv_tsamurai },
	{ gladiatr_games, GLADIATR_GAME_COUNT, &rbx_drv_gladiatr },
	{ superqix_games, SUPERQIX_GAME_COUNT, &rbx_drv_superqix },
	{ exzisus_games, EXZISUS_GAME_COUNT, &rbx_drv_exzisus },
	{ roundup_games, ROUNDUP_GAME_COUNT, &rbx_drv_roundup },
	{ gsword_games, GSWORD_GAME_COUNT, &rbx_drv_gsword },
	{ cyclemb_games, CYCLEMB_GAME_COUNT, &rbx_drv_cyclemb },
	{ spdheat_games, SPDHEAT_GAME_COUNT, &rbx_drv_spdheat },
	{ jollyjgr_games, JOLLYJGR_GAME_COUNT, &rbx_drv_jollyjgr },
	{ suprridr_games, SUPRRIDR_GAME_COUNT, &rbx_drv_suprridr },
	{ rollrace_games, ROLLRACE_GAME_COUNT, &rbx_drv_rollrace },
	{ ksayakyu_games, KSAYAKYU_GAME_COUNT, &rbx_drv_ksayakyu },
	{ ssrj_games, SSRJ_GAME_COUNT, &rbx_drv_ssrj },
	{ minivadr_games, MINIVADR_GAME_COUNT, &rbx_drv_minivadr },
	{ marinedt_games, MARINEDT_GAME_COUNT, &rbx_drv_marinedt },
	{ wyvernf0_games, WYVERNF0_GAME_COUNT, &rbx_drv_wyvernf0 },
	{ bigevglf_games, BIGEVGLF_GAME_COUNT, &rbx_drv_bigevglf },
	{ changela_games, CHANGELA_GAME_COUNT, &rbx_drv_changela },
	{ pitnrun_games, PITNRUN_GAME_COUNT, &rbx_drv_pitnrun },
	{ crbaloon_games, CRBALOON_GAME_COUNT, &rbx_drv_crbaloon },
	{ sspeedr_games, SSPEEDR_GAME_COUNT, &rbx_drv_sspeedr },
	{ fgoal_games, FGOAL_GAME_COUNT, &rbx_drv_fgoal },
	{ grchamp_games, GRCHAMP_GAME_COUNT, &rbx_drv_grchamp },
	{ sbowling_games, SBOWLING_GAME_COUNT, &rbx_drv_sbowling },
	{ taitosj_games, TAITOSJ_GAME_COUNT, &rbx_drv_taitosj },
	{ lkage_games, LKAGE_GAME_COUNT, &rbx_drv_lkage },
	{ flstory_games, FLSTORY_GAME_COUNT, &rbx_drv_flstory },
	{ retofinv_games, RETOFINV_GAME_COUNT, &rbx_drv_retofinv },
	{ kikikai_games, KIKIKAI_GAME_COUNT, &rbx_drv_kikikai },
	{ msisaac_games, MSISAAC_GAME_COUNT, &rbx_drv_msisaac },
	{ nycaptor_games, NYCAPTOR_GAME_COUNT, &rbx_drv_nycaptor },
	{ fortylove_games, FORTYLOVE_GAME_COUNT, &rbx_drv_fortylove },
	{ bking_games, BKING_GAME_COUNT, &rbx_drv_bking },
	{ chaknpop_games, CHAKNPOP_GAME_COUNT, &rbx_drv_chaknpop },
	{ buggychl_games, BUGGYCHL_GAME_COUNT, &rbx_drv_buggychl },
	{ lsasquad_games, LSASQUAD_GAME_COUNT, &rbx_drv_lsasquad },
};

// the game list's systems: each board system its own, the single-game boards by year
static const char *const s_groups[] = { "B System", "O System", "X System", "H System", "L System", "Z System",
	"F2 System", "F3 System", "SJ System", "Classic", "Standalone" };

static int group(const rbx_game *g)
{
	static const char *const sys[] = { "Taito B System", "Taito O System", "Taito X System", "Taito H System",
		"Taito L System", "Taito Z System", "Taito F2 System", "Taito F3 System", "Taito SJ System" };
	for (int i = 0; i < (int)(sizeof(sys) / sizeof(sys[0])); i++)
		if (!strcmp(sys[i], g->board))
			return i;
	const int year = atoi(g->year);   // (single-game boards: up to 1985 Classic, later Standalone)
	return year && year <= 1985 ? 9 : 10;
}

static int board_order(const char *board)
{
	// the board systems first, then the single-game boards; unknown boards last
	static const char *const order[] = { "Taito B System", "Taito O System", "Taito X System", "Taito F2 System", "Taito F3 System", "Taito H System", "Taito L System", "Taito Z System", "Rastan", "Rainbow Islands", "Operation Wolf", "Space Invaders (8080)", "Arkanoid", "Bubble Bobble", "Taito TNZS", "Qix", "Asuka & Asuka", "Volfied", "Darius", "Ninja Warriors", "Warrior Blade", "Operation Thunderbolt", "Top Speed", "Slap Shot", "World Grand Prix", "Halley's Comet", "Samurai Nihon-Ichi", "Gladiator", "Super Qix", "Exzisus", "Round-Up / The Pit", "Great Swordsman", "Cycle Maabou", "Super Dead Heat", "Jolly Jogger", "Super Rider", "Fighting Roller", "Kusayakyuu", "Super Speed Race Jr", "Mini Vaders", "Marine Date", "Wyvern F-0", "Big Event Golf", "Change Lanes", "Pit & Run", "Crazy Balloon", "Super Speed Race", "Field Goal", "Grand Champion", "Strike Bowling", "Taito SJ System", "The Legend of Kage", "The FairyLand Story", "Return of the Invaders", "KiKi KaiKai", "Metal Soldier Isaac II", "N.Y. Captor", "Forty-Love", "Birdie King", "Chack'n Pop", "Buggy Challenge", "Land Sea Air Squad" };
	const int n = (int)(sizeof(order) / sizeof(order[0]));
	const int single = strstr(board, "System") == NULL;
	for (int i = 0; i < n; i++)
		if (!strcmp(order[i], board))
			return single * n + i;
	return 1000 + single * n;
}

const rbx_module rbx_mod_taito = {
	"taito", "Taito",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
