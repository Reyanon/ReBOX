/*  The Konami module (rbx_module.h): Konami boards, their drivers and game tables, and the game list's systems.
    Every Konami driver of MAME is listed (ADR-022); the boards not ported yet run with rbx_drv_konami_none (shown
    unplayable). This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../rbx_module.h"

#include <string.h>

#include "gen/konamigx.inc"
#include "gen/plygonet.inc"
#include "gen/djmain.inc"
#include "gen/tasman.inc"
#include "gen/mystwarr.inc"
#include "gen/moo.inc"
#include "gen/xexex.inc"
#include "gen/gijoe.inc"
#include "gen/dbz.inc"
#include "gen/rungun.inc"
#include "gen/asterix.inc"
#include "gen/bishi.inc"
#include "gen/qdrmfgp.inc"
#include "gen/piratesh.inc"
#include "gen/xmen.inc"
#include "gen/tmnt2.inc"
#include "gen/goldenregion.inc"
#include "gen/tmnt.inc"
#include "gen/twin16.inc"
#include "gen/nemesis.inc"
#include "gen/gradius3.inc"
#include "gen/wecleman.inc"
#include "gen/ultraman.inc"
#include "gen/overdriv.inc"
#include "gen/lethal.inc"
#include "gen/vendetta.inc"
#include "gen/simpsons.inc"
#include "gen/parodius.inc"
#include "gen/rollerg.inc"
#include "gen/surpratk.inc"
#include "gen/k88games.inc"
#include "gen/aliens.inc"
#include "gen/crimfght.inc"
#include "gen/thunderx.inc"
#include "gen/chqflag.inc"
#include "gen/blockhl.inc"
#include "gen/hcastle.inc"
#include "gen/ajax.inc"
#include "gen/mainevt.inc"
#include "gen/contra.inc"
#include "gen/combatsc.inc"
#include "gen/labyrunr.inc"
#include "gen/bladestl.inc"
#include "gen/battlnts.inc"
#include "gen/bottom9.inc"
#include "gen/flkatck.inc"
#include "gen/fastlane.inc"
#include "gen/rockrage.inc"
#include "gen/spy.inc"
#include "gen/ddribble.inc"
#include "gen/jackal.inc"
#include "gen/konblands.inc"
#include "gen/hexion.inc"
#include "gen/divebomb.inc"
#include "gen/ironhors.inc"
#include "gen/scotrsht.inc"
#include "gen/finalizr.inc"
#include "gen/megazone.inc"
#include "gen/jailbrek.inc"
#include "gen/rocnrope.inc"
#include "gen/pandoras.inc"
#include "gen/shaolins.inc"
#include "gen/yiear.inc"
#include "gen/mikie.inc"
#include "gen/trackfld.inc"
#include "gen/hyperspt.inc"
#include "gen/sbasketb.inc"
#include "gen/circusc.inc"
#include "gen/pingpong.inc"
#include "gen/tp84.inc"
#include "gen/gberet.inc"
#include "gen/gyruss.inc"
#include "gen/pooyan.inc"
#include "gen/timeplt.inc"
#include "gen/tutankhm.inc"
#include "gen/junofrst.inc"
#include "gen/gryzor_ms.inc"
#include "gen/konmedal.inc"
#include "gen/konmedal68k.inc"
#include "gen/konmedal020.inc"
#include "gen/konmedalppc.inc"
#include "gen/kontest.inc"
#include "gen/kpontoon.inc"
#include "gen/mogura.inc"
#include "gen/quickpick5.inc"
#include "gen/tgtpanic.inc"
#include "gen/cougar.inc"
#include "gen/kongs470.inc"
#include "gen/konendev.inc"
#include "gen/stingnet.inc"
#include "gen/hornet.inc"
#include "gen/nwktr.inc"
#include "gen/zr107.inc"
#include "gen/gticlub.inc"
#include "gen/cobra.inc"
#include "gen/viper.inc"
#include "gen/konamim2.inc"
#include "gen/ultrsprt.inc"
#include "gen/firebeat.inc"
#include "gen/twinkle.inc"
#include "gen/ksys573.inc"
#include "gen/konamigv.inc"
#include "gen/konamigq.inc"
#include "gen/konamigs.inc"
#include "gen/kpython.inc"
#include "gen/kpython2.inc"

// a board not ported yet: its games in the list, unplayable (supported: none)
static int none_init(const rbx_game *g) { (void)g; return -1; }
static int none_supported(const char *machine) { (void)machine; return 0; }
static const rbx_driver rbx_drv_konami_none = { "Konami (not ported yet)", none_init, NULL, NULL, NULL, NULL, NULL,
                                               none_supported, NULL, NULL };
extern const rbx_driver rbx_drv_ajax;
extern const rbx_driver rbx_drv_aliens;
extern const rbx_driver rbx_drv_asterix;
extern const rbx_driver rbx_drv_battlnts;
extern const rbx_driver rbx_drv_bishi;
extern const rbx_driver rbx_drv_bladestl;
extern const rbx_driver rbx_drv_chqflag;
extern const rbx_driver rbx_drv_circusc;
extern const rbx_driver rbx_drv_combatsc;
extern const rbx_driver rbx_drv_contra;
extern const rbx_driver rbx_drv_dbz;
extern const rbx_driver rbx_drv_ddribble;
extern const rbx_driver rbx_drv_fastlane;
extern const rbx_driver rbx_drv_finalizr;
extern const rbx_driver rbx_drv_flkatck;
extern const rbx_driver rbx_drv_gberet;
extern const rbx_driver rbx_drv_gijoe;
extern const rbx_driver rbx_drv_gradius3;
extern const rbx_driver rbx_drv_gyruss;
extern const rbx_driver rbx_drv_hcastle;
extern const rbx_driver rbx_drv_hyperspt;
extern const rbx_driver rbx_drv_ironhors;
extern const rbx_driver rbx_drv_jackal;
extern const rbx_driver rbx_drv_jailbrek;
extern const rbx_driver rbx_drv_junofrst;
extern const rbx_driver rbx_drv_konamigx;
extern const rbx_driver rbx_drv_konmedal;
extern const rbx_driver rbx_drv_konmedal68k;
extern const rbx_driver rbx_drv_kontest;
extern const rbx_driver rbx_drv_labyrunr;
extern const rbx_driver rbx_drv_mainevt;
extern const rbx_driver rbx_drv_megazone;
extern const rbx_driver rbx_drv_mikie;
extern const rbx_driver rbx_drv_mogura;
extern const rbx_driver rbx_drv_moo;
extern const rbx_driver rbx_drv_mystwarr;
extern const rbx_driver rbx_drv_nemesis;
extern const rbx_driver rbx_drv_pandoras;
extern const rbx_driver rbx_drv_pingpong;
extern const rbx_driver rbx_drv_pooyan;
extern const rbx_driver rbx_drv_quickpick5;
extern const rbx_driver rbx_drv_rockrage;
extern const rbx_driver rbx_drv_rocnrope;
extern const rbx_driver rbx_drv_rungun;
extern const rbx_driver rbx_drv_sbasketb;
extern const rbx_driver rbx_drv_scotrsht;
extern const rbx_driver rbx_drv_shaolins;
extern const rbx_driver rbx_drv_simpsons;
extern const rbx_driver rbx_drv_tgtpanic;
extern const rbx_driver rbx_drv_timeplt;
extern const rbx_driver rbx_drv_tmnt;
extern const rbx_driver rbx_drv_tmnt2;
extern const rbx_driver rbx_drv_tp84;
extern const rbx_driver rbx_drv_trackfld;
extern const rbx_driver rbx_drv_tutankhm;
extern const rbx_driver rbx_drv_twin16;
extern const rbx_driver rbx_drv_ultraman;
extern const rbx_driver rbx_drv_wecleman;
extern const rbx_driver rbx_drv_xmen;
extern const rbx_driver rbx_drv_yiear;

static const rbx_board_tab s_boards[] = {
	{ konamigx_games, KONAMIGX_GAME_COUNT, &rbx_drv_konamigx },
	{ plygonet_games, PLYGONET_GAME_COUNT, &rbx_drv_konami_none },
	{ djmain_games, DJMAIN_GAME_COUNT, &rbx_drv_konami_none },
	{ tasman_games, TASMAN_GAME_COUNT, &rbx_drv_konami_none },
	{ mystwarr_games, MYSTWARR_GAME_COUNT, &rbx_drv_mystwarr },
	{ moo_games, MOO_GAME_COUNT, &rbx_drv_moo },
	{ xexex_games, XEXEX_GAME_COUNT, &rbx_drv_moo },
	{ gijoe_games, GIJOE_GAME_COUNT, &rbx_drv_gijoe },
	{ dbz_games, DBZ_GAME_COUNT, &rbx_drv_dbz },
	{ rungun_games, RUNGUN_GAME_COUNT, &rbx_drv_rungun },
	{ asterix_games, ASTERIX_GAME_COUNT, &rbx_drv_asterix },
	{ bishi_games, BISHI_GAME_COUNT, &rbx_drv_bishi },
	{ qdrmfgp_games, QDRMFGP_GAME_COUNT, &rbx_drv_konami_none },
	{ piratesh_games, PIRATESH_GAME_COUNT, &rbx_drv_konami_none },
	{ xmen_games, XMEN_GAME_COUNT, &rbx_drv_xmen },
	{ tmnt2_games, TMNT2_GAME_COUNT, &rbx_drv_tmnt2 },
	{ goldenregion_games, GOLDENREGION_GAME_COUNT, &rbx_drv_konami_none },
	{ tmnt_games, TMNT_GAME_COUNT, &rbx_drv_tmnt },
	{ twin16_games, TWIN16_GAME_COUNT, &rbx_drv_twin16 },
	{ nemesis_games, NEMESIS_GAME_COUNT, &rbx_drv_nemesis },
	{ gradius3_games, GRADIUS3_GAME_COUNT, &rbx_drv_gradius3 },
	{ wecleman_games, WECLEMAN_GAME_COUNT, &rbx_drv_wecleman },
	{ ultraman_games, ULTRAMAN_GAME_COUNT, &rbx_drv_ultraman },
	{ overdriv_games, OVERDRIV_GAME_COUNT, &rbx_drv_konami_none },
	{ lethal_games, LETHAL_GAME_COUNT, &rbx_drv_konami_none },
	{ vendetta_games, VENDETTA_GAME_COUNT, &rbx_drv_simpsons },
	{ simpsons_games, SIMPSONS_GAME_COUNT, &rbx_drv_simpsons },
	{ parodius_games, PARODIUS_GAME_COUNT, &rbx_drv_simpsons },
	{ rollerg_games, ROLLERG_GAME_COUNT, &rbx_drv_simpsons },
	{ surpratk_games, SURPRATK_GAME_COUNT, &rbx_drv_simpsons },
	{ k88games_games, K88GAMES_GAME_COUNT, &rbx_drv_aliens },
	{ aliens_games, ALIENS_GAME_COUNT, &rbx_drv_aliens },
	{ crimfght_games, CRIMFGHT_GAME_COUNT, &rbx_drv_aliens },
	{ thunderx_games, THUNDERX_GAME_COUNT, &rbx_drv_aliens },
	{ chqflag_games, CHQFLAG_GAME_COUNT, &rbx_drv_chqflag },
	{ blockhl_games, BLOCKHL_GAME_COUNT, &rbx_drv_aliens },
	{ hcastle_games, HCASTLE_GAME_COUNT, &rbx_drv_hcastle },
	{ ajax_games, AJAX_GAME_COUNT, &rbx_drv_ajax },
	{ mainevt_games, MAINEVT_GAME_COUNT, &rbx_drv_mainevt },
	{ contra_games, CONTRA_GAME_COUNT, &rbx_drv_contra },
	{ combatsc_games, COMBATSC_GAME_COUNT, &rbx_drv_combatsc },
	{ labyrunr_games, LABYRUNR_GAME_COUNT, &rbx_drv_labyrunr },
	{ bladestl_games, BLADESTL_GAME_COUNT, &rbx_drv_bladestl },
	{ battlnts_games, BATTLNTS_GAME_COUNT, &rbx_drv_battlnts },
	{ bottom9_games, BOTTOM9_GAME_COUNT, &rbx_drv_mainevt },
	{ flkatck_games, FLKATCK_GAME_COUNT, &rbx_drv_flkatck },
	{ fastlane_games, FASTLANE_GAME_COUNT, &rbx_drv_fastlane },
	{ rockrage_games, ROCKRAGE_GAME_COUNT, &rbx_drv_rockrage },
	{ spy_games, SPY_GAME_COUNT, &rbx_drv_mainevt },
	{ ddribble_games, DDRIBBLE_GAME_COUNT, &rbx_drv_ddribble },
	{ jackal_games, JACKAL_GAME_COUNT, &rbx_drv_jackal },
	{ konblands_games, KONBLANDS_GAME_COUNT, &rbx_drv_konami_none },
	{ hexion_games, HEXION_GAME_COUNT, &rbx_drv_konami_none },
	{ divebomb_games, DIVEBOMB_GAME_COUNT, &rbx_drv_konami_none },
	{ ironhors_games, IRONHORS_GAME_COUNT, &rbx_drv_ironhors },
	{ scotrsht_games, SCOTRSHT_GAME_COUNT, &rbx_drv_scotrsht },
	{ finalizr_games, FINALIZR_GAME_COUNT, &rbx_drv_finalizr },
	{ megazone_games, MEGAZONE_GAME_COUNT, &rbx_drv_megazone },
	{ jailbrek_games, JAILBREK_GAME_COUNT, &rbx_drv_jailbrek },
	{ rocnrope_games, ROCNROPE_GAME_COUNT, &rbx_drv_rocnrope },
	{ pandoras_games, PANDORAS_GAME_COUNT, &rbx_drv_pandoras },
	{ shaolins_games, SHAOLINS_GAME_COUNT, &rbx_drv_shaolins },
	{ yiear_games, YIEAR_GAME_COUNT, &rbx_drv_yiear },
	{ mikie_games, MIKIE_GAME_COUNT, &rbx_drv_mikie },
	{ trackfld_games, TRACKFLD_GAME_COUNT, &rbx_drv_trackfld },
	{ hyperspt_games, HYPERSPT_GAME_COUNT, &rbx_drv_hyperspt },
	{ sbasketb_games, SBASKETB_GAME_COUNT, &rbx_drv_sbasketb },
	{ circusc_games, CIRCUSC_GAME_COUNT, &rbx_drv_circusc },
	{ pingpong_games, PINGPONG_GAME_COUNT, &rbx_drv_pingpong },
	{ tp84_games, TP84_GAME_COUNT, &rbx_drv_tp84 },
	{ gberet_games, GBERET_GAME_COUNT, &rbx_drv_gberet },
	{ gyruss_games, GYRUSS_GAME_COUNT, &rbx_drv_gyruss },
	{ pooyan_games, POOYAN_GAME_COUNT, &rbx_drv_pooyan },
	{ timeplt_games, TIMEPLT_GAME_COUNT, &rbx_drv_timeplt },
	{ tutankhm_games, TUTANKHM_GAME_COUNT, &rbx_drv_tutankhm },
	{ junofrst_games, JUNOFRST_GAME_COUNT, &rbx_drv_junofrst },
	{ gryzor_ms_games, GRYZOR_MS_GAME_COUNT, &rbx_drv_konami_none },
	{ konmedal_games, KONMEDAL_GAME_COUNT, &rbx_drv_konmedal },
	{ konmedal68k_games, KONMEDAL68K_GAME_COUNT, &rbx_drv_konmedal68k },
	{ konmedal020_games, KONMEDAL020_GAME_COUNT, &rbx_drv_konami_none },
	{ konmedalppc_games, KONMEDALPPC_GAME_COUNT, &rbx_drv_konami_none },
	{ kontest_games, KONTEST_GAME_COUNT, &rbx_drv_kontest },
	{ kpontoon_games, KPONTOON_GAME_COUNT, &rbx_drv_konami_none },
	{ mogura_games, MOGURA_GAME_COUNT, &rbx_drv_mogura },
	{ quickpick5_games, QUICKPICK5_GAME_COUNT, &rbx_drv_quickpick5 },
	{ tgtpanic_games, TGTPANIC_GAME_COUNT, &rbx_drv_tgtpanic },
	{ cougar_games, COUGAR_GAME_COUNT, &rbx_drv_konami_none },
	{ kongs470_games, KONGS470_GAME_COUNT, &rbx_drv_konami_none },
	{ konendev_games, KONENDEV_GAME_COUNT, &rbx_drv_konami_none },
	{ stingnet_games, STINGNET_GAME_COUNT, &rbx_drv_konami_none },
	{ hornet_games, HORNET_GAME_COUNT, &rbx_drv_konami_none },
	{ nwktr_games, NWKTR_GAME_COUNT, &rbx_drv_konami_none },
	{ zr107_games, ZR107_GAME_COUNT, &rbx_drv_konami_none },
	{ gticlub_games, GTICLUB_GAME_COUNT, &rbx_drv_konami_none },
	{ cobra_games, COBRA_GAME_COUNT, &rbx_drv_konami_none },
	{ viper_games, VIPER_GAME_COUNT, &rbx_drv_konami_none },
	{ konamim2_games, KONAMIM2_GAME_COUNT, &rbx_drv_konami_none },
	{ ultrsprt_games, ULTRSPRT_GAME_COUNT, &rbx_drv_konami_none },
	{ firebeat_games, FIREBEAT_GAME_COUNT, &rbx_drv_konami_none },
	{ twinkle_games, TWINKLE_GAME_COUNT, &rbx_drv_konami_none },
	{ ksys573_games, KSYS573_GAME_COUNT, &rbx_drv_konami_none },
	{ konamigv_games, KONAMIGV_GAME_COUNT, &rbx_drv_konami_none },
	{ konamigq_games, KONAMIGQ_GAME_COUNT, &rbx_drv_konami_none },
	{ konamigs_games, KONAMIGS_GAME_COUNT, &rbx_drv_konami_none },
	{ kpython_games, KPYTHON_GAME_COUNT, &rbx_drv_konami_none },
	{ kpython2_games, KPYTHON2_GAME_COUNT, &rbx_drv_konami_none },
};

// the game list's tabs: the generations of ADR-022 (3D / PC: the boards beyond the Vita, listed only)
static const char *const s_groups[] = { "GX", "16-bit", "8-bit", "Classic", "Medal", "3D / PC" };

// the boards in ADR-022's order with their tab
static const struct { const char *board; int tab; } s_order[] = {
	{ "Konami GX", 0 },
	{ "Konami Polygonet", 0 },
	{ "Konami DJ Main", 0 },
	{ "Konami Tasman", 0 },
	{ "Konami Mystic Warriors", 1 },
	{ "Konami Wild West C.O.W.-Boys of Moo Mesa", 1 },
	{ "Konami Xexex", 1 },
	{ "Konami G.I. Joe", 1 },
	{ "Konami Dragon Ball Z", 1 },
	{ "Konami Run and Gun", 1 },
	{ "Konami Asterix", 1 },
	{ "Konami Bishi Bashi Champ Mini Game Senshuken", 1 },
	{ "Konami Quiz Do Re Mi Fa Grand Prix", 1 },
	{ "Konami Pirate Ship", 1 },
	{ "Konami X-Men", 1 },
	{ "Konami TMNT 2", 1 },
	{ "Konami Golden Region", 1 },
	{ "Konami TMNT", 1 },
	{ "Konami Twin 16", 1 },
	{ "Konami Nemesis", 1 },
	{ "Konami Gradius III", 1 },
	{ "Konami WEC Le Mans 24", 1 },
	{ "Konami Ultraman", 1 },
	{ "Konami Over Drive", 1 },
	{ "Konami Lethal Enforcers", 2 },
	{ "Konami Vendetta", 2 },
	{ "Konami The Simpsons", 2 },
	{ "Konami Parodius Da!", 2 },
	{ "Konami Rollergames", 2 },
	{ "Konami Surprise Attack", 2 },
	{ "Konami '88 Games", 2 },
	{ "Konami Aliens", 2 },
	{ "Konami Crime Fighters", 2 },
	{ "Konami Super Contra", 2 },
	{ "Konami Chequered Flag", 2 },
	{ "Konami Block Hole", 2 },
	{ "Konami Haunted Castle", 2 },
	{ "Konami Ajax", 2 },
	{ "Konami The Main Event", 2 },
	{ "Konami Contra", 2 },
	{ "Konami Combat School", 2 },
	{ "Konami Trick Trap", 2 },
	{ "Konami Blades of Steel", 2 },
	{ "Konami Battlantis", 2 },
	{ "Konami Bottom of the Ninth", 2 },
	{ "Konami MX5000", 2 },
	{ "Konami Fast Lane", 2 },
	{ "Konami Rock'n Rage", 2 },
	{ "Konami S.P.Y.", 2 },
	{ "Konami Double Dribble", 2 },
	{ "Konami Jackal", 2 },
	{ "Konami Badlands", 2 },
	{ "Konami Hexion", 2 },
	{ "Konami Kyuukoukabakugekitai", 2 },
	{ "Konami Iron Horse", 3 },
	{ "Konami Scooter Shooter", 3 },
	{ "Konami Finalizer", 3 },
	{ "Konami Mega Zone", 3 },
	{ "Konami Jail Break", 3 },
	{ "Konami Roc'n Rope", 3 },
	{ "Konami Pandora's Palace", 3 },
	{ "Konami Kicker", 3 },
	{ "Konami Yie Ar Kung-Fu", 3 },
	{ "Konami Mikie", 3 },
	{ "Konami Track & Field", 3 },
	{ "Konami Hyper Sports", 3 },
	{ "Konami Super Basketball", 3 },
	{ "Konami Circus Charlie", 3 },
	{ "Konami Konami's Ping-Pong", 3 },
	{ "Konami Time Pilot '84", 3 },
	{ "Konami Green Beret", 3 },
	{ "Konami Gyruss", 3 },
	{ "Konami Pooyan", 3 },
	{ "Konami Time Pilot", 3 },
	{ "Konami Tutankham", 3 },
	{ "Konami Juno First", 3 },
	{ "Konami Gryzor (Modular System)", 3 },
	{ "Konami Medal", 4 },
	{ "Konami Medal 68000", 4 },
	{ "Konami Medal 68020", 4 },
	{ "Konami Medal PowerPC", 4 },
	{ "Konami Test Board", 4 },
	{ "Konami Pontoon", 4 },
	{ "Konami Mogura Desse", 4 },
	{ "Konami Quick Pick 5", 4 },
	{ "Konami Target Panic", 4 },
	{ "Konami Spin Fever", 4 },
	{ "Konami Trio de Bingo", 4 },
	{ "Konami Endeavour", 4 },
	{ "Konami Tropical Chance", 4 },
	{ "Konami Hornet", 5 },
	{ "Konami NWK-TR", 5 },
	{ "Konami ZR107", 5 },
	{ "Konami GTI Club", 5 },
	{ "Konami Cobra", 5 },
	{ "Konami Viper", 5 },
	{ "Konami M2", 5 },
	{ "Konami Ultra Sports", 5 },
	{ "Konami Firebeat", 5 },
	{ "Konami Twinkle", 5 },
	{ "Konami System 573", 5 },
	{ "Konami GV", 5 },
	{ "Konami GQ", 5 },
	{ "Konami GS", 5 },
	{ "Konami Python", 5 },
	{ "Konami Python 2", 5 },
};
#define NORDER ((int)(sizeof(s_order) / sizeof(s_order[0])))

static int board_order(const char *board)
{
	for (int i = 0; i < NORDER; i++)
		if (!strcmp(s_order[i].board, board))
			return i;
	return 1000;
}

static int group(const rbx_game *g)
{
	const int i = board_order(g->board);
	return i < NORDER ? s_order[i].tab : 0;
}

const rbx_module rbx_mod_konami = {
	"konami", "Konami",
	s_boards, (int)(sizeof(s_boards) / sizeof(s_boards[0])),
	s_groups, (int)(sizeof(s_groups) / sizeof(s_groups[0])),
	group, board_order,
};
