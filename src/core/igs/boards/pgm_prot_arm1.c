/*  PGM protection: IGS027A type 1 (55857E / 55857G), ported from MAME src/mame/igs/pgmprot_igs027a_type1.cpp / .h
    (MAME 0.289, BSD-3-Clause, copyright-holders: David Haywood, ElSemi, Xing Xing).
    - pgm_arm_type1: the ARM7 running its internal ROM ("prot") for Photo Y2K, Knights of Valour Superheroes and
      their bootlegs (kovshp / kovshxas / kovlsqh2 / kovqhsgs with MAME's command remapping and ROM decoders).
    - pgm_arm_type1_sim: MAME's simulations (kov / kovboot, puzzli2, py2k2, pgm3in1, pstar, oldsplus).
    - pgm_arm_type1_cave: the simulations with the Cave map (ddp3, ket, espgal; the whole 0x000000-0x3fffff is ROM).
    The ARM runs a scanline at a time after the 68000 (MAME interleaves them at the scanline timer).
    The simulation code below follows MAME line for line; MAME's member names are kept through macros on S.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pgm.h"
#include "../../cpu/arm7/arm7.h"
#include "../machine/pgmcrypt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int32_t s32;
typedef uint32_t offs_t;

#define ARM1_CLOCK 20000000

// (MAME's logging / printf of the simulations: dropped)
#define sim_printf(...) ((void)0)
#define sim_log(...) ((void)0)
// table read with MAME's unchecked index kept inside the table (0 past it)
#define TBL(t, i) ((unsigned)(i) < sizeof(t) / sizeof((t)[0]) ? (t)[(unsigned)(i)] : 0)

static uint32_t bsw(uint32_t v, int n, const int8_t *b)   // MAME bitswap<n>(v, b[0] (msb) .. b[n-1] (lsb))
{
	uint32_t r = 0;
	for (int k = 0; k < n; k++)
		r |= ((v >> b[k]) & 1u) << (n - 1 - k);
	return r;
}
#define BITSWAP16(v, ...) ((u16)bsw((v), 16, (const int8_t[16]){ __VA_ARGS__ }))

// the whole device state (saved as one block); field names are MAME's members
typedef struct {
	// simulations
	u16 m_value0, m_value1, m_valuekey, m_ddp3lastcommand;
	u32 m_valueresponse;
	s32 m_curslots;
	u32 m_slots[0x100];
	u16 m_pstar_e7_value, m_pstar_b1_value, m_pstar_ce_value;
	u16 m_kov_c0_value, m_kov_cb_value, m_kov_fe_value;
	u16 m_extra_ram[0x100];
	s32 m_puzzli_54_trigger;
	s32 m_simregion;
	// puzzli2
	s32 stage, tableoffs, tableoffs2, entries_left, currentcolumn, currentrow, num_entries, full_entry, prev_tablloc;
	s32 numbercolumns, depth;
	u16 m_row_bitmask;
	s32 hackcount, hackcount2, hack_47_value, hack_31_table_offset, hack_31_table_offset2, p2_31_retcounter;
	u8 coverage[256];
	s32 command_31_write_type;
	u16 level_structure[8][10];
	// py2k2
	u16 m_py2k2_sprite_pos, m_py2k2_sprite_base, m_py2k2_prev_base;
	// emulation
	u32 m_arm_type1_latch_arm_w, m_arm_type1_latch_68k_w, m_arm_type1_counter;
	u32 shareram[0x20];    // 0x50800000 (0x40 bytes; the 68000's 0x4f0000-0x4f003f reach past it, as in MAME)
	u8 iram[0x400];        // 0x10000000
	u8 xor_ram[0x400];     // 0x50000000
	uint64_t frac;
} arm1_state;

static arm1_state S;
static void (*arm_sim_handler)(int pc);
static int puzzli2_take_leveldata_value(u8 datvalue);

#define m_value0 S.m_value0
#define m_value1 S.m_value1
#define m_valuekey S.m_valuekey
#define m_ddp3lastcommand S.m_ddp3lastcommand
#define m_valueresponse S.m_valueresponse
#define m_curslots S.m_curslots
#define m_slots S.m_slots
#define m_pstar_e7_value S.m_pstar_e7_value
#define m_pstar_b1_value S.m_pstar_b1_value
#define m_pstar_ce_value S.m_pstar_ce_value
#define m_kov_c0_value S.m_kov_c0_value
#define m_kov_cb_value S.m_kov_cb_value
#define m_kov_fe_value S.m_kov_fe_value
#define m_extra_ram S.m_extra_ram
#define m_puzzli_54_trigger S.m_puzzli_54_trigger
#define m_simregion S.m_simregion
#define stage S.stage
#define tableoffs S.tableoffs
#define tableoffs2 S.tableoffs2
#define entries_left S.entries_left
#define currentcolumn S.currentcolumn
#define currentrow S.currentrow
#define num_entries S.num_entries
#define full_entry S.full_entry
#define prev_tablloc S.prev_tablloc
#define numbercolumns S.numbercolumns
#define depth S.depth
#define m_row_bitmask S.m_row_bitmask
#define hackcount S.hackcount
#define hackcount2 S.hackcount2
#define hack_47_value S.hack_47_value
#define hack_31_table_offset S.hack_31_table_offset
#define hack_31_table_offset2 S.hack_31_table_offset2
#define p2_31_retcounter S.p2_31_retcounter
#define coverage S.coverage
#define command_31_write_type S.command_31_write_type
#define level_structure S.level_structure
#define m_py2k2_sprite_pos S.m_py2k2_sprite_pos
#define m_py2k2_sprite_base S.m_py2k2_sprite_base
#define m_py2k2_prev_base S.m_py2k2_prev_base

/**************************** SIMULATIONS (MAME) *****************************/

static u16 arm7_type1_sim_r(offs_t offset)
{
	if (offset == 0)
	{
		u16 d = m_valueresponse & 0xffff;
		u16 realkey = m_valuekey >> 8;
		realkey |= m_valuekey;
		d ^= realkey;
		return d;
	}
	else if (offset == 1)
	{
		u16 d = m_valueresponse >> 16;
		u16 realkey = m_valuekey >> 8;
		realkey |= m_valuekey;
		d ^= realkey;
		return d;
	}
	return 0xffff;
}

/* working */
static void command_handler_ddp3(int pc)
{
	switch (m_ddp3lastcommand)
	{
		default:
			sim_printf("%06x command %02x | %04x\n", pc, m_ddp3lastcommand, m_value0);
			m_valueresponse = 0x880000;
			break;

		case 0x40:
			m_valueresponse = 0x880000;
			m_slots[(m_value0 >> 10) & 0x1F]=
				(m_slots[(m_value0 >> 5) & 0x1F]+
					m_slots[(m_value0 >> 0) & 0x1F]) & 0xffffff;
			break;

		case 0x67: // set high bits
	//      sim_printf("%s command %02x | %04x\n", machine().describe_context().c_str(), m_ddp3lastcommand, m_value0);
			m_valueresponse = 0x880000;
			m_curslots = (m_value0 & 0xff00) >> 8;
			m_slots[m_curslots] = (m_value0 & 0x00ff) << 16;
			break;

		case 0xe5: // set low bits for operation?
		//  sim_printf("%s command %02x | %04x\n", machine().describe_context().c_str(), m_ddp3lastcommand, m_value0);
			m_valueresponse = 0x880000;
			m_slots[m_curslots] |= (m_value0 & 0xffff);
			break;

		case 0x8e: // read back result of operations
	//      sim_printf("%s command %02x | %04x\n", machine().describe_context().c_str(), m_ddp3lastcommand, m_value0);
			m_valueresponse = m_slots[m_value0 & 0xff];
			break;

		case 0x99: // reset?
			m_simregion = 0;//pgm_region_port();
			m_valuekey = 0x100;
			m_valueresponse = 0x00880000 | m_simregion << 8;
			break;

	}
}

/* preliminary */

// should be correct, note each value only appears once
static const u8 puzzli2_level_decode[256] = {
	// 0  ,  1  ,  2  ,  3  ,  4  ,  5   , 6  ,  7  ,  8  ,  9  ,  a  ,  b  ,  c  ,  d  ,  e  ,  f  ,
	0x32, 0x3e, 0xb2, 0x37, 0x31, 0x22, 0xd6, 0x0d, 0x35, 0x5c, 0x8d, 0x3c, 0x7a, 0x5f, 0xd7, 0xac, // 0x0
//   0  ,  0  ,  0  ,  0  ,  0  ,  1  ,  1  ,  0  ,  1  ,  1  ,  0  ,  0  ,  0  ,  0  ,  x  ,  x  ,
	0x53, 0xff, 0xeb, 0x44, 0xe8, 0x11, 0x69, 0x77, 0xd9, 0x34, 0x36, 0x45, 0xa6, 0xe9, 0x1c, 0xc6, // 0x1
//   0  ,  0  ,  x  ,  x  ,  x  ,  0  ,  x  ,  0  ,  x  ,  0  ,  0  ,  0  ,  0  ,  x  ,  0  ,  x  ,
	0x3b, 0xbd, 0xad, 0x2e, 0x18, 0xdf, 0xa1, 0xab, 0xdd, 0x52, 0x57, 0xc2, 0xe5, 0x0a, 0x00, 0x6d, // 0x2
//   0  ,  0  ,  0  ,  1  ,  1  ,  1  ,  1  ,  x  ,  1  ,  1  ,  0  ,  0  ,  1  ,  1  ,  x  ,  0  ,
	0x67, 0x64, 0x15, 0x70, 0xb6, 0x39, 0x27, 0x78, 0x82, 0xd2, 0x71, 0xb9, 0x13, 0xf5, 0x93, 0x92, // 0x3
//   0  ,  x  ,  1  ,  1  ,  x  ,  1  ,  1  ,  1  ,  1  ,  1  ,  1  ,  1  ,  x  ,  0  ,  x  ,  x  ,
	0xfa, 0xe7, 0x5e, 0xb0, 0xf6, 0xaf, 0x95, 0x8a, 0x7c, 0x73, 0xf9, 0x63, 0x86, 0xcb, 0x1a, 0x56, // 0x4
//   0  ,  1  ,  1  ,  0  ,  0  ,  0  ,  0  ,  1  ,  1  ,  1  ,  1  ,  0  ,  1  ,  1  ,  1  ,  0  ,
	0xf1, 0x3a, 0xae, 0x61, 0x01, 0x29, 0x97, 0x23, 0x8e, 0x5d, 0x9a, 0x65, 0x74, 0x21, 0x20, 0x40, // 0x5
//   0  ,  1  ,  1  ,  1  ,  1  ,  0  ,  x  ,  x  ,  x  ,  0  ,  0  ,  1  ,  1  ,  1  ,  1  ,  1  ,
	0xd3, 0x05, 0xa2, 0xe1, 0xbc, 0x9e, 0x1e, 0x10, 0x14, 0x0c, 0x88, 0x9c, 0xec, 0x38, 0xb5, 0x9d, // 0x6
//   1  ,  0  ,  0  ,  x  ,  1  ,  1  ,  0  ,  0  ,  x  ,  0  ,  x  ,  0  ,  0  ,  1  ,  1  ,  1  ,
	0x2d, 0xf7, 0x17, 0x0e, 0x84, 0xc7, 0x7d, 0xce, 0x94, 0x16, 0x48, 0xa8, 0x81, 0x6e, 0x7b, 0xd8, // 0x7
//   1  ,  1  ,  1  ,  1  ,  x  ,  0  ,  x  ,  0  ,  1  ,  1  ,  1  ,  x  ,  x  ,  1  ,  1  ,  1  ,
	0xa7, 0x7f, 0x42, 0xe6, 0xa0, 0x2a, 0xef, 0xee, 0x24, 0xba, 0xb8, 0x7e, 0xc9, 0x2b, 0x90, 0xcc, // 0x8
//   1  ,  x  ,  1  ,  1  ,  1  ,  1  ,  1  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
	0x5b, 0xd1, 0xf3, 0xe2, 0x6f, 0xed, 0x9f, 0xf0, 0x4b, 0x54, 0x8c, 0x08, 0xf8, 0x51, 0x68, 0xc8, // 0x9
//   x  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  x  ,  0  ,  x  ,  0  ,  0  ,  0  ,  0  ,  1  ,
	0x03, 0x0b, 0xbb, 0xc1, 0xe3, 0x4d, 0x04, 0xc5, 0x8f, 0x09, 0x0f, 0xbf, 0x62, 0x49, 0x76, 0x59, // 0xa
//   1  ,  1  ,  1  ,  1  ,  1  ,  x  ,  0  ,  1  ,  1  ,  0  ,  x  ,  1  ,  1  ,  1  ,  1  ,  0  ,
	0x1d, 0x80, 0xde, 0x60, 0x07, 0xe0, 0x1b, 0x66, 0xa5, 0xbe, 0xcd, 0x87, 0xdc, 0xc3, 0x6b, 0x4e, // 0xb
//   0  ,  1  ,  1  ,  1  ,  1  ,  x  ,  0  ,  x  ,  0  ,  x  ,  0  ,  1  ,  1  ,  0  ,  1  ,  1  ,
	0xd0, 0xfd, 0xd4, 0x3f, 0x98, 0x96, 0x2f, 0x4c, 0xb3, 0xea, 0x2c, 0x75, 0xe4, 0xc0, 0x6c, 0x6a, // 0xc
//   0  ,  x  ,  1  ,  1  ,  0  ,  1  ,  1  ,  1  ,  1  ,  0  ,  1  ,  1  ,  x  ,  1  ,  1  ,  1  ,
	0x9b, 0xb7, 0x43, 0x8b, 0x41, 0x47, 0x02, 0xdb, 0x99, 0x3d, 0xa3, 0x79, 0x50, 0x4f, 0xb4, 0x55, // 0xd
//   1  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  x  ,  x  ,  1  ,  1  ,  1  ,  0  ,  1  ,  1  ,  1  ,
	0x5a, 0x25, 0xf4, 0xca, 0x58, 0x30, 0xc4, 0x12, 0xa9, 0x46, 0xda, 0x91, 0xa4, 0xaa, 0xfc, 0x85, // 0xe
//   1  ,  1  ,  0  ,  1  ,  1  ,  1  ,  1  ,  0  ,  0  ,  1  ,  1  ,  1  ,  1  ,  0  ,  0  ,  x  ,
	0xfb, 0x89, 0x06, 0xcf, 0xfe, 0x33, 0xd5, 0x28, 0x1f, 0x19, 0x4a, 0xb1, 0x83, 0xf2, 0x72, 0x26, // 0xf
//   x  ,  x  ,  1  ,  1  ,  1  ,  1  ,  1  ,  1  ,  x  ,  0  ,  1  ,  1  ,  1  ,  1  ,  1  ,  1  ,
};


#define puzzli2_printf sim_log

static void command_handler_puzzli2(int pc)
{
	switch (m_ddp3lastcommand)
	{
		case 0x31:
		{
			// how is this selected? command 54?

			/* writes the following sequence before how to play
			 each level has a different sequence written before it, size of sequence doesn't seem directly connected to level size (unlike the reads)
			 so it's probably compressed somehow as well as scrambled?  68k doesnt know in advance how big each lot of data is either, it only stops
			 writing when it gets a difference response from the MCU.

			00138278: 31 00fd | (set xor table offset)
		UNKNOWN - related to depth / number of columns?
			00138278: 31 0087 | value 87, after xor is 75 (table address,value fd,f2)
		COLUMN 1
			00138278: 31 0032 | value 32, after xor is 40 (table address,value fe,72) << 4 is the number of entries in this column
			00138278: 31 0029 | value 29, after xor is 0f (table address,value ff,26) << 0x0f is a mask of 4 bits..

			00138278: 31 0031 | value 31, after xor is 03 (table address,value 00,32)  -> 0x0103
			00138278: 31 003f | value 3f, after xor is 01 (table address,value 01,3e)  -> 0x0101
			00138278: 31 00b0 | value b0, after xor is 02 (table address,value 02,b2)  -> 0x0102
			00138278: 31 0035 | value 35, after xor is 02 (table address,value 03,37)  -> 0x0102
		COLUMN 2
			00138278: 31 0071 | value 71, after xor is 40 (table address,value 04,31) << 4 is the number of entries in this column
			00138278: 31 002d | value 2d, after xor is 0f (table address,value 05,22) << 0x0f is a mask of 4 bits..

			00138278: 31 00d5 | value d5, after xor is 03 (table address,value 06,d6)  -> 0x0103
			00138278: 31 000d | value 0d, after xor is 00 (table address,value 07,0d)  -> 0x0100
			00138278: 31 0034 | value 34, after xor is 01 (table address,value 08,35)  -> 0x0101
			00138278: 31 0059 | value 59, after xor is 05 (table address,value 09,5c)  -> 0x0105
		COLUMN 3
			00138278: 31 00dd | value dd, after xor is 50 (table address,value 0a,8d) << 5 is the number of entries in this column
			00138278: 31 0023 | value 23, after xor is 1f (table address,value 0b,3c) << 0x1f is a mask of 5 bits..

			00138278: 31 007a | value 7a, after xor is 00 (table address,value 0c,7a)  -> 0x0100
			00138278: 31 00f3 | value f3, after xor is 01 (table address,value fd,f2)  -> 0x0101
			00138278: 31 0077 | value 77, after xor is 05 (table address,value fe,72)  -> 0x0105
			00138278: 31 0022 | value 22, after xor is 04 (table address,value ff,26)  -> 0x0104
			00138278: 31 0036 | value 36, after xor is 04 (table address,value 00,32)  -> 0x0104
		COLUMN 4
			00138278: 31 002e | value 2e, after xor is 10 (table address,value 01,3e) << 1 is the number of entries in this column
			00138278: 31 00b3 | value b3, after xor is 01 (table address,value 02,b2) << 0x01 is a mask of 1 bit..

			00138278: 31 0035 | value 35, after xor is 02 (table address,value 03,37)  -> 0x0102
		COLUMN 5
			00138278: 31 0041 | value 41, after xor is 70 (table address,value 04,31) << 7 is the number of entries in this column
			00138278: 31 005d | value 5d, after xor is 7f (table address,value 05,22) << 0x7f is a mask of 7 bits..

			00138278: 31 00d6 | value d6, after xor is 00 (table address,value 06,d6)  -> 0x0100
			00138278: 31 000c | value 0c, after xor is 01 (table address,value 07,0d)  -> 0x0101
			00138278: 31 0036 | value 36, after xor is 03 (table address,value 08,35)  -> 0x0103
			00138278: 31 005e | value 5e, after xor is 02 (table address,value 09,5c)  -> 0x0102
			00138278: 31 0089 | value 89, after xor is 04 (table address,value 0a,8d)  -> 0x0104
			00138278: 31 003c | value 3c, after xor is 00 (table address,value 0b,3c)  -> 0x0100
			00138278: 31 007a | value 7a, after xor is 00 (table address,value 0c,7a)  -> 0x0100
		COLUMN 6
			00138278: 31 00a2 | value a2, after xor is 50 (table address,value fd,f2) << 5 is the number of entries in this column
			00138278: 31 006d | value 6d, after xor is 1f (table address,value fe,72) << 0x1f is a mask of 5 bits..

			00138278: 31 0023 | value 23, after xor is 05 (table address,value ff,26)  -> 0x0105
			00138278: 31 0037 | value 37, after xor is 05 (table address,value 00,32)  -> 0x0105
			00138278: 31 003f | value 3f, after xor is 01 (table address,value 01,3e)  -> 0x0101
			00138278: 31 00b3 | value b3, after xor is 01 (table address,value 02,b2)  -> 0x0101
			00138278: 31 0034 | value 34, after xor is 03 (table address,value 03,37)  -> 0x0103
			 ^ (end, returning 630006 as playfield width)

			*/

			if (command_31_write_type==2)
			{
				puzzli2_printf("%08x: %02x %04x | ",pc, m_ddp3lastcommand, m_value0);

				// this shouldn't apply to the stuff written on startup, only the level data..

				if (hackcount2==0)
				{
					puzzli2_take_leveldata_value(m_value0 & 0xff);

					hack_31_table_offset = m_value0 & 0xff;
					hack_31_table_offset2 = 0;
					hackcount2++;
					m_valueresponse = 0x00d20000;

					//puzzli2_printf("(set xor table offset)\n");
				}
				else  // how do we decide end?
				{
					int end = puzzli2_take_leveldata_value(m_value0 & 0xff);

					if (!end)
					{
						// always d2 0000 when writing doing level data
						// but different for the writes on startup?
						m_valueresponse = 0x00d20000;

						//u8 tableaddr = (hack_31_table_offset + (hack_31_table_offset2 & 0xf)) & 0xff;
						//u8 xoredval = m_value0 ^ puzzli2_level_decode[tableaddr];
						//puzzli2_printf("value %02x, after xor is %02x (table address,value %02x,%02x)\n", m_value0, xoredval, tableaddr, puzzli2_level_decode[tableaddr]);

						hackcount2++;
						hack_31_table_offset2++;
					}
					else
					{
						hackcount2 = 0;

						// when the ARM detects the end of the stream has been reached it returns a 0x63 status with the number of columns in the data word
						m_valueresponse = 0x00630000 | numbercolumns;

						//u8 tableaddr = (hack_31_table_offset + (hack_31_table_offset2 & 0xf)) & 0xff;
						//u8 xoredval = m_value0 ^ puzzli2_level_decode[tableaddr];
						//puzzli2_printf("value %02x, after xor is %02x (table address,value %02x,%02x) (end, returning %02x as playfield width)\n", m_value0, xoredval, tableaddr, puzzli2_level_decode[tableaddr], m_valueresponse);
					}
				}
			}
			else
			{
				// todo, responses when uploading the startup values are different
				puzzli2_printf("%08x: %02x %04x (for z80 address?)\n ",pc, m_ddp3lastcommand, m_value0);

				m_valueresponse = 0x00d20000 | p2_31_retcounter;
				p2_31_retcounter++; // returns 0xc for the first one, 0x19 for the last one
			}

		}
		break;

		// after writing the compressed and scrambled data stream for the level (copied from ROM) with command 0x31
		// the game expects to read back a fully formed level structure from the ARM
		case 0x13:
		{
			puzzli2_printf("%08x: %02x %04x (READ LEVEL DATA) | ",pc, m_ddp3lastcommand, m_value0);

			// this is the how to play screen, correctly returned with current code
			/*
			u16 retvals[61] =
			{ 0x0008, // depth (-2?)
			  0x0103, 0x0101, 0x0102, 0x0102, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // first column
			  0x0103, 0x0100, 0x0101, 0x0105, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			  0x0100, 0x0101, 0x0105, 0x0104, 0x0104, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			  0x0102, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			  0x0100, 0x0101, 0x0103, 0x0102, 0x0104, 0x0100 ,0x0100, 0x0000, 0x0000, 0x0000,
			  0x0105, 0x0105, 0x0101, 0x0101, 0x0103, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000  // last column
			};
			*/


			u16* leveldata = &level_structure[0][0];
			if (hackcount==0)
			{
				m_valueresponse = 0x002d0000 | ((depth>>4)+1); // this *seems* to come from upper bits of the first real value written to the device during the level stream (verify, seems wrong for some levels because you get a black bar on the bottom of the screen, but might be bad xors)
				puzzli2_printf("level depth returning %08x\n", m_valueresponse );
			}
			else if (hackcount<((10*numbercolumns)+1))
			{
				m_valueresponse = 0x002d0000 | (hackcount - 1 < 8 * 10 ? leveldata[hackcount-1] : 0);
				puzzli2_printf("level data returning %08x\n", m_valueresponse );
			}
			else
			{
				hackcount=0;
				m_valueresponse = 0x00740054;  // 0x0074 0054 is returned after how to play reads above.. where does 0x54 come from?
				puzzli2_printf("END returning %08x\n", m_valueresponse );

			}

			hackcount++;


			// 2d seems to be used when there is more data available
			// 74 seems to be used when there isn't.. (end of buffer reached?)
			// 2d or 74! (based on?)

		}
		break;



		case 0x38: // Reset
			puzzli2_printf("%08x: %02x %04x (RESET)\n",pc, m_ddp3lastcommand, m_value0);
			m_simregion = pgm_region_port();
			m_valueresponse = 0x780000 | m_simregion<<8; // this must also return the cart region or the game will act in odd ways when inserting a coin on continue, or during the game on later levels
			m_valuekey = 0x100;
			m_puzzli_54_trigger = 0;

		break;




		// 47 and 52 are used to get the images during the intro sequence, different each loop
		// also some other gfx?
		// logic here seems correct, not sure where the 0x19 and 0x5 etc. come from tho!
		case 0x47:
			puzzli2_printf("%08x: %02x %04x (GFX OFF PART 1)\n",pc, m_ddp3lastcommand, m_value0);

			hack_47_value = m_value0;

			//hack_47_value = ((m_value0 & 0x0700) >> 8) * 0x19;
			//hack_47_value +=((m_value0 & 0x0007) >> 0) * 0x05;
			if (m_value0 & 0xf0f0) puzzli2_printf("unhandled 0x47 bits %04x\n", m_value0);

			//puzzli2_printf("0x47 value was %04x - using %04x\n", m_value0, hack_47_value);

			m_valueresponse = 0x00740047;

		break;

		case 0x52:
			puzzli2_printf("%08x: %02x %04x (GFX OFF PART 2)\n",pc, m_ddp3lastcommand, m_value0);

			//puzzli2_printf("which %04x\n", m_value0);
			if (m_value0 & 0xfff0) puzzli2_printf("unhandled 0x52 bits %04x\n", m_value0);

			// it writes a value of 0x0000 then expects to read back like
			// this for the lower part of the game backgrounds
			if (m_value0 == 0x0000)
			{
				int val = ((hack_47_value & 0x0f00) >> 8) * 0x19;
				m_valueresponse = 0x00740000 | (val & 0xffff);
			}
			else
			{
				int val = ((hack_47_value & 0x0f00) >> 8) * 0x19;
				val +=((hack_47_value & 0x000f) >> 0) * 0x05;
				val += m_value0 & 0x000f;
				m_valueresponse = 0x00740000 | (val & 0xffff);

			}



		break;



		case 0x61: // ??
			puzzli2_printf("%08x: %02x %04x\n",pc, m_ddp3lastcommand, m_value0);

			// this command is written before the values used to decrypt the z80 addresses (assumed) are uploaded with command 31
			command_31_write_type = 1;

			m_valueresponse = 0x36 << 16;
			p2_31_retcounter = 0xc;
		break;

		case 0x41: // ASIC status?
			puzzli2_printf("%08x: %02x %04x (UNK)\n",pc, m_ddp3lastcommand, m_value0);

			// this command is written after the values used to decrypt the z80 addresses (assumed) are uploaded with command 31
			command_31_write_type = 0;

			//m_valueresponse = 0x74 << 16;
			m_valueresponse = 0x740061;
		break;

		case 0x54: // ??
			puzzli2_printf("%08x: %02x %04x\n",pc, m_ddp3lastcommand, m_value0);

			// this command is written before uploading the compressed level data stream with command 31

			command_31_write_type = 2;
			stage = -1;
			m_puzzli_54_trigger = 1;
			hackcount2 = 0;
			hackcount = 0;
			m_valueresponse = 0x36 << 16;

			//  clear the return structure
			for (int cols = 0; cols < 8; cols++)
				for (int rows=0;rows<10;rows++)
					level_structure[cols][rows] = 0x0000;

		break;

	/*

	puzzli2 on startup (00148a84)         puzzli2s on startup (0014cf58)

	    (001489f6: 61 0202  0014ceca: 61 0202)
	                                           always
	    : 31 004e a6f7 | : 31 | 0051 14c9       + 26DD2
	    : 31 279e 534f | : 31 | 27a0 c121
	    : 31 ab5c a7cf | : 31 | ab5f 15a1
	    : 31 145f 7054 | : 31 | 1461 de26
	    : 31 85a0 7b7f | : 31 | 85a2 e951
	    : 31 7003 c5ab | : 31 | 7006 337d
	    : 31 456d f3aa | : 31 | 4570 617c

	    (00148b34: 41 e2bb  0014d008: 41 706d)

	    actual values needed       always
	    0x001694a8 / 0x0019027a  + 26DD2
	    0x0016cfae / 0x00193D80
	    0x0016ebf2 / 0x001959c4
	    0x0016faa8 / 0x0019687a
	    0x00174416 / 0x0019b1e8

	    0x00166178 / 0x0018cf4a
	    0x00166e72 / 0x0018dc44

	     as you can see the difference between the values written is always 26dd2, as is the difference between offsets expected
	     this makes it impossible to know which value is for which address without further tests!


	*/

		// I think the values returned here must be connected to the values written to command 31 on startup
		// 63/67 are used on startup to get the z80 music at least
		case 0x63: // used as a read address by the 68k code (related to previous uploaded values like cave?) should point at a table of ~0x80 in size? seems to use values as further pointers?
			puzzli2_printf("%08x: %02x %04x (Z80 ADDR PART 1)\n",pc, m_ddp3lastcommand, m_value0);

			if (!strcmp(pgm_game->set,"puzzli2"))
			{
				if (m_value0 == 0x0000)
				{
					m_valueresponse = 0x001694a8;
				}
				else if (m_value0 == 0x0001)
				{
					m_valueresponse = 0x0016cfae;
				}
				else if (m_value0 == 0x0002)
				{
					m_valueresponse = 0x0016ebf2; // right for puzzli2 , wrong for puzzli2s, probably calculated from the writes then?
				}
				else if (m_value0 == 0x0003) // before 'cast' screen
				{
					m_valueresponse = 0x0016faa8;
				}
				else if (m_value0 == 0x0004) // 2 player demo
				{
					m_valueresponse = 0x00174416;
				}
				else
				{
					puzzli2_printf("unk case x63\n");
					m_valueresponse = 0x00600000; // wrong

				}
			}
			else // puzzli2 super
			{
				if (m_value0 == 0x0000)
				{
					m_valueresponse = 0x19027a;
				}
				else if (m_value0 == 0x0001)
				{
					m_valueresponse = 0x193D80;
				}
				else if (m_value0 == 0x0002)
				{
					m_valueresponse = 0x1959c4;
				}
				else if (m_value0 == 0x0003)
				{
					m_valueresponse = 0x19687a;
				}
				else if (m_value0 == 0x0004)
				{
					m_valueresponse = 0x19b1e8;
				}
				else
				{
					puzzli2_printf("unk case x63\n");
					m_valueresponse = 0x00600000; // wrong
				}
			}
		break;

		case 0x67: // used as a read address by the 68k code (related to previous uploaded values like cave?) directly reads ~0xDBE from the address..
			puzzli2_printf("%08x: %02x %04x (Z80 ADDR PART 2)\n",pc, m_ddp3lastcommand, m_value0);

			if (!strcmp(pgm_game->set,"puzzli2"))
			{
				if ( (m_value0 == 0x0000) || (m_value0 == 0x0001) || (m_value0 == 0x0002) || (m_value0 == 0x0003) )
				{
					m_valueresponse = 0x00166178; // right for puzzli2 , wrong for puzzli2s, probably calculated from the writes then?
				}
				else if ( m_value0 == 0x0004 ) // 2 player demo
				{
					m_valueresponse = 0x00166e72;
				}
				else
				{
					puzzli2_printf("unk case x67\n");
					m_valueresponse = 0x00400000; // wrong
				}
			}
			else // puzzli2 super
			{
				if ((m_value0 == 0x0000) || (m_value0 == 0x0001) || (m_value0 == 0x0002) ||  (m_value0 == 0x0003))
				{
					m_valueresponse = 0x18cf4a;
				}
				else if ( m_value0 == 0x0004 ) // 2 player demo
				{
					m_valueresponse = 0x0018dc44;
				}
				else
				{
					puzzli2_printf("unk case x67\n");
					m_valueresponse = 0x00600000; // wrong
				}
			}
		break;

		default:
			puzzli2_printf("%08x: %02x %04x\n",pc, m_ddp3lastcommand, m_value0);

			m_valueresponse = 0x74 << 16;
		break;
	}
}

static u32 py2k2_sprite_offset(u16 base, u16 pos)
{
	u16 ret = 0;
	u16 offset = (base * 16) + (pos & 0xf);

	switch (base & ~0x3f)
	{
		case 0x000: ret = BITSWAP16(offset ^ 0x0030, 15, 14, 13, 12, 11, 10, 0, 2, 3, 9, 5, 4, 8, 7, 6, 1); break;
		case 0x040: ret = BITSWAP16(offset ^ 0x03c0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 1, 2, 0, 5, 3, 4); break;
		case 0x080: ret = BITSWAP16(offset ^ 0x0000, 15, 14, 13, 12, 11, 10, 0, 3, 4, 6, 8, 7, 5, 9, 2, 1); break;
		case 0x0c0: ret = BITSWAP16(offset ^ 0x0001, 15, 14, 13, 12, 11, 10, 6, 5, 4, 3, 2, 1, 9, 8, 7, 0); break;
		case 0x100: ret = BITSWAP16(offset ^ 0x0030, 15, 14, 13, 12, 11, 10, 0, 2, 3, 9, 5, 4, 8, 7, 6, 1); break;
		case 0x140: ret = BITSWAP16(offset ^ 0x01c0, 15, 14, 13, 12, 11, 10, 2, 8, 7, 6, 4, 3, 5, 9, 0, 1); break;
		case 0x180: ret = BITSWAP16(offset ^ 0x0141, 15, 14, 13, 12, 11, 10, 4, 8, 2, 6, 1, 7, 9, 5, 3, 0); break;
		case 0x1c0: ret = BITSWAP16(offset ^ 0x0090, 15, 14, 13, 12, 11, 10, 5, 3, 7, 2, 1, 4, 0, 9, 8, 6); break;
		case 0x200: ret = BITSWAP16(offset ^ 0x02a1, 15, 14, 13, 12, 11, 10, 9, 1, 7, 8, 5, 6, 2, 4, 3, 0); break;
		case 0x240: ret = BITSWAP16(offset ^ 0x0000, 15, 14, 13, 12, 11, 10, 3, 2, 1, 0, 9, 8, 7, 6, 5, 4); break;
		case 0x280: ret = BITSWAP16(offset ^ 0x02a1, 15, 14, 13, 12, 11, 10, 9, 1, 7, 8, 5, 6, 2, 4, 3, 0); break;
		case 0x2c0: ret = BITSWAP16(offset ^ 0x0000, 15, 14, 13, 12, 11, 10, 0, 3, 4, 6, 8, 7, 5, 9, 2, 1); break;
		case 0x300: ret = BITSWAP16(offset ^ 0x03c0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 1, 2, 0, 5, 3, 4); break;
		case 0x340: ret = BITSWAP16(offset ^ 0x0030, 15, 14, 13, 12, 11, 10, 0, 2, 3, 9, 5, 4, 8, 7, 6, 1); break;
		case 0x380: ret = BITSWAP16(offset ^ 0x0001, 15, 14, 13, 12, 11, 10, 6, 5, 4, 3, 2, 1, 9, 8, 7, 0); break;
		case 0x3c0: ret = BITSWAP16(offset ^ 0x0090, 15, 14, 13, 12, 11, 10, 5, 3, 7, 2, 1, 4, 0, 9, 8, 6); break;
		case 0x400: ret = BITSWAP16(offset ^ 0x02a1, 15, 14, 13, 12, 11, 10, 9, 1, 7, 8, 5, 6, 2, 4, 3, 0); break;
		case 0x440: ret = BITSWAP16(offset ^ 0x03c0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 1, 2, 0, 5, 3, 4); break;
		case 0x480: ret = BITSWAP16(offset ^ 0x0141, 15, 14, 13, 12, 11, 10, 4, 8, 2, 6, 1, 7, 9, 5, 3, 0); break;
		case 0x4c0: ret = BITSWAP16(offset ^ 0x01c0, 15, 14, 13, 12, 11, 10, 2, 8, 7, 6, 4, 3, 5, 9, 0, 1); break;
		case 0x500: ret = BITSWAP16(offset ^ 0x0141, 15, 14, 13, 12, 11, 10, 4, 8, 2, 6, 1, 7, 9, 5, 3, 0); break;
		case 0x540: ret = BITSWAP16(offset ^ 0x0030, 15, 14, 13, 12, 11, 10, 0, 2, 3, 9, 5, 4, 8, 7, 6, 1); break;
		case 0x580: ret = BITSWAP16(offset ^ 0x03c0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 1, 2, 0, 5, 3, 4); break;
		case 0x5c0: ret = BITSWAP16(offset ^ 0x0090, 15, 14, 13, 12, 11, 10, 5, 3, 7, 2, 1, 4, 0, 9, 8, 6); break;
		case 0x600: ret = BITSWAP16(offset ^ 0x0000, 15, 14, 13, 12, 11, 10, 0, 3, 4, 6, 8, 7, 5, 9, 2, 1); break;
		case 0x640: ret = BITSWAP16(offset ^ 0x0000, 15, 14, 13, 12, 11, 10, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5); break;
	}

	if (offset >= 0xce80/2 && offset <= 0xceff/2) ret -= 0x0100;
	if (offset >= 0xcf00/2 && offset <= 0xcf7f/2) ret += 0x0100;

	return ret;
}

/* preliminary */
static void command_handler_py2k2(int pc)
{
	switch (m_ddp3lastcommand)
	{
		case 0x30:
			m_valueresponse = py2k2_sprite_offset(m_py2k2_sprite_base, m_py2k2_sprite_pos++);
		break;
		case 0x32:
			m_py2k2_sprite_base = m_value0;
			m_py2k2_sprite_pos = 0;
			m_valueresponse = py2k2_sprite_offset(m_py2k2_sprite_base, m_py2k2_sprite_pos++);
		break;
		case 0xba:
			m_valueresponse = m_py2k2_prev_base;
			m_py2k2_prev_base = m_value0;
		break;
		case 0x99: // reset?
			m_py2k2_prev_base = m_value0;
			m_simregion = pgm_region_port();
			m_valuekey = 0x100;
			m_valueresponse = 0x00880000 | m_simregion<<8;
		break;
		case 0xc0:
			puzzli2_printf("%06x command %02x | %04x\n", pc, m_ddp3lastcommand, m_value0);
			m_valueresponse = 0x880000;
			break;
		case 0xc3:
			m_valueresponse = 0x904000 + ((m_extra_ram[0xc0] + (m_value0 * 0x40)) * 4);
		break;
		case 0xd0:
			m_valueresponse = 0xa01000 + (m_value0 * 0x20);
		break;
		case 0xdc:
			m_valueresponse = 0xa00800 + (m_value0 * 0x40);
		break;
		case 0xe0:
			m_valueresponse = 0xa00000 + ((m_value0 & 0x1f) * 0x40);
		break;
		case 0xcb: // Background layer 'x' select (pgm3in1, same as kov)
			m_valueresponse = 0x880000;
		break;
		case 0xcc: // Background layer offset (pgm3in1, same as kov)
		{
			int y = m_value0;
			if (y & 0x400) y = -(0x400 - (y & 0x3ff));
			m_valueresponse = 0x900000 + ((m_extra_ram[0xcb] + (y * 0x40)) * 4);
		}
		break;
		case 0x33:
		case 0x34:
		case 0x35:
		case 0x37:
		case 0x38:
		default:
			puzzli2_printf("%06x command %02x | %04x\n", pc, m_ddp3lastcommand, m_value0);
			m_valueresponse = 0x880000;
			break;
	}
}

/* preliminary */


static const int pstar_ba[0x1E]={
	0x02,0x00,0x00,0x01,0x00,0x03,0x00,0x00, //0
	0x02,0x00,0x06,0x00,0x22,0x04,0x00,0x03, //8
	0x00,0x00,0x06,0x00,0x20,0x07,0x00,0x03, //10
	0x00,0x21,0x01,0x00,0x00,0x63
};

static const int pstar_b0[0x10]={
	0x09,0x0A,0x0B,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x00,0x00,0x00,0x00
};

static const int pstar_ae[0x10]={
	0x5D,0x86,0x8C ,0x8B,0xE0,0x8B,0x62,0xAF,
	0xB6,0xAF,0x10A,0xAF,0x00,0x00,0x00,0x00
};

static const int pstar_a0[0x10]={
	0x02,0x03,0x04,0x05,0x06,0x01,0x0A,0x0B,
	0x0C,0x0D,0x0E,0x09,0x00,0x00,0x00,0x00,
};

static const int pstar_9d[0x10]={
	0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

static const int pstar_90[0x10]={
	0x0C,0x10,0x0E,0x0C,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
static const int pstar_8c[0x23]={
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,
	0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,
	0x02,0x02,0x03,0x03,0x03,0x04,0x04,0x04,
	0x03,0x03,0x03
};

static const int pstar_80[0x1a3]={
	0x03,0x03,0x04,0x04,0x04,0x04,0x05,0x05,
	0x05,0x05,0x06,0x06,0x03,0x03,0x04,0x04,
	0x05,0x05,0x05,0x05,0x06,0x06,0x07,0x07,
	0x03,0x03,0x04,0x04,0x05,0x05,0x05,0x05,
	0x06,0x06,0x07,0x07,0x06,0x06,0x06,0x06,
	0x06,0x06,0x06,0x07,0x07,0x07,0x07,0x07,
	0x06,0x06,0x06,0x06,0x06,0x06,0x07,0x07,
	0x07,0x07,0x08,0x08,0x05,0x05,0x05,0x05,
	0x05,0x05,0x05,0x06,0x06,0x06,0x07,0x07,
	0x06,0x06,0x06,0x07,0x07,0x07,0x08,0x08,
	0x09,0x09,0x09,0x09,0x07,0x07,0x07,0x07,
	0x07,0x08,0x08,0x08,0x08,0x09,0x09,0x09,
	0x06,0x06,0x07,0x07,0x07,0x08,0x08,0x08,
	0x08,0x08,0x09,0x09,0x05,0x05,0x06,0x06,
	0x06,0x07,0x07,0x08,0x08,0x08,0x08,0x09,
	0x07,0x07,0x07,0x07,0x07,0x08,0x08,0x08,
	0x08,0x09,0x09,0x09,0x06,0x06,0x07,0x03,
	0x07,0x06,0x07,0x07,0x08,0x07,0x05,0x04,
	0x03,0x03,0x04,0x04,0x05,0x05,0x06,0x06,
	0x06,0x06,0x06,0x06,0x03,0x04,0x04,0x04,
	0x04,0x05,0x05,0x06,0x06,0x06,0x06,0x07,
	0x04,0x04,0x05,0x05,0x06,0x06,0x06,0x06,
	0x06,0x07,0x07,0x08,0x05,0x05,0x06,0x07,
	0x07,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
	0x05,0x05,0x05,0x07,0x07,0x07,0x07,0x07,
	0x07,0x08,0x08,0x08,0x08,0x08,0x09,0x09,
	0x09,0x09,0x03,0x04,0x04,0x05,0x05,0x05,
	0x06,0x06,0x07,0x07,0x07,0x07,0x08,0x08,
	0x08,0x09,0x09,0x09,0x03,0x04,0x05,0x05,
	0x04,0x03,0x04,0x04,0x04,0x05,0x05,0x04,
	0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
	0x03,0x03,0x03,0x04,0x04,0x04,0x04,0x04,
	0x04,0x04,0x04,0x04,0x04,0x03,0x03,0x03,
	0x03,0x03,0x03,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,
	0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00
};

static void command_handler_pstars(int pc)
{
	switch (m_ddp3lastcommand)
	{
		case 0x99:
			m_simregion = pgm_region_port();
			m_valuekey = 0x100;
			m_valueresponse = 0x00880000 | m_simregion<<8;
			break;

		case 0xe0:
			m_valueresponse = 0xa00000 + (m_value0 << 6);
			break;

		case 0xdc:
			m_valueresponse = 0xa00800 + (m_value0 << 6);
			break;

		case 0xd0:
			m_valueresponse = 0xa01000 + (m_value0 << 5);
			break;

		case 0xb1:
			m_pstar_b1_value = m_value0;
			m_valueresponse = 0x890000;
			break;

		case 0xbf:
			m_valueresponse = m_pstar_b1_value * m_value0;
			break;

		case 0xc1: //TODO:TIMER  0,1,2,FIX TO 0 should be OK?
			m_valueresponse = 0;
			break;

		case 0xce: //TODO:TIMER  0,1,2
			m_pstar_ce_value = m_value0;
			m_valueresponse=0x890000;
			break;

		case 0xcf: //TODO:TIMER  0,1,2
			m_extra_ram[m_pstar_ce_value & 0xff] = m_value0;   // (masked: MAME indexes past the array)
			m_valueresponse = 0x890000;
			break;

		case 0xe7:
			m_pstar_e7_value = (m_value0 >> 12) & 0xf;
			m_slots[m_pstar_e7_value] &= 0xffff;
			m_slots[m_pstar_e7_value] |= (m_value0 & 0xff) << 16;
			m_valueresponse = 0x890000;
			break;

		case 0xe5:
			m_slots[m_pstar_e7_value] &= 0xff0000;
			m_slots[m_pstar_e7_value] |= m_value0;
			m_valueresponse = 0x890000;
			break;

		case 0xf8: //@73C
			m_valueresponse = m_slots[m_value0 & 0xf] & 0xffffff;
			break;

		case 0xba:
			m_valueresponse = TBL(pstar_ba, m_value0);
			break;

		case 0xb0:
			m_valueresponse = TBL(pstar_b0, m_value0);
			break;

		case 0xae:
			m_valueresponse = TBL(pstar_ae, m_value0);
			break;

		case 0xa0:
			m_valueresponse = TBL(pstar_a0, m_value0);
			break;

		case 0x9d:
			m_valueresponse = TBL(pstar_9d, m_value0);
			break;

		case 0x90:
			m_valueresponse = TBL(pstar_90, m_value0);
			break;

		case 0x8c:
			m_valueresponse = TBL(pstar_8c, m_value0);
			break;

		case 0x80:
			m_valueresponse = TBL(pstar_80, m_value0);
			break;

		default:
			m_valueresponse = 0x890000;
			sim_log("PSTARS PC(%06x) UNKNOWN %4X %4X\n", pc, m_value1, m_value0);
	}
}

/* Old KOV and bootlegs sim ... really these should be read out... */

static const u8 kov_BATABLE[0x40] = {
	0x00,0x29,0x2c,0x35,0x3a,0x41,0x4a,0x4e,0x57,0x5e,0x77,0x79,0x7a,0x7b,0x7c,0x7d,
	0x7e,0x7f,0x80,0x81,0x82,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x90,
	0x95,0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9e,0xa3,0xd4,0xa9,0xaf,0xb5,0xbb,0xc1
};

static const u8 kov_B0TABLE[16] = { 2, 0, 1, 4, 3 }; // Maps char portraits to tables


static void command_handler_kov(int pc)
{
	switch (m_ddp3lastcommand)
	{
		case 0x67: // unknown or status check?
		case 0x8e:
		case 0xa3:
		case 0x33: // kovsgqyz (a3)
		case 0x3a: // kovplus
		case 0xc5: // kovplus
			m_valueresponse = 0x880000;
		break;

		case 0x99: // Reset
			m_simregion = pgm_region_port();
			m_valueresponse = 0x880000 | m_simregion<<8;
			m_valuekey = 0x100;
		break;

		case 0x9d: // Sprite palette offset
			m_valueresponse = 0xa00000 + ((m_value0 & 0x1f) * 0x40);
		break;

		case 0xb0: // Read from data table
			m_valueresponse = kov_B0TABLE[m_value0 & 0x0f];
		break;

		case 0xb4: // Copy slot 'a' to slot 'b'
		case 0xb7: // kovsgqyz (b4)
		{
			m_valueresponse = 0x880000;

			if (m_value0 == 0x0102) m_value0 = 0x0100; // why?

			m_slots[(m_value0 >> 8) & 0x0f] = m_slots[(m_value0 >> 0) & 0x0f];
		}
		break;

		case 0xba: // Read from data table
			m_valueresponse = kov_BATABLE[m_value0 & 0x3f];
		break;

		case 0xc0: // Text layer 'x' select
			m_valueresponse = 0x880000;
			m_kov_c0_value = m_value0;
		break;

		case 0xc3: // Text layer offset
			m_valueresponse = 0x904000 + ((m_kov_c0_value + (m_value0 * 0x40)) * 4);
		break;

		case 0xcb: // Background layer 'x' select
			m_valueresponse = 0x880000;
			m_kov_cb_value = m_value0;
		break;

		case 0xcc: // Background layer offset
		{
			int y = m_value0;
			if (y & 0x400) y = -(0x400 - (y & 0x3ff));
			m_valueresponse = 0x900000 + ((m_kov_cb_value + (y * 0x40)) * 4);
		}
		break;

		case 0xd0: // Text palette offset
		case 0xcd: // kovsgqyz (d0)
			m_valueresponse = 0xa01000 + (m_value0 * 0x20);
		break;

		case 0xd6: // Copy slot to slot 0
			m_valueresponse = 0x880000;
			m_slots[0] = m_slots[m_value0 & 0x0f];
		break;

		case 0xdc: // Background palette offset
		case 0x11: // kovsgqyz (dc)
			m_valueresponse = 0xa00800 + (m_value0 * 0x40);
		break;

		case 0xe0: // Sprite palette offset
		case 0x9e: // kovsgqyz (e0)
			m_valueresponse = 0xa00000 + ((m_value0 & 0x1f) * 0x40);
		break;

		case 0xe5: // Write slot (low)
		{
			m_valueresponse = 0x880000;

			s32 sel = (m_curslots >> 12) & 0x0f;
			m_slots[sel] = (m_slots[sel] & 0x00ff0000) | ((m_value0 & 0xffff) <<  0);
		}
		break;

		case 0xe7: // Write slot (and slot select) (high)
		{
			m_valueresponse = 0x880000;
			m_curslots = m_value0;

			s32 sel = (m_curslots >> 12) & 0x0f;
			m_slots[sel] = (m_slots[sel] & 0x0000ffff) | ((m_value0 & 0x00ff) << 16);
		}
		break;

		case 0xf0: // Some sort of status read?
			m_valueresponse = 0x00c000;
		break;

		case 0xf8: // Read slot
		case 0xab: // kovsgqyz (f8)
			m_valueresponse = m_slots[m_value0 & 0x0f] & 0x00ffffff;
		break;

		case 0xfc: // Adjust damage level to char experience level
			m_valueresponse = (m_value0 * m_kov_fe_value) >> 6;
		break;

		case 0xfe: // Damage level adjust
			m_valueresponse = 0x880000;
			m_kov_fe_value = m_value0;
		break;

		default:
			m_valueresponse = 0x880000;
//                  sim_log("Unknown ASIC27 command: %2.2x data: %4.4x\n", (data ^ m_valuekey) & 0xff, m_value0);
		break;
	}
}


/* Oriental Legend Super Plus ARM simulation */

static const int oldsplus_80[0x5]={
	0xbb8,0x1770,0x2328,0x2ee0,0xf4240
};

static const int oldsplus_fc[0x20]={
	0x00,0x00,0x0a,0x3a,0x4e,0x2e,0x03,0x40,
	0x33,0x43,0x26,0x2c,0x00,0x00,0x00,0x00,
	0x00,0x00,0x44,0x4d,0xb,0x27,0x3d,0x0f,
	0x37,0x2b,0x02,0x2f,0x15,0x45,0x0e,0x30
};

static const int oldsplus_a0[0x20]={
	0x000,0x023,0x046,0x069,0x08c,0x0af,0x0d2,0x0f5,
	0x118,0x13b,0x15e,0x181,0x1a4,0x1c7,0x1ea,0x20d,
	0x20d,0x20d,0x20d,0x20d,0x20d,0x20d,0x20d,0x20d,
	0x20d,0x20d,0x20d,0x20d,0x20d,0x20d,0x20d,0x20d,
};

static const int oldsplus_90[0x7]={
	0x50,0xa0,0xc8,0xf0,0x190,0x1f4,0x258
};

static const int oldsplus_5e[0x20]={
	0x04,0x04,0x04,0x04,0x04,0x03,0x03,0x03,
	0x02,0x02,0x02,0x01,0x01,0x01,0x01,0x01,
	0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

static const int oldsplus_b0[0xe0]={
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,
	0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,0x14,

	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
	0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,
	0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,

	0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,
	0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,
	0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,0x14,
	0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,

	0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
	0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
	0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,
	0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,

	0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,
	0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,
	0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,
	0x1c,0x1d,0x1e,0x1f,0x1f,0x1f,0x1f,0x1f
};

static const int oldsplus_ae[0xe0]={
	0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,
	0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,
	0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,
	0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,

	0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,
	0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,
	0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,
	0x50,0x50,0x50,0x50,0x50,0x50,0x50,0x50,

	0x1E,0x1F,0x20,0x21,0x22,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,

	0x1F,0x20,0x21,0x22,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,

	0x20,0x21,0x22,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,

	0x21,0x22,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,

	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23,
	0x23,0x23,0x23,0x23,0x23,0x23,0x23,0x23
};

static const int oldsplus_ba[0x4]={
	0x3138,0x2328,0x1C20,0x1518
};

static const int oldsplus_9d[0x111]={
	0x0000,0x0064,0x00c8,0x012c,0x0190,0x01f4,0x0258,0x02bc,
	0x02f8,0x0334,0x0370,0x03ac,0x03e8,0x0424,0x0460,0x049c,
	0x04d8,0x0514,0x0550,0x058c,0x05c8,0x0604,0x0640,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x0000,
	0x0064,0x00c8,0x012c,0x0190,0x01f4,0x0258,0x02bc,0x0302,
	0x0348,0x038e,0x03d4,0x041a,0x0460,0x04a6,0x04ec,0x0532,
	0x0578,0x05be,0x0604,0x064a,0x0690,0x06d6,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x0000,0x0064,
	0x00c8,0x012c,0x0190,0x01f4,0x0258,0x02bc,0x0316,0x0370,
	0x03ca,0x0424,0x047e,0x04d8,0x0532,0x058c,0x05e6,0x0640,
	0x069a,0x06f4,0x074e,0x07a8,0x0802,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x0000,0x0064,0x00c8,
	0x012c,0x0190,0x01f4,0x0258,0x02bc,0x032a,0x0398,0x0406,
	0x0474,0x04e2,0x0550,0x05be,0x062c,0x069a,0x0708,0x0776,
	0x07e4,0x0852,0x08c0,0x092e,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x0000,0x0064,0x00c8,0x012c,
	0x0190,0x01f4,0x0258,0x02bc,0x0348,0x03d4,0x0460,0x04ec,
	0x0578,0x0604,0x0690,0x071c,0x07a8,0x0834,0x08c0,0x094c,
	0x09d8,0x0a64,0x0af0,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x0000,0x0064,0x00c8,0x012c,0x0190,
	0x01f4,0x0258,0x02bc,0x0384,0x044c,0x0514,0x05dc,0x06a4,
	0x076c,0x0834,0x08fc,0x09c4,0x0a8c,0x0b54,0x0c1c,0x0ce4,
	0x0dac,0x0e74,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x0000,0x0064,0x00c8,0x012c,0x0190,0x01f4,
	0x0258,0x02bc,0x030c,0x035c,0x03ac,0x03fc,0x044c,0x049c,
	0x04ec,0x053c,0x058c,0x05dc,0x062c,0x067c,0x06cc,0x071c,
	0x076c,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,0x06bc,
	0x06bc
};

static const int oldsplus_8c[0x20]={
	0x0032,0x0032,0x0064,0x0096,0x0096,0x00fa,0x012c,0x015e,
	0x0032,0x0064,0x0096,0x00c8,0x00c8,0x012c,0x015e,0x0190,
	0x0064,0x0096,0x00c8,0x00fa,0x00fa,0x015e,0x0190,0x01c2,
	0x0096,0x00c8,0x00fa,0x012c,0x012c,0x0190,0x01c2,0x01f4
};


static void command_handler_oldsplus(int pc)
{
	switch (m_ddp3lastcommand)
	{
		case 0x88:
			m_simregion = pgm_region_port();
			m_valuekey = 0x100;
			m_valueresponse = 0x00990000 | m_simregion<<8;

			break;

		case 0xd0:
			m_valueresponse = 0xa01000 + (m_value0 << 5);
			break;

		case 0xc0:
			m_valueresponse = 0xa00000 + (m_value0 << 6);
			break;

		case 0xc3:
			m_valueresponse = 0xa00800 + (m_value0 << 6);
			break;

		case 0x36:
			m_extra_ram[0x36] = m_value0;
			m_valueresponse = 0x990000;
			break;

		case 0x33:
			m_extra_ram[0x33] = m_value0;
			m_valueresponse = 0x990000;
			break;

		case 0x35:
			m_extra_ram[0x36] += m_value0;
			m_valueresponse = 0x990000;
			break;

		case 0x37:
			m_extra_ram[0x33] += m_value0;
			m_valueresponse = 0x990000;
			break;

		case 0x34:
			m_valueresponse = m_extra_ram[0x36];
			break;

		case 0x38:
			m_valueresponse = m_extra_ram[0x33];
			break;

		case 0x80:
			m_valueresponse = TBL(oldsplus_80, m_value0);
			break;

		case 0xe7:
			m_extra_ram[0xe7] = m_value0;
			m_valueresponse = 0x990000;
			break;

		case 0xe5:
			switch (m_extra_ram[0xe7])
			{
				case 0xb000:
					m_slots[0xb] = m_value0;
					m_slots[0xc] = 0;
					break;

				case 0xc000:
					m_slots[0xc] = m_value0;
					break;

				case 0xd000:
					m_slots[0xd] = m_value0;
					break;

				case 0xf000:
					m_slots[0xf] = m_value0;
					break;
			}
			m_valueresponse = 0x990000;
			break;

		case 0xf8:
			m_valueresponse = TBL(m_slots, m_value0);
			break;

		case 0xfc:
			m_valueresponse = TBL(oldsplus_fc, m_value0);
			break;

		case 0xc5:
			m_slots[0xd] --;
			m_valueresponse = 0x990000;
			break;

		case 0xd6:
			m_slots[0xb] ++;
			m_valueresponse = 0x990000;
			break;

		case 0x3a:
			m_slots[0xf] = 0;
			m_valueresponse = 0x990000;
			break;

		case 0xf0:
			m_extra_ram[0xf0] = m_value0;
			m_valueresponse = 0x990000;
			break;

		case 0xed:
			m_valueresponse = m_value0 << 0x6;
			m_valueresponse += m_extra_ram[0xf0];
			m_valueresponse = m_valueresponse << 0x2;
			m_valueresponse += 0x900000;
			break;

		case 0xe0:
			m_extra_ram[0xe0] = m_value0;
			m_valueresponse = 0x990000;
			break;

		case 0xdc:
			m_valueresponse = m_value0 << 0x6;
			m_valueresponse += m_extra_ram[0xe0];
			m_valueresponse = m_valueresponse << 0x2;
			m_valueresponse += 0x904000;
			break;

		case 0xcb:
			m_valueresponse =  0xc000;
			break;

		case 0xa0:
			m_valueresponse = TBL(oldsplus_a0, m_value0);
			break;

		case 0xba:
			m_valueresponse = TBL(oldsplus_ba, m_value0);
			break;

		case 0x5e:
			m_valueresponse = TBL(oldsplus_5e, m_value0);
			break;

		case 0xb0:
			m_valueresponse = TBL(oldsplus_b0, m_value0);
			break;

		case 0xae:
			m_valueresponse = TBL(oldsplus_ae, m_value0);
			break;

		case 0x9d:
			m_valueresponse = TBL(oldsplus_9d, m_value0);
			break;

		case 0x90:
			m_valueresponse = TBL(oldsplus_90, m_value0);
			break;

		case 0x8c:
			m_valueresponse = TBL(oldsplus_8c, m_value0);
			break;

		default:
			m_valueresponse = 0x990000;
			sim_printf("%06X: oldsplus_UNKNOWN W CMD %X  VAL %X\n", pc,m_value1,m_value0);
			break;
	}
}

static void arm7_type1_sim_w(offs_t offset, u16 data)
{
	const int pc = 0;

	if (offset == 0)
	{
		m_value0 = data;
		return;
	}
	else if (offset == 1)
	{
		u16 realkey;
		if ((data >> 8) == 0xff)
			m_valuekey = 0xff00;
		realkey = m_valuekey >> 8;
		realkey |= m_valuekey;
		{
			m_valuekey += 0x0100;
			m_valuekey &= 0xff00;
			if (m_valuekey == 0xff00)
				m_valuekey =  0x0100;
		}
		data ^= realkey;
		m_value1 = data;
		m_value0 ^= realkey;

		m_ddp3lastcommand = m_value1 & 0xff;

		arm_sim_handler(pc);
	}
	else if (offset == 2)
	{
	}
}

static u16 arm7_type1_sim_protram_r(offs_t offset)
{
	if (offset == 4)
		return m_simregion;

	return 0x0000;
}

static u16 pstars_arm7_type1_sim_protram_r(offs_t offset)
{
	if (offset == 4)        //region
		return pgm_region_port();
	else if (offset >= 0x10)  //timer
	{
		const u16 val = m_extra_ram[offset - 0x10];
		if (!0)
		{
			sim_log("PSTARS ACCESS COUNTER %6X\n", val);
			m_extra_ram[offset - 0x10]--;
		}
		return val;
	}
	return 0x0000;
}


static int count_bits(u16 value)
{
	int count = 0;
	for (int i = 0; i < 16; i++)
	{
		int bit = (value >> i) & 1;

		if (bit) count++;
	}

	return count;
}

static int get_position_of_bit(u16 value, int bit_wanted)
{
	int count = 0;
	for (int i = 0; i < 16; i++)
	{
		int bit = (value >> i) & 1;

		if (bit) count++;

		if (count==(bit_wanted+1))
			return i;
	}

	return -1;
}

static int puzzli2_take_leveldata_value(u8 datvalue)
{
	if (stage == -1)
	{
		entries_left = 0;
		currentcolumn = 0;
		currentrow = 0;
		num_entries = 0;
		full_entry = 0;
		prev_tablloc = 0;
		numbercolumns = 0;
		depth = 0;
		m_row_bitmask = 0;

		puzzli2_printf("%02x <- table offset\n", datvalue);
		tableoffs = datvalue;
		tableoffs2 = 0;
		stage = 0;
	}
	else
	{
		u8 rawvalue = datvalue;
		u8 tableloc = (tableoffs+tableoffs2) & 0xff;
		rawvalue ^= puzzli2_level_decode[tableloc];

		tableoffs2++;
		tableoffs2 &= 0xf;

		if (stage == 0)
		{
			stage = 1;

			// this seems to be the first thing returned back when reading the level structure always seems to be 0x8 or 0x7, makes sense, levels would be too difficult otherwise ;-) (actually puzzli2 seems to have one specifying 5 unless it's a decrypt table error?!)
			depth = (rawvalue & 0xf0);
			numbercolumns = (rawvalue & 0x0f);
			numbercolumns++;

			puzzli2_printf("%02x <- Sizes (level depth %01x) (number of columns %01x)", rawvalue, depth>>4, numbercolumns);

			if ((depth != 0x80) && (depth != 0x70) && (depth != 0x50))
				sim_log("depth isn't 0x5, 0x7 or 0x8");

			// it seems to use this to specify the number of columns, ie how many data structures follow, so that the ARM knows when to send back the 'finished' flag.
			// it also gets returned at the end with said flag
			if ((numbercolumns != 0x6) && (numbercolumns != 0x7) && (numbercolumns != 0x8))
				sim_log("number of columns specified isn't 6,7, or 8");

			puzzli2_printf("\n");

		}
		else if (stage == 1)
		{
			puzzli2_printf("%02x <- Number of Entries for this Column (and upper mask) (column is %d) (xor table location is %02x) ", rawvalue, currentcolumn, tableloc);
			stage = 2;
			entries_left = (rawvalue >> 4);
			m_row_bitmask = (rawvalue & 0x0f)<<8;

			full_entry = rawvalue;
			prev_tablloc = tableloc;

			num_entries = entries_left;

			if (num_entries == 0x00)
			{
				puzzli2_printf("0 entries for this column?"); // seems a valid condition based on the data
			}

			puzzli2_printf("\n");
		}
		else if (stage == 2)
		{
			puzzli2_printf("%02x <- Mask value equal to number of entries (xor table location is %02x)", rawvalue, tableloc);
			stage = 3;

			m_row_bitmask |= rawvalue;

			int num_mask_bits = count_bits(m_row_bitmask);

			if (num_mask_bits != num_entries)
				puzzli2_printf(" error - number of mask bits != number of entries - ");

			//
			if (entries_left == 0)
			{
				// for 0 entries skip back to state 1 instead of 3, because there is nothing following
				stage = 1;
				currentcolumn++;
				currentrow = 0;
				m_row_bitmask = 0;

				coverage[tableloc] = 1;
				if (rawvalue != 0)
				{
					puzzli2_printf(" invalid mask after 00 length?");
					// attempt to correct the table
					//new_puzzli2_level_decode[tableloc] = new_puzzli2_level_decode[tableloc] ^ rawvalue;

				}
				coverage[prev_tablloc] = 1;
				if (full_entry != 0)
				{
					puzzli2_printf(" previous value wasn't 0x00");
				}

				if (currentcolumn == numbercolumns)
				{
					return 1;
				}

			}
			else
			{
				if (num_entries> 0xa)
				{
					puzzli2_printf(" more than 10 entries?");
				}
				else
				{
					// this isn't a strict rule
					// the mask is used so they can specify spaces between elements too without storing the 00 bytes
					coverage[tableloc] = 1;

					int desired_mask = 0;

					if (num_entries == 0x00) desired_mask = 0x00;
					if (num_entries == 0x01) desired_mask = 0x01;
					if (num_entries == 0x02) desired_mask = 0x03;
					if (num_entries == 0x03) desired_mask = 0x07;
					if (num_entries == 0x04) desired_mask = 0x0f;
					if (num_entries == 0x05) desired_mask = 0x1f;
					if (num_entries == 0x06) desired_mask = 0x3f;
					if (num_entries == 0x07) desired_mask = 0x7f;
					if (num_entries == 0x08) desired_mask = 0xff;
					if (num_entries == 0x09) desired_mask = 0xff;
					if (num_entries == 0x0a) desired_mask = 0xff;

					if (rawvalue != desired_mask)
					{
						puzzli2_printf(" possible wrong mask?");
					}

				}

			}

			puzzli2_printf("\n");

		}
		else if (stage == 3)
		{
			u16 object_value;

			// return values
			// 0x0100 = normal fish
			// 0x0120 = fish in bubble
			// 0x0140 = fish in egg
			// 0x0160 = buggy fish in egg (displayed as normal fish)
			// 0x0180 = fish on hook
			// 0x01a0 = fish on hook (uncatchable)
			// 0x01c0 - fish on hook (uncatchable)
			// 0x01e0 - fish on hook (uncatchable)

			// fish values
			// 100 101 102 103 104 105 106 107 108 normal
			// 110 - renders as a flashing fish you can't catch? glitches game?
			// 111 - repeat of other fish type you can't catch...

			if (rawvalue <= 0x10) // regular fish
			{
				int fishtype = rawvalue;
				puzzli2_printf("%02x <- fish type %d", rawvalue, fishtype);
				object_value = 0x0100 + fishtype;
				// 0x110 is a flashy fish? might be glitchy and need a special number..
			}
			else if (rawvalue <= 0x21) // fish in bubbles
			{
				int fishtype = rawvalue - 0x11;
				puzzli2_printf("%02x <- fish in bubble %d", rawvalue, fishtype);
				object_value = 0x0120 + fishtype;
				// 0x130 is a flashy fish? might be glitchy and need a special number..
			}
			else if (rawvalue <= 0x32) // fish in eggs
			{
				int fishtype = rawvalue - 0x22;
				puzzli2_printf("%02x <- fish in egg %d", rawvalue, fishtype);
				object_value = 0x0140 + fishtype;
				// 0x150 is a flashy fish? might be glitchy and need a special number..

			}
			else if (rawvalue <= 0x43) // fish on hook cases, seem to be base 0x180
			{
				int fishtype = rawvalue - 0x33;
				puzzli2_printf("%02x <- fish on hook %d", rawvalue, fishtype);
				object_value = 0x0180 + fishtype;
				// 0x190 is a flashy fish? might be glitchy and need a special number..

			}
			////////////////////// special objects follow
			else if (rawvalue == 0xd0) {object_value = 0x0200; puzzli2_printf("%02x <- generic bubbles", rawvalue);}

			else if (rawvalue == 0xe0) {object_value = 0x8000; puzzli2_printf("%02x <- solid middle", rawvalue);}
			else if (rawvalue == 0xe1) {object_value = 0x8020; puzzli2_printf("%02x <- solid top slant down", rawvalue);} // solid slant top down
			else if (rawvalue == 0xe2) {object_value = 0x8040; puzzli2_printf("%02x <- solid top slant up", rawvalue);} // solid slant top up
			else if (rawvalue == 0xe3) {object_value = 0x8060; puzzli2_printf("%02x <- solid bottom slant up", rawvalue);}
			else if (rawvalue == 0xe4) {object_value = 0x8080; puzzli2_printf("%02x <- solid bottom slant down", rawvalue);} // sold slant bottom up

			else                     {object_value = 0xffff; puzzli2_printf("%02x <- unknown object", rawvalue);}

			puzzli2_printf("  (xor table location is %02x)\n",tableloc);

			if (object_value == 0xffff)
			{
				object_value = 0x110;
				sim_log("unknown object type %02x\n", rawvalue);
			}

			const int realrow = get_position_of_bit(m_row_bitmask, currentrow);

			if (realrow != -1 && currentcolumn < 8 && realrow < 10)   // (bounds: MAME stops at the fatalerror above)
				level_structure[currentcolumn][realrow] = object_value;

			currentrow++;

			entries_left--;
			if (entries_left == 0)
			{
				stage = 1;
				currentcolumn++;
				currentrow = 0;
				m_row_bitmask = 0;

				if (currentcolumn == numbercolumns)
				{
					return 1;
				}

			}
		}

	}

	return 0;
}



// (MAME's member names end here)
#undef m_value0
#undef m_value1
#undef m_valuekey
#undef m_ddp3lastcommand
#undef m_valueresponse
#undef m_curslots
#undef m_slots
#undef m_pstar_e7_value
#undef m_pstar_b1_value
#undef m_pstar_ce_value
#undef m_kov_c0_value
#undef m_kov_cb_value
#undef m_kov_fe_value
#undef m_extra_ram
#undef m_puzzli_54_trigger
#undef m_simregion
#undef stage
#undef tableoffs
#undef tableoffs2
#undef entries_left
#undef currentcolumn
#undef currentrow
#undef num_entries
#undef full_entry
#undef prev_tablloc
#undef numbercolumns
#undef depth
#undef m_row_bitmask
#undef hackcount
#undef hackcount2
#undef hack_47_value
#undef hack_31_table_offset
#undef hack_31_table_offset2
#undef p2_31_retcounter
#undef coverage
#undef command_31_write_type
#undef level_structure
#undef m_py2k2_sprite_pos
#undef m_py2k2_sprite_base
#undef m_py2k2_prev_base

/**************************** EMULATION *******************************/
/* used by photoy2k, kovsh */

enum { MODE_ARM, MODE_SIM };
enum { PROTRAM_NONE, PROTRAM_SIM, PROTRAM_PSTARS };

static int s_mode;
static int s_cave;            // pgm_arm_type1_cave: no game ROM bank past 0x3fffff
static int s_fake_region;     // kovsh_fake_region_r at 0x4f0008
static int s_kovshp_w;        // kovshp_asic27a_write_word at 0x500000-0x500005
static uint32_t s_sim_base;   // arm7_type1_sim_r / _w at base-base+5 (0x500000, the Cave games 0x400000)
static int s_protram;         // the simulations' read handler at 0x4f0000-0x4f003f
static arm7_t *s_arm;
static uint8_t *s_irom;
static uint32_t s_irom_size;

// ---- ARM side (55857E_arm7_map) ----
static uint32_t arm_r32(void *u, uint32_t a)
{
	(void)u;
	if (a == 0x40000000)   // arm7_type1_protlatch_r
		return S.m_arm_type1_latch_68k_w;
	if (a == 0x4000000c)   // arm7_type1_unk_r
		return S.m_arm_type1_counter++;
	if (a >= 0x50800000 && a <= 0x5080003f)   // arm7_type1_shareram_r
		return S.shareram[(a - 0x50800000) >> 2];
	return 0;   // (0x08100000-0x083fffff: arm7_type1_exrom_r, unpopulated, 0; the rest unmapped)
}
static uint16_t arm_r16(void *u, uint32_t a) { return (uint16_t)(arm_r32(u, a & ~3u) >> ((a & 2) * 8)); }
static uint8_t arm_r8(void *u, uint32_t a) { return (uint8_t)(arm_r32(u, a & ~3u) >> ((a & 3) * 8)); }

static void arm_w32m(uint32_t a, uint32_t d, uint32_t mask)
{
	if (a == 0x40000000) {   // arm7_type1_protlatch_w
		S.m_arm_type1_latch_arm_w = (S.m_arm_type1_latch_arm_w & ~mask) | (d & mask);
		S.m_arm_type1_latch_68k_w &= ~mask;
		return;
	}
	if (a >= 0x50800000 && a <= 0x5080003f) {   // arm7_type1_shareram_w
		uint32_t *w = &S.shareram[(a - 0x50800000) >> 2];
		*w = (*w & ~mask) | (d & mask);
	}
	// (0x40000008: nopw)
}
static void arm_w32(void *u, uint32_t a, uint32_t d) { (void)u; arm_w32m(a, d, 0xffffffffu); }
static void arm_w16(void *u, uint32_t a, uint16_t d)
{
	(void)u;
	const int sh = (a & 2) * 8;
	arm_w32m(a & ~3u, (uint32_t)d << sh, 0xffffu << sh);
}
static void arm_w8(void *u, uint32_t a, uint8_t d)
{
	(void)u;
	const int sh = (a & 3) * 8;
	arm_w32m(a & ~3u, (uint32_t)d << sh, 0xffu << sh);
}

// ---- 68000 side ----
static u16 arm7_type1_68k_protlatch_r(offs_t offset)
{
	return (u16)(offset ? S.m_arm_type1_latch_arm_w >> 16 : S.m_arm_type1_latch_arm_w);
}

static void arm7_type1_68k_protlatch_w(offs_t offset, u16 data)
{
	switch (offset) {
	case 1: S.m_arm_type1_latch_68k_w = (S.m_arm_type1_latch_68k_w & 0x0000ffff) | ((u32)data << 16); break;
	case 0: S.m_arm_type1_latch_68k_w = (S.m_arm_type1_latch_68k_w & 0xffff0000) | data; break;
	}
}

// the 68000's word n is the low half of the ARM's word n (MAME share16[BYTE_XOR_LE(offset << 1)])
static u16 arm7_type1_ram_r(offs_t offset) { return (u16)S.shareram[offset & 0x1f]; }
static void arm7_type1_ram_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 *w = &S.shareram[offset & 0x1f];
	*w = (*w & ~(u32)mem_mask) | (data & mem_mask);
}

static u16 kovsh_fake_region_r(void)
{
	const int regionhack = (int)pgm_region_hack();
	if (regionhack != 0xff) return (u16)regionhack;
	return arm7_type1_ram_r(0x4);
}

/* Fake remapping of ASIC commands to the ones used by KOVSH due to the lack of the real ARM rom for this set */
static void kovshp_asic27a_write_word(offs_t offset, u16 data)
{
	switch (offset) {
	case 0:
		S.m_arm_type1_latch_68k_w = (S.m_arm_type1_latch_68k_w & 0xffff0000) | data;
		return;
	case 1: {
		const u8 asic_key = data >> 8;
		u8 asic_cmd = (data & 0xff) ^ asic_key;
		switch (asic_cmd) {
		case 0x9a: asic_cmd = 0x99; break; // kovshxas

		case 0x38: asic_cmd = 0xad; break;
		case 0x43: asic_cmd = 0xca; break;
		case 0x56: asic_cmd = 0xac; break;
		case 0x73: asic_cmd = 0x93; break;
		case 0x84: asic_cmd = 0xb3; break;
		case 0x87: asic_cmd = 0xb1; break;
		case 0x89: asic_cmd = 0xb6; break;
		case 0x93: asic_cmd = 0x73; break;
		case 0xa5: asic_cmd = 0xa9; break;
		case 0xac: asic_cmd = 0x56; break;
		case 0xad: asic_cmd = 0x38; break;
		case 0xb1: asic_cmd = 0x87; break;
		case 0xb3: asic_cmd = 0x84; break;
		case 0xb4: asic_cmd = 0x90; break;
		case 0xb6: asic_cmd = 0x89; break;
		case 0xc5: asic_cmd = 0x8c; break;
		case 0xca: asic_cmd = 0x43; break;
		case 0xcc: asic_cmd = 0xf0; break;
		case 0xd0: asic_cmd = 0xe0; break;
		case 0xe0: asic_cmd = 0xd0; break;
		case 0xe7: asic_cmd = 0x70; break;
		case 0xed: asic_cmd = 0xcb; break;
		case 0xf0: asic_cmd = 0xcc; break;
		case 0xf1: asic_cmd = 0xf5; break;
		case 0xf2: asic_cmd = 0xf1; break;
		case 0xf4: asic_cmd = 0xf2; break;
		case 0xf5: asic_cmd = 0xf4; break;
		case 0xfc: asic_cmd = 0xc0; break;
		case 0xfe: asic_cmd = 0xc3; break;

		case 0xa6: asic_cmd = 0xa9; break;
		case 0xaa: asic_cmd = 0x56; break;
		case 0xf8: asic_cmd = 0xf3; break;
		}
		S.m_arm_type1_latch_68k_w = (S.m_arm_type1_latch_68k_w & 0x0000ffff) |
		                            ((u32)(asic_cmd ^ (asic_key | (asic_key << 8))) << 16);
		return;
	}
	}
}

static int a1_read16(uint32_t a, uint16_t *v)
{
	a &= ~1u;
	if (s_mode == MODE_ARM) {
		if (s_fake_region && a == 0x4f0008) { *v = kovsh_fake_region_r(); return 1; }
		if (a >= 0x4f0000 && a <= 0x4f003f) { *v = arm7_type1_ram_r((a - 0x4f0000) >> 1); return 1; }
		if (a >= 0x500000 && a <= 0x500003) { *v = arm7_type1_68k_protlatch_r((a - 0x500000) >> 1); return 1; }
		return 0;
	}
	if (a >= s_sim_base && a <= s_sim_base + 5) { *v = arm7_type1_sim_r((a - s_sim_base) >> 1); return 1; }
	if (s_protram != PROTRAM_NONE && a >= 0x4f0000 && a <= 0x4f003f) {
		const offs_t o = (a - 0x4f0000) >> 1;
		*v = s_protram == PROTRAM_PSTARS ? pstars_arm7_type1_sim_protram_r(o) : arm7_type1_sim_protram_r(o);
		return 1;
	}
	return 0;
}

static int a1_write16(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= ~1u;
	if (s_mode == MODE_ARM) {
		if (a >= 0x4f0000 && a <= 0x4f003f) { arm7_type1_ram_w((a - 0x4f0000) >> 1, v, mask); return 1; }
		if (s_kovshp_w && a >= 0x500000 && a <= 0x500005) { kovshp_asic27a_write_word((a - 0x500000) >> 1, v); return 1; }
		if (a >= 0x500000 && a <= 0x500003) { arm7_type1_68k_protlatch_w((a - 0x500000) >> 1, v); return 1; }
		return 0;
	}
	if (a >= s_sim_base && a <= s_sim_base + 5) { arm7_type1_sim_w((a - s_sim_base) >> 1, v); return 1; }
	return 0;
}

// ---- the idle loop (ReBOX speed: the result is the same as running it) ----
// The 55857E program waits for the 68000 in
//     head: bl poll ; bl tick ; b head
// poll reads the latch and returns at once while no command is flagged; tick reads the counter at 0x4000000c (it
// goes up by one a read) and keeps the value in RAM:
//     tick: stmdb sp!,{lr} ; mov r1,#0xff ; mov r0,#0x40000000 ; str r1,[r0,#8] ; ldr r1,[r0,#0xc]! ;
//           ldr r0,=base ; ldr r2,[r0,#X] ; cmp r1,r2 ; strne r1,[r0,#X] ; movne r1,#0 ;
//           strne r1,[r0,#X+4]! ; ldmne sp!,{pc}         (X: 0x34 kovsh, 0x10 photoy2k)
// An iteration is run an instruction at a time and checked to be that short way (four changing writes: the 0x40000008
// strobe, the RAM word, and poll's and tick's return addresses into the same stack word; the counter up by one; no register but r2 changed); the rest of the slice's iterations are
// then applied at once: the counter up by their number, the RAM word, r2 and the flags as the last one leaves them.
static struct { uint32_t head, tick, ofs; int bad; } s_idle;

static uint32_t irom32(uint32_t a) { uint32_t w = 0; if (a + 3 < s_irom_size) memcpy(&w, s_irom + a, 4); return w; }

static int idle_head(uint32_t pc)
{
	if (s_idle.head)
		return pc == s_idle.head;
	if (s_idle.bad || pc + 12 > s_irom_size)
		return 0;
	const uint32_t i0 = irom32(pc), i1 = irom32(pc + 4), i2 = irom32(pc + 8);
	if ((i0 & 0xff000000) != 0xeb000000 || (i1 & 0xff000000) != 0xeb000000 || i2 != 0xeafffffc)
		return 0;
	const uint32_t t = pc + 4 + 8 + ((uint32_t)((int32_t)(i1 << 8) >> 6));
	// (the word's offset X and the literal's place differ between programs: ldr r2,[r0,#X] ; strne r1,[r0,#X] ;
	// strne r1,[r0,#X+4]!)
	const uint32_t x = irom32(t + 24) & 0xfff;
	const uint32_t sig[12] = { 0xe92d4000, 0xe3a010ff, 0xe3a00101, 0xe5801008, 0xe5b0100c, 0xe51f0000, 0xe5902000 | x,
		                       0xe1510002, 0x15801000 | x, 0x13a01000, 0x15a01000 | (x + 4), 0x18bd8000 };
	for (int k = 0; k < 12; k++)
		if ((irom32(t + 4 * k) & (k == 5 ? 0xff7ff000 : 0xffffffff)) != sig[k])
			return 0;
	if (irom32(t + 24) != (0xe5902000 | x) || x > 0x3f0)
		return 0;
	s_idle.head = pc;
	s_idle.tick = t;
	s_idle.ofs = x;
	return 1;
}

static int arm1_idle(int budget)
{
	if ((s_arm->cpsr & 0x20) || s_arm->pending_interrupt || s_idle.bad)
		return budget;
	uint32_t pc = arm7_pc(s_arm);
	// inside the loop (a slice ended there): an instruction at a time up to its head
	for (int k = 0; s_idle.head && pc != s_idle.head && k < 64 && budget > 0 && !s_arm->pending_interrupt; k++) {
		budget -= arm7_run(s_arm, 1);
		pc = arm7_pc(s_arm);
	}
	if (budget <= 0 || s_arm->pending_interrupt || !idle_head(pc))
		return budget;
	// one iteration, checked
	uint32_t r[16];
	memcpy(r, s_arm->r, sizeof(r));
	const uint32_t w0 = s_arm->writes, c0 = S.m_arm_type1_counter;
	int cyc = 0, k = 0;
	do {
		cyc += arm7_run(s_arm, 1);
		k++;
	} while (arm7_pc(s_arm) != s_idle.head && k < 200 && !s_arm->pending_interrupt && !(s_arm->cpsr & 0x20));
	budget -= cyc;
	if (arm7_pc(s_arm) != s_idle.head || s_arm->pending_interrupt)
		return budget;
	int same = s_arm->writes - w0 == 4 && S.m_arm_type1_counter == c0 + 1 && s_arm->r[1] == 0;
	for (int i = 0; i < 16 && same; i++)
		same = i == 2 || s_arm->r[i] == r[i];
	const uint32_t base = s_arm->r[0] - (s_idle.ofs + 4);
	uint32_t *word = NULL;
	if (same && base >= 0x10000000 && base + s_idle.ofs + 4 <= 0x100003ff)
		word = (uint32_t *)(void *)(S.iram + (base - 0x10000000) + s_idle.ofs);
	if (!word || *word != c0)
		return budget;
	const uint32_t n = budget > 0 ? (uint32_t)budget / (uint32_t)cyc : 0;
	if (!n)
		return budget;
	S.m_arm_type1_counter = c0 + 1 + n;
	*word = c0 + n;              // (the value the last iteration read)
	s_arm->r[2] = c0 + n - 1;    // (the one before it, which it compared with)
	s_arm->cpsr = (s_arm->cpsr & 0x0fffffff) | 0x20000000;   // (cmp of two values one apart: C set)
	s_arm->total_cycles += (int64_t)n * cyc;
	return budget - (int)(n * (uint32_t)cyc);
}

static void a1_line(int line)
{
	(void)line;
	if (!s_arm)
		return;
	int budget = pgm_line_cycles(&S.frac, ARM1_CLOCK);
	budget = arm1_idle(budget);
	if (budget > 0)
		arm7_run(s_arm, budget);
}

// ---- the bootlegs' ROM decoders (kovlsqh2 / kovqhsgs) ----
// a 24-bit bitswap as two lookups (the bit permutation is linear: swap(lo | hi) = swap(lo) | swap(hi))
typedef struct { uint32_t lo[4096], hi[4096]; } perm24;
static void perm_make(perm24 *p, const int8_t *b, int n)
{
	for (uint32_t v = 0; v < 4096; v++) {
		p->lo[v] = bsw(v, n, b);
		p->hi[v] = bsw(v << 12, n, b);
	}
}
static inline uint32_t perm(const perm24 *p, uint32_t v) { return p->lo[v & 0xfff] | p->hi[(v >> 12) & 0xfff]; }

static int pgm_decode_kovlsqh2_tiles(perm24 *pa, perm24 *pd, uint8_t *tmp)
{
	uint32_t size;
	uint8_t *rgn = pgm_region("igs023", &size);
	if (!rgn || size < 0x180000 + 0x800000)
		return -1;
	u16 *src = (u16 *)(rgn + 0x180000);
	u16 *dst = (u16 *)tmp;
	static const int8_t ab[24] = { 23, 22, 9, 8, 21, 18, 0, 1, 2, 3, 16, 15, 14, 13, 12, 11, 10, 19, 20, 17, 7, 6, 5, 4 };
	static const int8_t db[16] = { 1, 14, 8, 7, 0, 15, 6, 9, 13, 2, 5, 10, 12, 3, 4, 11 };
	perm_make(pa, ab, 24);
	perm_make(pd, db, 16);
	memset(dst, 0, 0x800000);
	for (int i = 0; i < 0x800000 / 2; i++)
		dst[perm(pa, (uint32_t)i)] = (u16)perm(pd, src[i]);
	memcpy(src, dst, 0x800000);
	return 0;
}

static void pgm_decode_kovlsqh2_sprites(const perm24 *pa, u8 *src, uint8_t *dst)
{
	for (int i = 0; i < 0x800000; i++)
		dst[perm(pa, (uint32_t)i)] = src[i];
	memcpy(src, dst, 0x800000);
}

static int pgm_decode_kovlsqh2_samples(void)
{
	uint32_t size;
	uint8_t *rgn = pgm_region("ics", &size);
	if (!rgn || size < 0xc00000)
		return -1;
	u8 *src = rgn + 0x400000;
	for (int i = 0; i < 0x400000; i += 2)
		src[i + 0x000001] = src[i + 0x400001];
	memcpy(src + 0x400000, src, 0x400000);
	return 0;
}

static int pgm_decode_kovqhsgs_program(perm24 *pa, perm24 *pd, uint8_t *tmp)
{
	if (pgm_rom_size < 0x100000 + 0x400000)
		return -1;
	u16 *src = (u16 *)(pgm_rom + 0x100000);
	u16 *dst = (u16 *)tmp;
	static const int8_t ab[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 6, 7, 5, 4, 3, 2, 1, 0 };
	static const int8_t db[16] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 4, 5, 3, 2, 1, 0 };
	perm_make(pa, ab, 24);
	perm_make(pd, db, 16);
	memset(dst, 0, 0x400000);
	for (int i = 0; i < 0x400000 / 2; i++)
		dst[perm(pa, (uint32_t)i)] = (u16)perm(pd, src[i]);
	memcpy(src, dst, 0x400000);
	return 0;
}

static int pgm_decode_kovqhsgs2_program(perm24 *pa, uint8_t *tmp)
{
	if (pgm_rom_size < 0x100000 + 0x400000)
		return -1;
	u16 *src = (u16 *)(pgm_rom + 0x100000);
	u16 *dst = (u16 *)tmp;
	static const int8_t ab[24] = { 23, 22, 21, 20, 19, 16, 15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3, 4, 5, 6, 18, 17, 7 };
	perm_make(pa, ab, 24);
	memset(dst, 0, 0x400000);
	for (int i = 0; i < 0x400000 / 2; i++)
		dst[perm(pa, (uint32_t)i)] = src[i];
	memcpy(src, dst, 0x400000);
	return 0;
}

// init_kovlsqh2 / init_kovqhsgs: the program, tiles, sprites and samples
static int decode_kovlsqh2(int qhsgs)
{
	perm24 *pa = (perm24 *)malloc(sizeof(perm24)), *pd = (perm24 *)malloc(sizeof(perm24));
	uint8_t *tmp = (uint8_t *)malloc(0x800000);
	int rc = -1;
	if (pa && pd && tmp) {
		rc = qhsgs ? pgm_decode_kovqhsgs_program(pa, pd, tmp) : pgm_decode_kovqhsgs2_program(pa, tmp);
		if (!rc)
			rc = pgm_decode_kovlsqh2_tiles(pa, pd, tmp);
		if (!rc) {
			uint32_t csize, msize;
			uint8_t *col = pgm_region("igs023:sprcol", &csize);
			uint8_t *msk = pgm_region("igs023:sprmask", &msize);
			if (!col || !msk || csize < 0x3000000 || msize < 0x1000000)
				rc = -1;
			else {
				static const int8_t sb[24] = { 23, 10, 9, 22, 19, 18, 20, 21, 17, 16, 15, 14, 13, 12, 11, 8, 7, 6, 5, 4, 3, 2, 1, 0 };
				perm_make(pa, sb, 24);
				for (uint32_t o = 0; o < 0x3000000; o += 0x800000)
					pgm_decode_kovlsqh2_sprites(pa, col + o, tmp);
				pgm_decode_kovlsqh2_sprites(pa, msk + 0x0000000, tmp);
				pgm_decode_kovlsqh2_sprites(pa, msk + 0x0800000, tmp);
			}
		}
		if (!rc)
			rc = pgm_decode_kovlsqh2_samples();
	}
	free(pa);
	free(pd);
	free(tmp);
	return rc;
}

static void arm7_type1_latch_init(void)
{
	S.m_arm_type1_latch_arm_w = 0;
	S.m_arm_type1_latch_68k_w = 0;
	S.m_arm_type1_counter = 1;
}

static void a1_exit(void)
{
	free(s_arm);
	s_arm = NULL;
	arm_sim_handler = NULL;
}

static int start_arm(void)
{
	s_irom = pgm_region("prot", &s_irom_size);
	if (!s_irom || s_irom_size < 0x4000) {
		rbx_fail("IGS027A: internal ROM (prot) missing");
		return -1;
	}
	s_arm = (arm7_t *)calloc(1, sizeof(arm7_t));
	if (!s_arm) {
		rbx_fail("IGS027A: out of memory");
		return -1;
	}
	arm7_init(s_arm);
	s_arm->idle_skip = 1;   // (ReBOX: waiting loops end the slice, arm7.h)
	s_arm->jit = 1;         // (ReBOX: its ROM code translated on ARM hosts, arm7_jit.inc)
	arm7_jit_reset();
	s_arm->read32 = arm_r32;
	s_arm->read16 = arm_r16;
	s_arm->read8 = arm_r8;
	s_arm->write32 = arm_w32;
	s_arm->write16 = arm_w16;
	s_arm->write8 = arm_w8;
	arm7_map(s_arm, 0x00000000, 0x00003fff, s_irom, 0);
	arm7_map(s_arm, 0x10000000, 0x100003ff, S.iram, 1);   // internal ram for asic
	arm7_map(s_arm, 0x50000000, 0x500003ff, S.xor_ram, 1);   // uploads xor table to decrypt 68k rom here
	return 0;
}

static int a1_init(const char *init)
{
	memset(&S, 0, sizeof(S));
	memset(&s_idle, 0, sizeof(s_idle));
	s_arm = NULL;
	arm_sim_handler = NULL;
	const char *machine = pgm_game->machine;
	s_cave = !strcmp(machine, "pgm_arm_type1_cave");
	s_mode = !strcmp(machine, "pgm_arm_type1") ? MODE_ARM : MODE_SIM;
	s_fake_region = s_kovshp_w = 0;
	s_sim_base = 0x500000;
	s_protram = PROTRAM_SIM;
	uint8_t *rom = pgm_rom;
	const uint32_t rs = pgm_rom_size;

	if (!strcmp(init, "photoy2k")) {
		pgm_photoy2k_decrypt(rom, rs);
		arm7_type1_latch_init();
		s_fake_region = 1;
	} else if (!strcmp(init, "kovsh")) {
		pgm_kovsh_decrypt(rom, rs);
		arm7_type1_latch_init();
		s_fake_region = 1;
	} else if (!strcmp(init, "kovshp")) {
		pgm_kovshp_decrypt(rom, rs);
		arm7_type1_latch_init();
		s_fake_region = s_kovshp_w = 1;
	} else if (!strcmp(init, "kovshxas")) {
		arm7_type1_latch_init();
		s_fake_region = s_kovshp_w = 1;
	} else if (!strcmp(init, "kovlsqh2") || !strcmp(init, "kovqhsgs")) {
		const int qhsgs = !strcmp(init, "kovqhsgs");
		if (decode_kovlsqh2(qhsgs)) {
			rbx_fail("%s: ROM regions too small or out of memory", init);
			return -1;
		}
		arm7_type1_latch_init();
		s_fake_region = 1;
		s_kovshp_w = !qhsgs;
	} else if (!strcmp(init, "ddp3")) {
		pgm_py2k2_decrypt(rom, rs);   // yes, it's the same as photo y2k2
		arm_sim_handler = command_handler_ddp3;
		s_protram = PROTRAM_NONE;
	} else if (!strcmp(init, "ket") || !strcmp(init, "espgal")) {
		if (init[0] == 'k')
			pgm_ket_decrypt(rom, rs);
		else
			pgm_espgal_decrypt(rom, rs);
		arm_sim_handler = command_handler_ddp3;
		s_sim_base = 0x400000;
		s_protram = PROTRAM_NONE;
	} else if (!strcmp(init, "puzzli2")) {
		pgm_puzzli2_decrypt(rom, rs);
		arm_sim_handler = command_handler_puzzli2;
		pgm_irq4_disabled = 1;   // doesn't like this irq?? - seems to be RTC related
	} else if (!strcmp(init, "py2k2")) {
		pgm_py2k2_decrypt(rom, rs);
		arm_sim_handler = command_handler_py2k2;
	} else if (!strcmp(init, "pgm3in1")) {
		pgm_decrypt_pgm3in1(rom, rs);
		arm_sim_handler = command_handler_py2k2;
		pgm_irq4_disabled = 1;   // doesn't like this irq??
	} else if (!strcmp(init, "pstar")) {
		pgm_pstar_decrypt(rom, rs);
		arm7_type1_latch_init();
		arm_sim_handler = command_handler_pstars;
		s_protram = PROTRAM_PSTARS;
	} else if (!strcmp(init, "kov") || !strcmp(init, "kovboot")) {
		if (!strcmp(init, "kov"))
			pgm_kov_decrypt(rom, rs);
		arm7_type1_latch_init();
		arm_sim_handler = command_handler_kov;
	} else if (!strcmp(init, "oldsplus")) {
		pgm_oldsplus_decrypt(rom, rs);
		arm7_type1_latch_init();
		arm_sim_handler = command_handler_oldsplus;
	} else {
		rbx_fail("IGS027A type 1: unknown init %s", init);
		return -1;
	}
	if (s_mode == MODE_SIM && !arm_sim_handler) {
		rbx_fail("IGS027A type 1: %s needs the ARM", init);
		return -1;
	}
	if (s_mode == MODE_ARM && start_arm()) {
		a1_exit();
		return -1;
	}
	// kov_map / kov_sim_map: the game ROM bank goes on to 0x4effff (cavepgm_mem: ROM up to 0x3fffff only)
	if (!s_cave && rs > 0x400000) {
		const uint32_t end = rs - 1 < 0x4effff ? rs - 1 : 0x4effff;
		pgm_map(0x400000, end, rom + 0x400000, rs - 0x400000, 0);
	}
	return 0;
}

static void a1_reset(void)
{
	// (MAME: no machine_reset of its own; the ARM is reset, the latches and the simulations keep their state)
	if (s_arm)
		arm7_reset(s_arm);
		arm7_jit_reset();   // (the reset may patch the internal ROM)
}

static int a1_state(void *ctx, int load)
{
	int rc = 0;
	if (s_arm) {
		uint8_t regs[ARM7_STATE_SIZE];
		if (!load)
			memcpy(regs, s_arm, ARM7_STATE_SIZE);
		rc |= rbx_state_io(ctx, "arm7", regs, ARM7_STATE_SIZE);
		if (!rc && load)
			memcpy(s_arm, regs, ARM7_STATE_SIZE);
	}
	rc |= rbx_state_io(ctx, "arm1", &S, sizeof(S));
	return rc ? -1 : 0;
}

const pgm_prot pgm_prot_arm1 = { a1_init, a1_reset, a1_exit, a1_read16, a1_write16, a1_line, NULL, a1_state, 0x3fffff };
