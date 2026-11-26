/*  QDSP QS1000, ported to C from MAME 0.289 src/devices/sound/qs1000.cpp (BSD-3-Clause, copyright-holders: Philip
    Bennett) and okiadpcm.cpp (BSD-3-Clause, copyright-holders: Andrew Gardner, Aaron Giles). See qs1000.h.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "qs1000.h"
#include "../rbx.h"

#include <math.h>
#include <string.h>

enum { QS1000_KEYON = 1, QS1000_PLAYING = 2, QS1000_ADPCM = 4 };
#define QS1000_ADDRESS_MASK 0x00ffffffu

// ---- oki_adpcm_state (okiadpcm.cpp) ----
static const int8_t s_index_shift[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };
static int s_diff_lookup[49 * 16];
static int s_tables_computed;

static void compute_tables(void)
{
	static const int8_t nbl2bit[16][4] = {
		{ 1, 0, 0, 0}, { 1, 0, 0, 1}, { 1, 0, 1, 0}, { 1, 0, 1, 1},
		{ 1, 1, 0, 0}, { 1, 1, 0, 1}, { 1, 1, 1, 0}, { 1, 1, 1, 1},
		{-1, 0, 0, 0}, {-1, 0, 0, 1}, {-1, 0, 1, 0}, {-1, 0, 1, 1},
		{-1, 1, 0, 0}, {-1, 1, 0, 1}, {-1, 1, 1, 0}, {-1, 1, 1, 1}
	};
	if (s_tables_computed)
		return;
	s_tables_computed = 1;
	for (int step = 0; step <= 48; step++) {
		const int stepval = (int)floor(16.0 * pow(11.0 / 10.0, (double)step));
		for (int nib = 0; nib < 16; nib++)
			s_diff_lookup[step * 16 + nib] = nbl2bit[nib][0] * (stepval * nbl2bit[nib][1] + stepval / 2 * nbl2bit[nib][2] +
			                                                    stepval / 4 * nbl2bit[nib][3] + stepval / 8);
	}
}

static inline int32_t adpcm_clock(qs1000_channel *ch, uint8_t nibble)
{
	ch->adpcm_sig += s_diff_lookup[ch->adpcm_step * 16 + (nibble & 15)];
	if (ch->adpcm_sig > 2047)
		ch->adpcm_sig = 2047;
	else if (ch->adpcm_sig < -2048)
		ch->adpcm_sig = -2048;
	ch->adpcm_step += s_index_shift[nibble & 7];
	if (ch->adpcm_step > 48)
		ch->adpcm_step = 48;
	else if (ch->adpcm_step < 0)
		ch->adpcm_step = 0;
	return ch->adpcm_sig;
}

static inline uint8_t read_byte(const qs1000_t *q, uint32_t a)
{
	a &= QS1000_ADDRESS_MASK;
	return a < q->rom_size ? q->rom[a] : 0;
}

// ---- the wavetable engine ----
static void start_voice(qs1000_t *q, int chn)
{
	qs1000_channel *ch = &q->st.ch[chn];
	const uint32_t table_addr = ((uint32_t)ch->regs[0x01] << 16) | ((uint32_t)ch->regs[0x02] << 8) | ch->regs[0x03];
	// Fetch the sound information
	const uint16_t freq = (uint16_t)(read_byte(q, table_addr + 0) << 8 | read_byte(q, table_addr + 1));
	const uint32_t base = (uint32_t)(read_byte(q, table_addr + 4) << 8 | read_byte(q, table_addr + 5));
	// See Raccoon World and Wyvern Wings nullptr sound
	if (freq == 0)
		return;
	// Fetch the sample pointers and flags
	const uint8_t byte0 = read_byte(q, base);
	uint32_t start_addr = (uint32_t)byte0 << 16;
	start_addr |= (uint32_t)read_byte(q, base + 1) << 8;
	start_addr |= read_byte(q, base + 2);
	start_addr &= QS1000_ADDRESS_MASK;
	uint32_t loop_start = (uint32_t)(byte0 & 0xf0) << 16;
	loop_start |= (uint32_t)read_byte(q, base + 3) << 12;
	loop_start |= (uint32_t)read_byte(q, base + 4) << 4;
	loop_start |= read_byte(q, base + 5) >> 4;
	loop_start &= QS1000_ADDRESS_MASK;
	uint32_t loop_end = (uint32_t)(byte0 & 0xf0) << 16;
	loop_end |= (uint32_t)(read_byte(q, base + 5) & 0xf) << 16;
	loop_end |= (uint32_t)read_byte(q, base + 6) << 8;
	loop_end |= read_byte(q, base + 7);
	loop_end &= QS1000_ADDRESS_MASK;
	const uint8_t byte8 = read_byte(q, base + 8);

	ch->acc = 0;
	ch->start = start_addr;
	ch->addr = start_addr;
	ch->loop_start = loop_start;
	ch->loop_end = loop_end;
	ch->freq = freq;
	ch->flags = QS1000_PLAYING | QS1000_KEYON;
	if (byte8 & 0x08) {
		ch->adpcm_sig = 0;   // (oki_adpcm_state::reset)
		ch->adpcm_step = 0;
		ch->adpcm_addr = 0xffffffffu;
		ch->flags |= QS1000_ADPCM;
	}
}

static void wave_w(qs1000_t *q, uint32_t offset, uint8_t data)
{
	uint8_t *wr = q->st.wave_regs;
	switch (offset) {
	case 0x00: {
		const int ch = wr[0xe] & (QS1000_CHANNELS - 1);
		if (data == 0) {
			// TODO
			for (int i = 0; i < 16; ++i)
				q->st.ch[ch].regs[i] = wr[i];
			// Key on
			start_voice(q, ch);
		}
		if (data == 1) {
			// ?
		} else if (data == 2) {
			// Key off
			q->st.ch[ch].flags &= (uint16_t)~QS1000_KEYON;
		}
		break;
	}
	case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06: case 0x07:
	case 0x08: case 0x09: case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		if (wr[0x11] == 3)
			q->st.ch[wr[0xe] & (QS1000_CHANNELS - 1)].regs[offset] = data;   // Channel-specific write?
		else
			wr[offset] = data;   // Global write?
		break;
	default:
		wr[offset] = data;
	}
}

void qs1000_render(qs1000_t *q, int32_t *outl, int32_t *outr, int n)
{
	// Iterate over voices and accumulate sample data
	for (int c = 0; c < QS1000_CHANNELS; c++) {
		qs1000_channel *chan = &q->st.ch[c];
		if (!(chan->flags & QS1000_PLAYING))
			continue;
		const int32_t lvol = chan->regs[6], rvol = chan->regs[7], vol = chan->regs[8];
		const uint32_t fstep = (uint32_t)chan->freq * QS1000_STEP;
		if (chan->flags & QS1000_ADPCM) {
			for (int samp = 0; samp < n; samp++) {
				if (chan->addr >= chan->loop_end) {   // (looping disabled until envelopes work)
					chan->flags &= (uint16_t)~QS1000_PLAYING;
					break;
				}
				// Not too keen on this but it'll do for now
				while (chan->start + chan->adpcm_addr != chan->addr) {
					chan->adpcm_addr++;
					if (chan->start + chan->adpcm_addr >= chan->loop_end)
						chan->adpcm_addr = chan->loop_start - chan->start;
					const uint8_t data = read_byte(q, chan->start + (chan->adpcm_addr >> 1));
					const uint8_t nibble = (uint8_t)(((chan->adpcm_addr & 1) ? data : data >> 4) & 0xf);
					chan->adpcm_signal = adpcm_clock(chan, nibble);
				}
				const int32_t result = (int8_t)(chan->adpcm_signal >> 4);
				chan->acc += fstep;
				chan->addr = (chan->addr + (chan->acc >> 18)) & QS1000_ADDRESS_MASK;
				chan->acc &= ((1u << 18) - 1);
				outl[samp] += result * 4 * lvol * vol;
				outr[samp] += result * 4 * rvol * vol;
			}
		} else {
			for (int samp = 0; samp < n; samp++) {
				if (chan->addr >= chan->loop_end) {
					chan->flags &= (uint16_t)~QS1000_PLAYING;
					break;
				}
				const int32_t result = (int8_t)(read_byte(q, chan->addr) - 128);
				chan->acc += fstep;
				chan->addr = (chan->addr + (chan->acc >> 18)) & QS1000_ADDRESS_MASK;
				chan->acc &= ((1u << 18) - 1);
				outl[samp] += result * lvol * vol;
				outr[samp] += result * rvol * vol;
			}
		}
	}
}

// ---- the 8052 ----
static uint8_t cpu_port_in(void *user, int port)
{
	qs1000_t *q = (qs1000_t *)user;
	switch (port) {
	case 1: return q->p1_in ? q->p1_in(q->user) : 0;
	case 3: return q->p3_in ? q->p3_in(q->user) : 0;
	case 2: return 0;   // (p2_in unbound: 0)
	default: return 0xff;
	}
}

static uint8_t cpu_xdata_r(void *user, uint16_t a)
{
	qs1000_t *q = (qs1000_t *)user;
	return a < 0x100 ? q->st.ram[a] : 0;
}

static void cpu_xdata_w(void *user, uint16_t a, uint8_t v)
{
	qs1000_t *q = (qs1000_t *)user;
	if (a < 0x100)
		q->st.ram[a] = v;
	else if (a >= 0x200 && a <= 0x211)
		wave_w(q, a - 0x200u, v);
}

static void cpu_uart_tx(void *user, int data)
{
	(void)user;
	(void)data;   // (TxD not connected)
}

void qs1000_init(qs1000_t *q, uint32_t clock, const uint8_t *cpu_rom, uint32_t cpu_rom_size, const uint8_t *rom,
                 uint32_t rom_size)
{
	compute_tables();
	memset(q, 0, sizeof(*q));
	q->clock = clock;
	q->rom = rom;
	q->rom_size = rom_size;
	mcs51_init(&q->cpu, cpu_rom, cpu_rom_size);
	q->cpu.is_8052 = 1;
	q->cpu.port_in = cpu_port_in;
	q->cpu.xdata_r = cpu_xdata_r;
	q->cpu.xdata_w = cpu_xdata_w;
	q->cpu.uart_tx = cpu_uart_tx;   // (the receiver needs the serial port on)
	q->cpu.user = q;
}

void qs1000_reset(qs1000_t *q)
{
	mcs51_reset(&q->cpu);
	for (int i = 0; i < QS1000_CHANNELS; i++)
		q->st.ch[i].flags = 0;
}

int qs1000_run(qs1000_t *q, int cycles) { return mcs51_execute(&q->cpu, cycles); }
void qs1000_serial_in(qs1000_t *q, int data) { mcs51_uart_rx(&q->cpu, data); }
void qs1000_set_irq(qs1000_t *q, int state) { mcs51_set_input(&q->cpu, MCS51_INT1_LINE, state); }

int qs1000_state(qs1000_t *q, void *ctx, int load)
{
	(void)load;
	if (rbx_state_io(ctx, "qs1000", &q->st, sizeof(q->st)) || rbx_state_io(ctx, "qs1kcpu", &q->cpu.st, sizeof(q->cpu.st)) ||
	    rbx_state_io(ctx, "qs1kcp2", &q->cpu.st2, sizeof(q->cpu.st2)) ||
	    rbx_state_io(ctx, "qs1kuar", &q->cpu.uart, sizeof(q->cpu.uart)))
		return -1;
	return 0;
}
