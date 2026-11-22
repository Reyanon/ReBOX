/*  Minimal zip reader (stored and deflate entries, central directory) for ROM sets.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "zip.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

struct rbx_zip {
	FILE *fp;
	int count;
	rbx_zip_entry *e;
};

static uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }

rbx_zip *rbx_zip_open(const char *path)
{
	FILE *fp = fopen(path, "rb");
	if (!fp)
		return NULL;
	fseek(fp, 0, SEEK_END);
	long len = ftell(fp);
	// end of central directory: last 22 bytes + comment up to 64KB
	long tail = len < 0x10000 + 22 ? len : 0x10000 + 22;
	uint8_t *buf = (uint8_t *)malloc(tail);
	rbx_zip *z = NULL;
	if (!buf || fseek(fp, len - tail, SEEK_SET) || fread(buf, 1, tail, fp) != (size_t)tail)
		goto fail;
	long eocd = -1;
	for (long i = tail - 22; i >= 0; i--)
		if (le32(buf + i) == 0x06054b50) { eocd = i; break; }
	if (eocd < 0)
		goto fail;
	int count = le16(buf + eocd + 10);
	uint32_t cdsize = le32(buf + eocd + 12), cdofs = le32(buf + eocd + 16);
	free(buf);
	buf = (uint8_t *)malloc(cdsize ? cdsize : 1);
	if (!buf || fseek(fp, cdofs, SEEK_SET) || fread(buf, 1, cdsize, fp) != cdsize)
		goto fail;
	z = (rbx_zip *)calloc(1, sizeof(*z));
	z->e = (rbx_zip_entry *)calloc(count ? count : 1, sizeof(rbx_zip_entry));
	z->fp = fp;
	uint32_t p = 0;
	for (int i = 0; i < count && p + 46 <= cdsize; i++) {
		const uint8_t *h = buf + p;
		if (le32(h) != 0x02014b50)
			break;
		rbx_zip_entry *e = &z->e[z->count];
		e->method = le16(h + 10);
		e->crc = le32(h + 16);
		e->csize = le32(h + 20);
		e->size = le32(h + 24);
		int nlen = le16(h + 28), xlen = le16(h + 30), clen = le16(h + 32);
		e->offset = le32(h + 42);
		int n = nlen < (int)sizeof(e->name) - 1 ? nlen : (int)sizeof(e->name) - 1;
		memcpy(e->name, h + 46, n);
		e->name[n] = 0;
		p += 46 + nlen + xlen + clen;
		if (n && e->name[n - 1] != '/')
			z->count++;
	}
	free(buf);
	return z;
fail:
	free(buf);
	fclose(fp);
	return NULL;
}

void rbx_zip_close(rbx_zip *z)
{
	if (!z)
		return;
	fclose(z->fp);
	free(z->e);
	free(z);
}

int rbx_zip_count(const rbx_zip *z) { return z ? z->count : 0; }
const rbx_zip_entry *rbx_zip_get(const rbx_zip *z, int i) { return z && i >= 0 && i < z->count ? &z->e[i] : NULL; }

static const char *base_name(const char *s)
{
	const char *b = s;
	for (; *s; s++)
		if (*s == '/' || *s == '\\')
			b = s + 1;
	return b;
}

static int name_eq(const char *a, const char *b)
{
	for (; *a && *b; a++, b++)
		if (tolower((unsigned char)*a) != tolower((unsigned char)*b))
			return 0;
	return *a == *b;
}

int rbx_zip_find(const rbx_zip *z, const char *name, uint32_t crc, uint32_t size)
{
	if (!z)
		return -1;
	if (crc)
		for (int i = 0; i < z->count; i++)
			if (z->e[i].crc == crc && z->e[i].size == size)
				return i;
	for (int i = 0; i < z->count; i++)
		if (name && name_eq(base_name(z->e[i].name), name))
			return i;
	return -1;
}

int rbx_zip_read(rbx_zip *z, int i, uint8_t *out)
{
	if (!z || i < 0 || i >= z->count)
		return -1;
	const rbx_zip_entry *e = &z->e[i];
	uint8_t lh[30];
	if (fseek(z->fp, e->offset, SEEK_SET) || fread(lh, 1, 30, z->fp) != 30 || le32(lh) != 0x04034b50)
		return -1;
	if (fseek(z->fp, e->offset + 30 + le16(lh + 26) + le16(lh + 28), SEEK_SET))
		return -1;
	if (e->method == 0)
		return fread(out, 1, e->size, z->fp) == e->size ? 0 : -1;
	if (e->method != 8)
		return -1;
	uint8_t *in = (uint8_t *)malloc(e->csize ? e->csize : 1);
	if (!in)
		return -1;
	int rc = -1;
	if (fread(in, 1, e->csize, z->fp) == e->csize) {
		z_stream s;
		memset(&s, 0, sizeof(s));
		if (inflateInit2(&s, -MAX_WBITS) == Z_OK) {
			s.next_in = in;
			s.avail_in = e->csize;
			s.next_out = out;
			s.avail_out = e->size;
			int r = inflate(&s, Z_FINISH);
			if ((r == Z_STREAM_END || r == Z_OK) && s.total_out == e->size)
				rc = 0;
			inflateEnd(&s);
		}
	}
	free(in);
	return rc;
}

int rbx_zip_read_cb(rbx_zip *z, int i, int (*cb)(void *user, const uint8_t *data, uint32_t len), void *user)
{
	if (!z || i < 0 || i >= z->count)
		return -1;
	const rbx_zip_entry *e = &z->e[i];
	uint8_t lh[30];
	if (fseek(z->fp, e->offset, SEEK_SET) || fread(lh, 1, 30, z->fp) != 30 || le32(lh) != 0x04034b50)
		return -1;
	if (fseek(z->fp, e->offset + 30 + le16(lh + 26) + le16(lh + 28), SEEK_SET))
		return -1;
	if (e->method != 0 && e->method != 8)
		return -1;
	enum { CHUNK = 0x40000 };
	uint8_t *in = (uint8_t *)malloc(CHUNK), *out = (uint8_t *)malloc(CHUNK);
	z_stream s;
	memset(&s, 0, sizeof(s));
	int rc = -1;
	if (!in || !out || (e->method == 8 && inflateInit2(&s, -MAX_WBITS) != Z_OK))
		goto done;
	uint32_t left_in = e->method == 8 ? e->csize : e->size, total = 0;
	int r = Z_OK;
	while (total < e->size) {
		if (e->method == 0) {   // (stored)
			const uint32_t n = left_in < CHUNK ? left_in : CHUNK;
			if (!n || fread(out, 1, n, z->fp) != n || cb(user, out, n))
				break;
			left_in -= n;
			total += n;
			continue;
		}
		if (!s.avail_in) {
			const uint32_t n = left_in < CHUNK ? left_in : CHUNK;
			if (n && fread(in, 1, n, z->fp) != n)
				break;
			left_in -= n;
			s.next_in = in;
			s.avail_in = n;
		}
		s.next_out = out;
		s.avail_out = CHUNK;
		r = inflate(&s, Z_NO_FLUSH);
		if (r != Z_OK && r != Z_STREAM_END)
			break;
		const uint32_t n = CHUNK - s.avail_out;
		if (total + n > e->size || (n && cb(user, out, n)))
			break;
		total += n;
		if (r == Z_STREAM_END || (!n && !s.avail_in && !left_in))
			break;
	}
	if (total == e->size)
		rc = 0;
	if (e->method == 8)
		inflateEnd(&s);
done:
	free(in);
	free(out);
	return rc;
}
