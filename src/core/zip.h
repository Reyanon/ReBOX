/*  Minimal zip reader (stored and deflate entries, central directory) for ROM sets.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ZIP_H
#define RBX_ZIP_H

#include <stdint.h>

typedef struct {
	char name[128];
	uint32_t crc, size, csize, offset;
	uint16_t method;
} rbx_zip_entry;

typedef struct rbx_zip rbx_zip;

rbx_zip *rbx_zip_open(const char *path);
void rbx_zip_close(rbx_zip *z);
int rbx_zip_count(const rbx_zip *z);
const rbx_zip_entry *rbx_zip_get(const rbx_zip *z, int i);
// find by CRC first (size must match), then by name (case-insensitive, path ignored). -1 if missing
int rbx_zip_find(const rbx_zip *z, const char *name, uint32_t crc, uint32_t size);
// read a whole entry into out (entry size bytes). 0 = ok
int rbx_zip_read(rbx_zip *z, int i, uint8_t *out);
// helpers for the frontends (update packages)
static inline const char *rbx_zip_entry_path(const rbx_zip *z, int i) { const rbx_zip_entry *e = rbx_zip_get(z, i); return e ? e->name : ""; }
static inline uint32_t rbx_zip_entry_size(const rbx_zip *z, int i) { const rbx_zip_entry *e = rbx_zip_get(z, i); return e ? e->size : 0; }

#endif
