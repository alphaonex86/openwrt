// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Record layout, little-endian: magic[8] seq[4] boot_seq[4] uptime_s[4] kind[1] 0[1] len[2]
 * crc32[4] 0xff[4], then len text bytes; a record fills whole pages, its tail stays erased.
 */
#ifdef __KERNEL__
#include <linux/string.h>
#else
#include <string.h>
#endif
#include "onu_crashlog_core.h"

static void put32(u8 *p, u32 v)
{
	p[0] = v;
	p[1] = v >> 8;
	p[2] = v >> 16;
	p[3] = v >> 24;
}

static u32 get32(const u8 *p)
{
	return p[0] | (u32)p[1] << 8 | (u32)p[2] << 16 | (u32)p[3] << 24;
}

static u32 get32be(const u8 *p)
{
	return (u32)p[0] << 24 | (u32)p[1] << 16 | (u32)p[2] << 8 | p[3];
}

u32 ocl_crc32(u32 crc, const u8 *p, size_t n)
{
	int k;

	crc = ~crc;
	while (n--) {
		crc ^= *p++;
		for (k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xedb88320u & -(crc & 1));
	}
	return ~crc;
}

size_t ocl_bytes(size_t text_len)
{
	return (OCL_HDR + text_len + OCL_PAGE - 1) / OCL_PAGE * OCL_PAGE;
}

static u32 record_crc(const u8 *hdr, const u8 *text, size_t len)
{
	return ocl_crc32(ocl_crc32(0, hdr, 24), text, len);
}

size_t ocl_encode(u8 *out, size_t cap, const struct ocl_rec *r)
{
	size_t n = ocl_bytes(r->len);

	if (r->len > OCL_MAX_TEXT || n > cap)
		return 0;
	memset(out, 0xff, n);
	memcpy(out, OCL_MAGIC, OCL_MAGIC_LEN);
	put32(out + 8, r->seq);
	put32(out + 12, r->boot_seq);
	put32(out + 16, r->uptime_s);
	out[20] = r->kind;
	out[21] = 0;
	out[22] = r->len;
	out[23] = r->len >> 8;
	memcpy(out + OCL_HDR, r->text, r->len);
	put32(out + 24, record_crc(out, r->text, r->len));
	return n;
}

int ocl_parse(const u8 *p, size_t avail, struct ocl_rec *r)
{
	size_t i;
	u16 len;

	if (avail < OCL_PAGE)
		return OCL_FOREIGN;
	for (i = 0; i < OCL_PAGE && p[i] == 0xff; i++)
		;
	if (i == OCL_PAGE)
		return OCL_ERASED;
	if (memcmp(p, OCL_MAGIC, OCL_MAGIC_LEN))
		return OCL_FOREIGN;
	len = p[22] | (u16)p[23] << 8;
	if (len > OCL_MAX_TEXT || ocl_bytes(len) > avail ||
	    get32(p + 24) != record_crc(p, p + OCL_HDR, len))
		return OCL_FOREIGN;
	r->seq = get32(p + 8);
	r->boot_seq = get32(p + 12);
	r->uptime_s = get32(p + 16);
	r->kind = p[20];
	r->len = len;
	r->text = p + OCL_HDR;
	r->bytes = ocl_bytes(len);
	return OCL_RECORD;
}

/* A boot ended unclean when its last record says neither an orderly reboot nor a crash. */
int ocl_ended_unclean(u8 last_kind)
{
	return last_kind && last_kind != OCL_CLEAN && last_kind != OCL_PANIC && last_kind != OCL_PRETIMEOUT;
}

const char *ocl_kind_name(u8 kind)
{
	static const char *const names[] = { "?", "boot", "clean", "panic", "pretimeout", "unclean", "gpon" };

	return kind < sizeof(names) / sizeof(names[0]) ? names[kind] : "?";
}

/* The offsets, in log order, of the records a compaction writes back: the newest OCL_KEEP_CRASH
 * crash records and, apart, the newest OCL_KEEP_GPON GPON notes, so a re-ranging OLT can never push
 * a crash out; of the rest only the newest boot record, the one before it and the last one, so a
 * compaction never erases how the previous boot ended nor that this one began. */
size_t ocl_keep(const u8 *log, size_t used, size_t *offs, size_t cap)
{
	size_t off, n = 0, crash = 0, gpon = 0, boot = used, before_boot = used, last = used;
	struct ocl_rec r;

	for (off = 0; off < used && ocl_parse(log + off, used - off, &r) == OCL_RECORD; off += r.bytes) {
		if (r.kind == OCL_BOOT) {
			boot = off;
			before_boot = last;
		} else if (r.kind != OCL_CLEAN) {
			*(r.kind == OCL_GPON ? &gpon : &crash) += 1;
		}
		last = off;
	}
	for (off = 0; off < used && ocl_parse(log + off, used - off, &r) == OCL_RECORD; off += r.bytes) {
		int marker = r.kind == OCL_BOOT || r.kind == OCL_CLEAN;
		int counted = !marker && (r.kind == OCL_GPON ? gpon-- <= OCL_KEEP_GPON : crash-- <= OCL_KEEP_CRASH);

		if (!counted && off != boot && off != before_boot && off != last)
			continue;
		if (n < cap)
			offs[n++] = off;
	}
	return n;
}

int ocl_behind_our_kernel(const u8 *uimage_hdr, const char *release, u64 kernel_ofs, u64 region_ofs)
{
	char name[33];
	const char *ver;

	memcpy(name, uimage_hdr + 32, 32);
	name[32] = 0;
	ver = strstr(name, "OpenWrt Linux-");
	return get32be(uimage_hdr) == 0x27051956u && ver && !strcmp(ver + 14, release) &&
	       kernel_ofs + 64 + get32be(uimage_hdr + 12) <= region_ofs;
}
