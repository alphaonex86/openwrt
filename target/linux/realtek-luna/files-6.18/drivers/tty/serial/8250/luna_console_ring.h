/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Single-producer single-consumer byte ring of the asynchronous console.
 * Pure C with no kernel API, so the x86 ladder tests the shipped code.
 * The producer is printk's console write (serialized by the console lock),
 * the consumer the drain thread. A record that does not fit whole is dropped
 * and counted; the next record that fits is preceded by a marker naming the
 * count. Never blocks, never overwrites unread bytes.
 */
#ifndef LUNA_CONSOLE_RING_H
#define LUNA_CONSOLE_RING_H

#ifdef __KERNEL__
#define LCR_LOAD(p)	smp_load_acquire(p)
#define LCR_STORE(p, v)	smp_store_release(p, v)
#else
#define LCR_LOAD(p)	__atomic_load_n(p, __ATOMIC_ACQUIRE)
#define LCR_STORE(p, v)	__atomic_store_n(p, v, __ATOMIC_RELEASE)
#endif

struct luna_con_ring {
	unsigned char *buf;
	unsigned int mask;	/* size - 1, size a power of two */
	unsigned int head;	/* written by the producer only */
	unsigned int tail;	/* written by the consumer only */
	unsigned int dropped;	/* producer only: bytes lost since the last marker */
	unsigned int lost;	/* producer only: bytes lost since boot */
};

static inline unsigned int luna_con_ring_used(struct luna_con_ring *r)
{
	return LCR_LOAD(&r->head) - LCR_LOAD(&r->tail);
}

static inline unsigned int luna_con_crlf_len(const char *s, unsigned int n)
{
	unsigned int i, len = n;

	for (i = 0; i < n; i++)
		len += s[i] == '\n';
	return len;
}

static inline unsigned int luna_con_marker(char *out, unsigned int dropped)
{
	static const char head[] = "\r\n[console: ";
	static const char tail[] = " bytes dropped]\r\n";
	char digits[10];
	unsigned int n = 0, i;

	for (i = 0; i < sizeof(head) - 1; i++)
		out[n++] = head[i];
	i = 0;
	do {
		digits[i++] = (char)('0' + dropped % 10);
		dropped /= 10;
	} while (dropped);
	while (i)
		out[n++] = digits[--i];
	for (i = 0; i < sizeof(tail) - 1; i++)
		out[n++] = tail[i];
	return n;
}

static inline void luna_con_ring_push(struct luna_con_ring *r, unsigned int *head,
				      const char *s, unsigned int n, int crlf)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (crlf && s[i] == '\n')
			r->buf[(*head)++ & r->mask] = '\r';
		r->buf[(*head)++ & r->mask] = (unsigned char)s[i];
	}
}

/* Queue one record, '\n' sent as "\r\n". -> bytes queued (0 when dropped). */
static inline unsigned int luna_con_ring_put(struct luna_con_ring *r, const char *s,
					     unsigned int n)
{
	unsigned int head = r->head;
	unsigned int room = r->mask + 1 - (head - LCR_LOAD(&r->tail));
	unsigned int need = luna_con_crlf_len(s, n);
	char marker[48];
	unsigned int mlen = r->dropped ? luna_con_marker(marker, r->dropped) : 0;

	if (need + mlen > room) {
		r->dropped += need;
		r->lost += need;
		return 0;
	}
	luna_con_ring_push(r, &head, marker, mlen, 0);
	luna_con_ring_push(r, &head, s, n, 1);
	r->dropped = 0;
	LCR_STORE(&r->head, head);
	return need + mlen;
}

/* Take up to @max bytes. -> bytes copied into @out. */
static inline unsigned int luna_con_ring_get(struct luna_con_ring *r, unsigned char *out,
					     unsigned int max)
{
	unsigned int tail = r->tail;
	unsigned int n = LCR_LOAD(&r->head) - tail;
	unsigned int i;

	if (n > max)
		n = max;
	for (i = 0; i < n; i++)
		out[i] = r->buf[(tail + i) & r->mask];
	LCR_STORE(&r->tail, tail + n);
	return n;
}

/* Take up to @max bytes, stopping after the first '\n'.
 * -> bytes copied; *@eol says whether the last one ends a line. */
static inline unsigned int luna_con_ring_get_line(struct luna_con_ring *r, unsigned char *out,
						  unsigned int max, int *eol)
{
	unsigned int tail = r->tail;
	unsigned int avail = LCR_LOAD(&r->head) - tail;
	unsigned int n = 0;

	*eol = 0;
	while (n < avail && n < max && !*eol) {
		out[n] = r->buf[(tail + n) & r->mask];
		*eol = out[n++] == '\n';
	}
	LCR_STORE(&r->tail, tail + n);
	return n;
}

#endif /* LUNA_CONSOLE_RING_H */
