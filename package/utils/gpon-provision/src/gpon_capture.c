// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * gpon-capture: the OMCI and PLOAM capture rings from the first record of every boot, before any
 * follower can log in. Each new record is appended to <dir>/<ring>.olt1 (RAM, capped: the start is
 * kept) and, with a collector, sent as one UDP datagram "olt1cap <boot id> <ring> <olt1 line>".
 * Restarted within a boot, it resumes after the last record its files hold.
 *
 *   gpon-capture [-d dir] [-m max_bytes] [-c addr -P port] [-i interval_ms] [-p proc_dir] [-n passes]
 */
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static const char *const ring_names[] = { "oltcap", "ploamcap" };
#define NRINGS (sizeof(ring_names) / sizeof(ring_names[0]))

struct ring {
	long long last;
	long size;
	int capped;
	FILE *out;
};

static int udp_fd = -1;
static struct addrinfo *collector;
static int collector_down;
static char boot_id[64] = "-";

static int open_collector(const char *addr, const char *port)
{
	struct addrinfo hints = { .ai_socktype = SOCK_DGRAM, .ai_flags = AI_NUMERICHOST | AI_NUMERICSERV };
	int rc = getaddrinfo(addr, port, &hints, &collector);

	if (rc) {
		fprintf(stderr, "gpon-capture: collector %s:%s: %s\n", addr, port, gai_strerror(rc));
		return 0;
	}
	udp_fd = socket(collector->ai_family, SOCK_DGRAM, 0);
	if (udp_fd < 0)
		fprintf(stderr, "gpon-capture: socket: %s\n", strerror(errno));
	return udp_fd >= 0;
}

/* Fire and forget, but a collector that stops or starts answering is said once each way. */
static void send_record(const char *ring, const char *line)
{
	char msg[600];
	int n = snprintf(msg, sizeof(msg), "olt1cap %s %s %s", boot_id, ring, line);

	if (n <= 0 || n >= (int)sizeof(msg))
		return;
	if (sendto(udp_fd, msg, n, 0, collector->ai_addr, collector->ai_addrlen) < 0) {
		if (!collector_down)
			fprintf(stderr, "gpon-capture: collector unreachable: %s\n", strerror(errno));
		collector_down = 1;
	} else if (collector_down) {
		fprintf(stderr, "gpon-capture: collector reachable again\n");
		collector_down = 0;
	}
}

static long long record_seq(const char *line)
{
	long long seq;

	return sscanf(line, "olt1 %lld ", &seq) == 1 ? seq : -1;
}

static int ring_open(struct ring *r, const char *dir, const char *name)
{
	char path[512], line[512];

	snprintf(path, sizeof(path), "%s/%s.olt1", dir, name);
	r->out = fopen(path, "a+");
	if (!r->out) {
		fprintf(stderr, "gpon-capture: %s: %s\n", path, strerror(errno));
		return 0;
	}
	r->last = -1;
	r->size = 0;
	r->capped = 0;
	rewind(r->out);
	while (fgets(line, sizeof(line), r->out)) {
		r->size += strlen(line);
		if (record_seq(line) > r->last)
			r->last = record_seq(line);
		if (!strncmp(line, "# capped", 8))
			r->capped = 1;
	}
	if (!r->size)
		r->size = fprintf(r->out, "# boot %s\n", boot_id);
	return 1;
}

static void ring_pass(struct ring *r, const char *proc, const char *name, long max)
{
	char path[512], line[512];
	FILE *f;

	snprintf(path, sizeof(path), "%s/%s", proc, name);
	f = fopen(path, "r");
	if (!f)
		return;		/* the driver not loaded yet, or a family without this ring */
	while (fgets(line, sizeof(line), f)) {
		long long seq = record_seq(line);

		if (seq <= r->last)
			continue;
		r->last = seq;
		if (r->size + (long)strlen(line) <= max) {
			fputs(line, r->out);
			r->size += strlen(line);
		} else if (!r->capped) {
			fprintf(r->out, "# capped at %ld bytes: later records only in the ring and the collector\n", max);
			r->capped = 1;
		}
		if (udp_fd >= 0)
			send_record(name, line);
	}
	fclose(f);
	if (fflush(r->out))
		fprintf(stderr, "gpon-capture: %s.olt1: %s\n", name, strerror(errno));
}

int main(int argc, char **argv)
{
	const char *dir = "/tmp/gpon-capture", *proc = "/proc", *addr = NULL, *port = NULL;
	long max = 1024 * 1024, interval_ms = 500, passes = -1;
	struct ring rings[NRINGS];
	char path[512];
	FILE *f;
	unsigned int i;
	int c;

	while ((c = getopt(argc, argv, "d:m:c:P:i:p:n:")) != -1) {
		switch (c) {
		case 'd': dir = optarg; break;
		case 'm': max = atol(optarg); break;
		case 'c': addr = optarg; break;
		case 'P': port = optarg; break;
		case 'i': interval_ms = atol(optarg); break;
		case 'p': proc = optarg; break;
		case 'n': passes = atol(optarg); break;
		default:
			fprintf(stderr, "usage: gpon-capture [-d dir] [-m max_bytes] [-c addr -P port] "
				"[-i interval_ms] [-p proc_dir] [-n passes]\n");
			return 2;
		}
	}
	if (max <= 0 || interval_ms <= 0 || (!addr != !port) || (addr && !open_collector(addr, port)))
		return 2;
	if (mkdir(dir, 0755) && errno != EEXIST) {
		fprintf(stderr, "gpon-capture: %s: %s\n", dir, strerror(errno));
		return 1;
	}
	snprintf(path, sizeof(path), "%s/sys/kernel/random/boot_id", proc);
	f = fopen(path, "r");
	if (!f || !fgets(boot_id, sizeof(boot_id), f))
		fprintf(stderr, "gpon-capture: no boot id in %s: records go out as boot '-'\n", path);
	if (f)
		fclose(f);
	boot_id[strcspn(boot_id, "\n")] = 0;
	for (i = 0; i < NRINGS; i++)
		if (!ring_open(&rings[i], dir, ring_names[i]))
			return 1;
	for (;;) {
		struct timespec ts = { interval_ms / 1000, (interval_ms % 1000) * 1000000L };

		for (i = 0; i < NRINGS; i++)
			ring_pass(&rings[i], proc, ring_names[i], max);
		if (passes > 0 && !--passes)
			return 0;
		nanosleep(&ts, NULL);
	}
}
