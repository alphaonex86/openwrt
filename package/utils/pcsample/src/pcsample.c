// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pcsample: where the CPUs spend their time, kernel modules included.
 * Samples the cpu-clock software event on every online CPU for N seconds
 * and prints the share of each kallsyms symbol, busiest first.
 */
#include <errno.h>
#include <linux/perf_event.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#define DATA_PAGES	64
#define MAX_CPUS	16
#define NAME_LEN	96

struct ring {
	int fd;
	struct perf_event_mmap_page *meta;
	const unsigned char *data;
	uint64_t size;
};

struct sym {
	uint64_t addr;
	char name[NAME_LEN];
};

static uint64_t *ips;
static size_t n_ips, cap_ips;
static uint64_t lost;

static int push_ip(uint64_t ip)
{
	if (n_ips == cap_ips) {
		size_t cap = cap_ips ? 2 * cap_ips : 4096;
		uint64_t *grown = realloc(ips, cap * sizeof(*ips));

		if (!grown)
			return -1;
		ips = grown;
		cap_ips = cap;
	}
	ips[n_ips++] = ip;
	return 0;
}

static void ring_copy(const struct ring *r, uint64_t pos, void *dst, size_t len)
{
	unsigned char *out = dst;
	size_t i;

	for (i = 0; i < len; i++)
		out[i] = r->data[(pos + i) & (r->size - 1)];
}

static int drain(struct ring *r)
{
	uint64_t head = *(volatile __u64 *)&r->meta->data_head;
	uint64_t tail = r->meta->data_tail;

	__sync_synchronize();

	while (tail < head) {
		struct perf_event_header h;
		uint64_t body[2];

		ring_copy(r, tail, &h, sizeof(h));
		if (h.size < sizeof(h)) {
			fprintf(stderr, "pcsample: corrupt ring record (size %u)\n", h.size);
			return -1;
		}
		if (h.type == PERF_RECORD_SAMPLE && h.size >= sizeof(h) + 8) {
			ring_copy(r, tail + sizeof(h), body, 8);
			if (push_ip(body[0]))
				return -1;
		} else if (h.type == PERF_RECORD_LOST && h.size >= sizeof(h) + 16) {
			ring_copy(r, tail + sizeof(h), body, 16);
			lost += body[1];
		}
		tail += h.size;
	}
	__sync_synchronize();
	r->meta->data_tail = tail;
	return 0;
}

static int ring_open(struct ring *r, int cpu, long page)
{
	struct perf_event_attr a;
	size_t len = (DATA_PAGES + 1) * page;
	void *map;

	memset(&a, 0, sizeof(a));
	a.size = sizeof(a);
	a.type = PERF_TYPE_SOFTWARE;
	a.config = PERF_COUNT_SW_CPU_CLOCK;
	a.sample_period = 1000000;	/* 1 ms; a low-res hrtimer rounds it up to the tick */
	a.sample_type = PERF_SAMPLE_IP;
	a.disabled = 1;
	a.wakeup_events = 64;
	r->fd = syscall(SYS_perf_event_open, &a, -1, cpu, -1, 0);
	if (r->fd < 0) {
		fprintf(stderr, "pcsample: perf_event_open(cpu %d): %s\n", cpu, strerror(errno));
		return -1;
	}
	map = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, r->fd, 0);
	if (map == MAP_FAILED) {
		fprintf(stderr, "pcsample: mmap(cpu %d): %s\n", cpu, strerror(errno));
		close(r->fd);
		return -1;
	}
	r->meta = map;
	r->data = (const unsigned char *)map + page;
	r->size = (uint64_t)DATA_PAGES * page;
	return 0;
}

static int by_addr(const void *a, const void *b)
{
	const struct sym *x = a, *y = b;

	return x->addr < y->addr ? -1 : x->addr > y->addr;
}

static int by_value(const void *a, const void *b)
{
	uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;

	return x < y ? -1 : x > y;
}

/* Text symbols of the kernel and of every loaded module, sorted by address. */
static struct sym *load_kallsyms(size_t *n)
{
	FILE *f = fopen("/proc/kallsyms", "r");
	struct sym *syms = NULL;
	size_t cap = 0;
	char line[256];

	*n = 0;
	if (!f) {
		fprintf(stderr, "pcsample: /proc/kallsyms: %s\n", strerror(errno));
		return NULL;
	}
	while (fgets(line, sizeof(line), f)) {
		unsigned long long addr;
		char type, name[64], mod[31];
		int fields = sscanf(line, "%llx %c %63s %30s", &addr, &type, name, mod);

		if (fields < 3 || !strchr("tTwW", type) || !addr)
			continue;
		if (*n == cap) {
			struct sym *grown;

			cap = cap ? 2 * cap : 8192;
			grown = realloc(syms, cap * sizeof(*syms));
			if (!grown) {
				fprintf(stderr, "pcsample: out of memory for kallsyms\n");
				free(syms);
				fclose(f);
				return NULL;
			}
			syms = grown;
		}
		syms[*n].addr = addr;
		snprintf(syms[*n].name, NAME_LEN, "%s%s%s", name, fields == 4 ? " " : "",
			 fields == 4 ? mod : "");
		(*n)++;
	}
	fclose(f);
	if (!*n) {
		fprintf(stderr, "pcsample: /proc/kallsyms lists no address (kptr_restrict?)\n");
		free(syms);
		return NULL;
	}
	qsort(syms, *n, sizeof(*syms), by_addr);
	return syms;
}

static const char *sym_of(const struct sym *syms, size_t n, uint64_t ip)
{
	size_t lo = 0, hi = n;

	if (ip < syms[0].addr)
		return "[user]";
	while (hi - lo > 1) {
		size_t mid = lo + (hi - lo) / 2;

		if (syms[mid].addr <= ip)
			lo = mid;
		else
			hi = mid;
	}
	return syms[lo].name;
}

struct row {
	const char *name;
	uint64_t count;
};

static int by_count(const void *a, const void *b)
{
	const struct row *x = a, *y = b;

	return x->count < y->count ? 1 : x->count > y->count ? -1 : strcmp(x->name, y->name);
}

static int report(double min_pct, int seconds)
{
	size_t n_syms, n_rows = 0, i;
	struct sym *syms = load_kallsyms(&n_syms);
	struct row *rows;

	if (!syms)
		return 1;
	rows = calloc(n_ips ? n_ips : 1, sizeof(*rows));
	if (!rows) {
		fprintf(stderr, "pcsample: out of memory for the report\n");
		return 1;
	}
	qsort(ips, n_ips, sizeof(*ips), by_value);
	for (i = 0; i < n_ips; i++) {
		const char *name = sym_of(syms, n_syms, ips[i]);

		if (n_rows && (rows[n_rows - 1].name == name || !strcmp(rows[n_rows - 1].name, name)))
			rows[n_rows - 1].count++;
		else
			rows[n_rows++] = (struct row){ name, 1 };
	}
	qsort(rows, n_rows, sizeof(*rows), by_count);
	printf("pcsample: %zu samples, %llu lost, %d s\n", n_ips, (unsigned long long)lost, seconds);
	for (i = 0; i < n_rows; i++) {
		double pct = 100.0 * rows[i].count / n_ips;

		if (pct < min_pct)
			break;
		printf("%6.2f%% %7llu  %s\n", pct, (unsigned long long)rows[i].count, rows[i].name);
	}
	free(rows);
	free(syms);
	return 0;
}

int main(int argc, char **argv)
{
	struct ring rings[MAX_CPUS];
	struct pollfd pfd[MAX_CPUS];
	long page = sysconf(_SC_PAGESIZE);
	long cpus = sysconf(_SC_NPROCESSORS_ONLN);
	int seconds = argc > 1 ? atoi(argv[1]) : 0;
	double min_pct = argc > 2 ? atof(argv[2]) : 0.5;
	struct timespec now, end;
	int i, n = 0;

	if (seconds <= 0 || page <= 0 || cpus <= 0) {
		fprintf(stderr, "usage: pcsample <seconds> [min_percent]\n");
		return 2;
	}
	for (i = 0; i < cpus && i < MAX_CPUS; i++, n++) {
		if (ring_open(&rings[i], i, page))
			return 1;
		pfd[i] = (struct pollfd){ .fd = rings[i].fd, .events = POLLIN };
	}
	for (i = 0; i < n; i++)
		if (ioctl(rings[i].fd, PERF_EVENT_IOC_ENABLE, 0)) {
			fprintf(stderr, "pcsample: enable(cpu %d): %s\n", i, strerror(errno));
			return 1;
		}
	clock_gettime(CLOCK_MONOTONIC, &end);
	end.tv_sec += seconds;
	do {
		if (poll(pfd, n, 100) < 0 && errno != EINTR) {
			fprintf(stderr, "pcsample: poll: %s\n", strerror(errno));
			return 1;
		}
		for (i = 0; i < n; i++)
			if (drain(&rings[i]))
				return 1;
		clock_gettime(CLOCK_MONOTONIC, &now);
	} while (now.tv_sec < end.tv_sec || (now.tv_sec == end.tv_sec && now.tv_nsec < end.tv_nsec));
	for (i = 0; i < n; i++) {
		if (ioctl(rings[i].fd, PERF_EVENT_IOC_DISABLE, 0)) {
			fprintf(stderr, "pcsample: disable(cpu %d): %s\n", i, strerror(errno));
			return 1;
		}
		if (drain(&rings[i]))
			return 1;
	}
	if (!n_ips) {
		fprintf(stderr, "pcsample: no sample in %d s\n", seconds);
		return 1;
	}
	return report(min_pct, seconds);
}
