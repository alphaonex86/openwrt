// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pcsample: where the CPUs spend their time, kernel modules included.
 * Samples the cpu-clock software event on every online CPU for N seconds
 * and prints the share of each kallsyms symbol, busiest first.
 *
 * The sampling tick cannot land while interrupts are masked: it fires on the
 * instruction that unmasks them. A sample in that irq/lock plumbing is the
 * masked time of its caller, so it is charged to the first caller outside it.
 *
 * `spin` adds a SCHED_IDLE counting loop: it only gets the CPU nothing else
 * wants, so its rate under load over its idle rate is the free CPU, whatever
 * the kernel's tick accounting says.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <linux/perf_event.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define DATA_PAGES	64
#define MAX_CPUS	16
#define MAX_FRAMES	32
#define NAME_LEN	96
#define NO_SYM		UINT32_MAX

struct ring {
	int fd;
	struct perf_event_mmap_page *meta;
	const unsigned char *data;
	uint64_t size;
};

struct sym {
	uint64_t addr;
	char name[NAME_LEN];
	int plumbing;
};

struct sample {
	uint32_t sym;		/* n_syms: user space */
	uint32_t caller;	/* NO_SYM unless sym is irq/lock plumbing */
};

static struct sym *syms;
static size_t n_syms;
static struct sample *samples;
static size_t n_samples, cap_samples;
static uint64_t lost;
static volatile uint32_t *spins;
static uint32_t spins_seen;
static uint64_t spins_total;

static pid_t spin_start(void)
{
	struct sched_param idle = { 0 };
	pid_t pid;

	spins = mmap(NULL, sizeof(*spins), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (spins == MAP_FAILED) {
		fprintf(stderr, "pcsample: spin counter mmap: %s\n", strerror(errno));
		return -1;
	}
	pid = fork();
	if (pid < 0) {
		fprintf(stderr, "pcsample: fork: %s\n", strerror(errno));
		return -1;
	}
	if (!pid) {
		if (syscall(SYS_sched_setscheduler, 0, SCHED_IDLE, &idle)) {	/* musl stubs the libc call */
			fprintf(stderr, "pcsample: SCHED_IDLE: %s\n", strerror(errno));
			_exit(1);
		}
		for (;;)
			(*spins)++;
	}
	return pid;
}

static void spin_read(void)
{
	uint32_t now = *spins;

	spins_total += (uint32_t)(now - spins_seen);
	spins_seen = now;
}

static const char *const plumbing_prefix[] = {
	"arch_local_irq", "_raw_spin_unlock", "_raw_read_unlock", "_raw_write_unlock",
	"__local_bh_enable", "local_bh_enable", "do_raw_spin_unlock",
};

static int is_plumbing(const char *name)
{
	size_t i;

	for (i = 0; i < sizeof(plumbing_prefix) / sizeof(plumbing_prefix[0]); i++)
		if (!strncmp(name, plumbing_prefix[i], strlen(plumbing_prefix[i])))
			return 1;
	return 0;
}

static int by_addr(const void *a, const void *b)
{
	const struct sym *x = a, *y = b;

	return x->addr < y->addr ? -1 : x->addr > y->addr;
}

/* Text symbols of the kernel and of every loaded module, sorted by address. */
static int load_kallsyms(void)
{
	FILE *f = fopen("/proc/kallsyms", "r");
	size_t cap = 0;
	char line[256];

	if (!f) {
		fprintf(stderr, "pcsample: /proc/kallsyms: %s\n", strerror(errno));
		return -1;
	}
	while (fgets(line, sizeof(line), f)) {
		unsigned long long addr;
		char type, name[64], mod[31];
		int fields = sscanf(line, "%llx %c %63s %30s", &addr, &type, name, mod);

		if (fields < 3 || !strchr("tTwW", type) || !addr)
			continue;
		if (n_syms == cap) {
			struct sym *grown;

			cap = cap ? 2 * cap : 8192;
			grown = realloc(syms, cap * sizeof(*syms));
			if (!grown) {
				fprintf(stderr, "pcsample: out of memory for kallsyms\n");
				fclose(f);
				return -1;
			}
			syms = grown;
		}
		syms[n_syms].addr = addr;
		syms[n_syms].plumbing = is_plumbing(name);
		snprintf(syms[n_syms].name, NAME_LEN, "%s%s%s", name, fields == 4 ? " " : "",
			 fields == 4 ? mod : "");
		n_syms++;
	}
	fclose(f);
	if (!n_syms) {
		fprintf(stderr, "pcsample: /proc/kallsyms lists no address (kptr_restrict?)\n");
		return -1;
	}
	qsort(syms, n_syms, sizeof(*syms), by_addr);
	return 0;
}

static uint32_t sym_of(uint64_t ip)
{
	size_t lo = 0, hi = n_syms;

	if (ip < syms[0].addr)
		return n_syms;
	while (hi - lo > 1) {
		size_t mid = lo + (hi - lo) / 2;

		if (syms[mid].addr <= ip)
			lo = mid;
		else
			hi = mid;
	}
	return lo;
}

static const char *name_of(uint32_t sym)
{
	return sym == n_syms ? "[user]" : syms[sym].name;
}

static int push_sample(uint32_t sym, uint32_t caller)
{
	if (n_samples == cap_samples) {
		size_t cap = cap_samples ? 2 * cap_samples : 4096;
		struct sample *grown = realloc(samples, cap * sizeof(*samples));

		if (!grown) {
			fprintf(stderr, "pcsample: out of memory for samples\n");
			return -1;
		}
		samples = grown;
		cap_samples = cap;
	}
	samples[n_samples++] = (struct sample){ sym, caller };
	return 0;
}

static void ring_copy(const struct ring *r, uint64_t pos, void *dst, size_t len)
{
	unsigned char *out = dst;
	size_t i;

	for (i = 0; i < len; i++)
		out[i] = r->data[(pos + i) & (r->size - 1)];
}

/* A sample record: ip, then the kernel callchain (context markers included). */
static int take_sample(const struct ring *r, uint64_t pos, uint16_t size)
{
	uint64_t ip, nr, frame;
	uint32_t sym, caller = NO_SYM;
	uint64_t i;

	if (size < sizeof(struct perf_event_header) + 16)
		return 0;
	pos += sizeof(struct perf_event_header);
	ring_copy(r, pos, &ip, 8);
	ring_copy(r, pos + 8, &nr, 8);
	sym = sym_of(ip);
	if (sym != n_syms && syms[sym].plumbing) {
		if (nr > (size - sizeof(struct perf_event_header) - 16) / 8)
			nr = (size - sizeof(struct perf_event_header) - 16) / 8;
		for (i = 0; i < nr && i < MAX_FRAMES; i++) {
			uint32_t s;

			ring_copy(r, pos + 16 + 8 * i, &frame, 8);
			if (frame >= (uint64_t)PERF_CONTEXT_MAX)
				continue;
			s = sym_of(frame);
			if (s != n_syms && !syms[s].plumbing) {
				caller = s;
				break;
			}
		}
	}
	return push_sample(sym, caller);
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
		if (h.type == PERF_RECORD_SAMPLE) {
			if (take_sample(r, tail, h.size))
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
	a.sample_period = 1000000;
	a.sample_type = PERF_SAMPLE_IP | PERF_SAMPLE_CALLCHAIN;
	a.exclude_callchain_user = 1;
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

static int by_pair(const void *a, const void *b)
{
	const struct sample *x = a, *y = b;

	if (x->sym != y->sym)
		return x->sym < y->sym ? -1 : 1;
	return x->caller < y->caller ? -1 : x->caller > y->caller;
}

struct row {
	struct sample key;
	uint64_t count;
};

static int by_count(const void *a, const void *b)
{
	const struct row *x = a, *y = b;

	return x->count < y->count ? 1 : x->count > y->count ? -1 : by_pair(&x->key, &y->key);
}

static int report(double min_pct, int seconds)
{
	struct row *rows = calloc(n_samples, sizeof(*rows));
	size_t n_rows = 0, i;

	if (!rows) {
		fprintf(stderr, "pcsample: out of memory for the report\n");
		return 1;
	}
	qsort(samples, n_samples, sizeof(*samples), by_pair);
	for (i = 0; i < n_samples; i++) {
		if (n_rows && !by_pair(&rows[n_rows - 1].key, &samples[i]))
			rows[n_rows - 1].count++;
		else
			rows[n_rows++] = (struct row){ samples[i], 1 };
	}
	qsort(rows, n_rows, sizeof(*rows), by_count);
	printf("pcsample: %zu samples, %llu lost, %d s; \"a <- b\": b's masked time, unmasked in a\n",
	       n_samples, (unsigned long long)lost, seconds);
	for (i = 0; i < n_rows; i++) {
		double pct = 100.0 * rows[i].count / n_samples;

		if (pct < min_pct)
			break;
		if (rows[i].key.caller == NO_SYM)
			printf("%6.2f%% %7llu  %s\n", pct, (unsigned long long)rows[i].count,
			       name_of(rows[i].key.sym));
		else
			printf("%6.2f%% %7llu  %s <- %s\n", pct, (unsigned long long)rows[i].count,
			       name_of(rows[i].key.sym), name_of(rows[i].key.caller));
	}
	free(rows);
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
	int spin = argc > 3 && !strcmp(argv[3], "spin");
	struct timespec now, start, end;
	pid_t spinner = -1;
	int i, n = 0, status;

	if (seconds <= 0 || page <= 0 || cpus <= 0) {
		fprintf(stderr, "usage: pcsample <seconds> [min_percent] [spin]\n");
		return 2;
	}
	if (load_kallsyms())
		return 1;
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
	if (spin && (spinner = spin_start()) < 0)
		return 1;
	clock_gettime(CLOCK_MONOTONIC, &start);
	end = start;
	end.tv_sec += seconds;
	if (spin)
		spins_seen = *spins;
	do {
		if (poll(pfd, n, 100) < 0 && errno != EINTR) {
			fprintf(stderr, "pcsample: poll: %s\n", strerror(errno));
			return 1;
		}
		for (i = 0; i < n; i++)
			if (drain(&rings[i]))
				return 1;
		if (spin)
			spin_read();
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
	if (spin) {
		double s = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;

		spin_read();
		if (kill(spinner, SIGKILL) || waitpid(spinner, &status, 0) != spinner) {
			fprintf(stderr, "pcsample: stopping the spinner: %s\n", strerror(errno));
			return 1;
		}
		if (!WIFSIGNALED(status)) {
			fprintf(stderr, "pcsample: the spinner exited by itself (status %d)\n", status);
			return 1;
		}
		printf("spin: %.0f iterations/s at SCHED_IDLE over %.1f s\n", spins_total / s, s);
	}
	if (!n_samples) {
		fprintf(stderr, "pcsample: no sample in %d s\n", seconds);
		return 1;
	}
	return report(min_pct, seconds);
}
