// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE */
/*
 * cpu-attrib: where a busy CPU goes, recorded by the unit itself.
 *
 *   cpu-attrib watch [-p 5] [-b 90] [-n 3] [-w 5] [-H 300]   daemon
 *   cpu-attrib snap [-w 5]                                    one report to stdout
 *   cpu-attrib show                                           the kept reports, oldest first
 *
 * watch: when the CPU stays >= b % busy for n periods of p s, one report over w s goes to a RAM ring
 * (RING_DIR, RING_N files) and its summary to the kernel log (console, crash record); then quiet for H s.
 * At saturation ssh may not even log in: the unit keeps the evidence itself.
 * Sources: /proc/stat, softirqs, interrupts, /proc/<pid>/stat (a kworker's name carries its workqueue),
 * /proc/net/dev, /proc/net/vlan/config, conntrack stats, debugfs workqueue_stats when the kernel has it,
 * frames of a VLAN parent sampled by a packet socket (which VLAN, whether a netdev claims it), and
 * pcsample when the image carries it. Idle cost: one /proc/stat read per period.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <poll.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>

#define RING_DIR	"/tmp/cpu-attrib"
#define RING_N		4
#define KMSG_LINES	8
#define MAX_ROWS	64
#define MAX_TASKS	512
#define MAX_VIDS	64
#define MAX_PARENTS	4
#define SAMPLE_CAP	3000	/* frames a packet socket may copy per report */
#define NAME_LEN	64

struct row { char name[NAME_LEN]; unsigned long long v[4]; };
struct table { struct row r[MAX_ROWS]; int n; };
struct task { int pid; char comm[NAME_LEN]; char state; unsigned long long ticks; };

struct state {
	double t;
	unsigned long long cpu[10];
	unsigned long long running, blocked;
	struct table soft, irq, dev, wq, ct;
	struct task tasks[MAX_TASKS];
	int n_tasks;
	unsigned long long ct_count;
	int have_wq, have_ct;
};

struct vlan { char dev[IFNAMSIZ]; char parent[IFNAMSIZ]; int vid; };
struct vid_count { char parent[IFNAMSIZ]; int vid; int kind; int claimed; unsigned long n; };

static const char *const KIND[] = { "host", "bcast", "mcast", "other" };

/* ---- parsers: pure, host-tested ---- */

/* "cpu  u n s i io irq si st ..." -> fields (up to 10), their count */
int parse_cpu_line(const char *line, unsigned long long *f)
{
	int n = 0;
	const char *p = line;
	char *end;

	if (strncmp(p, "cpu ", 4))
		return 0;
	p += 4;
	while (n < 10) {
		unsigned long long v = strtoull(p, &end, 10);

		if (end == p)
			break;
		f[n++] = v;
		p = end;
	}
	return n;
}

/* a /proc/<pid>/stat line -> pid, comm (may hold spaces and parens), state, utime+stime; 0 on success */
int parse_pid_stat(const char *line, struct task *t)
{
	const char *open = strchr(line, '('), *close = strrchr(line, ')');
	unsigned long long ut, st;
	size_t len;

	if (!open || !close || close < open)
		return -1;
	t->pid = atoi(line);
	len = (size_t)(close - open - 1);
	if (len >= sizeof(t->comm))
		len = sizeof(t->comm) - 1;
	memcpy(t->comm, open + 1, len);
	t->comm[len] = 0;
	/* after ") ": state ppid pgrp session tty tpgid flags minflt cminflt majflt cmajflt utime stime */
	if (sscanf(close + 2, "%c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %llu %llu", &t->state, &ut, &st) != 3)
		return -1;
	t->ticks = ut + st;
	return 0;
}

/* a per-CPU counter file (/proc/softirqs, /proc/interrupts): each row's label and the sum of its counters */
int parse_percpu_table(const char *text, struct table *t)
{
	const char *line = strchr(text, '\n');	/* the CPU header */

	t->n = 0;
	while (line && *++line && t->n < MAX_ROWS) {
		const char *colon = strchr(line, ':'), *eol = strchr(line, '\n'), *p;
		struct row *r = &t->r[t->n];
		unsigned long long sum = 0;
		const char *lab = line;
		char *end;

		if (!eol)
			eol = line + strlen(line);
		if (!colon || colon > eol) {
			line = *eol ? eol : NULL;
			continue;
		}
		while (lab < colon && *lab == ' ')
			lab++;
		p = colon + 1;
		for (;;) {
			unsigned long long v = strtoull(p, &end, 10);

			if (end == p || end > eol)
				break;
			sum += v;
			p = end;
		}
		/* an IRQ row names its handler last: label "43 eth0" */
		while (p < eol && *p == ' ')
			p++;
		{
			const char *h = eol;

			while (h > p && h[-1] != ' ')
				h--;
			snprintf(r->name, sizeof(r->name), "%.*s%s%.*s", (int)(colon - lab), lab,
				 h < eol ? " " : "", (int)(eol - h), h);
		}
		r->v[0] = sum;
		t->n++;
		line = *eol ? eol : NULL;
	}
	return t->n;
}

/* /proc/net/dev -> per netdev rx packets, rx drop, tx packets, tx drop */
int parse_net_dev(const char *text, struct table *t)
{
	const char *line = text;
	int skip = 2;

	t->n = 0;
	while (line && *line && t->n < MAX_ROWS) {
		const char *eol = strchr(line, '\n');
		unsigned long long f[16];
		char name[NAME_LEN];

		if (skip > 0)
			skip--;
		else if (sscanf(line, " %63[^:]: %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu",
				name, &f[0], &f[1], &f[2], &f[3], &f[4], &f[5], &f[6], &f[7],
				&f[8], &f[9], &f[10], &f[11]) == 13) {
			struct row *r = &t->r[t->n++];

			snprintf(r->name, sizeof(r->name), "%s", name);
			r->v[0] = f[1];
			r->v[1] = f[3];
			r->v[2] = f[9];
			r->v[3] = f[11];
		}
		line = eol ? eol + 1 : NULL;
	}
	return t->n;
}

/* /proc/net/vlan/config: "gpon0.200 | 200 | gpon0" rows */
int parse_vlan_config(const char *text, struct vlan *v, int max)
{
	const char *line = text;
	int n = 0;

	while (line && *line && n < max) {
		const char *eol = strchr(line, '\n');

		if (sscanf(line, "%15s | %d | %15s", v[n].dev, &v[n].vid, v[n].parent) == 3 && v[n].vid > 0)
			n++;
		line = eol ? eol + 1 : NULL;
	}
	return n;
}

/* debugfs workqueue_stats: "name started completed cpu_us mayday" after a header */
int parse_wq_stats(const char *text, struct table *t)
{
	const char *line = strchr(text, '\n');

	t->n = 0;
	while (line && *++line && t->n < MAX_ROWS) {
		struct row *r = &t->r[t->n];
		const char *eol = strchr(line, '\n');

		if (sscanf(line, "%63s %llu %llu %llu %llu", r->name, &r->v[0], &r->v[1], &r->v[2], &r->v[3]) == 5) {
			size_t len = strlen(r->name);
			int j, seen = 0;

			/* 6.18 has two queues named "events": the n-th of a name is "name#n", so deltas pair them */
			for (j = 0; j < t->n; j++)
				seen += !strncmp(t->r[j].name, r->name, len) &&
					(!t->r[j].name[len] || t->r[j].name[len] == '#');
			if (seen)
				snprintf(r->name + len, sizeof(r->name) - len, "#%d", seen + 1);
			t->n++;
		}
		line = eol;
	}
	return t->n;
}

/* /proc/net/stat/nf_conntrack: per-CPU hex rows; the named columns summed */
int parse_ct_stat(const char *text, struct table *t)
{
	static const char *const want[] = { "insert", "insert_failed", "drop", "invalid", "early_drop" };
	char head[512];
	const char *line = strchr(text, '\n'), *p;
	int col[5], i, c;

	t->n = 0;
	if (!line || (size_t)(line - text) >= sizeof(head))
		return 0;
	memcpy(head, text, (size_t)(line - text));
	head[line - text] = 0;
	for (i = 0; i < 5; i++) {
		char *tok, *save = NULL, buf[512];

		snprintf(buf, sizeof(buf), "%s", head);
		col[i] = -1;
		for (c = 0, tok = strtok_r(buf, " ", &save); tok; tok = strtok_r(NULL, " ", &save), c++)
			if (!strcmp(tok, want[i]))
				col[i] = c;
		snprintf(t->r[i].name, NAME_LEN, "%s", want[i]);
		t->r[i].v[0] = 0;
	}
	t->n = 5;
	while (*++line) {
		unsigned long long v[24];
		char *end;
		int n = 0;

		for (p = line; n < 24; p = end) {
			v[n] = strtoull(p, &end, 16);
			if (end == p)
				break;
			n++;
		}
		for (i = 0; i < 5; i++)
			if (col[i] >= 0 && col[i] < n)
				t->r[i].v[0] += v[col[i]];
		line = strchr(line, '\n');
		if (!line)
			break;
	}
	return t->n;
}

/* the VLAN of a frame a packet socket read: the stripped tag when the kernel says so, else an in-band one,
 * else -1 (untagged) */
int frame_vid(int tag_valid, unsigned int tci, const unsigned char *data, size_t len)
{
	if (tag_valid)
		return (int)(tci & 0xfff);
	if (len >= 16 && data[12] == 0x81 && data[13] == 0x00)
		return ((data[14] << 8) | data[15]) & 0xfff;
	return -1;
}

int pkttype_kind(int pkttype)
{
	switch (pkttype) {
	case PACKET_HOST:
		return 0;
	case PACKET_BROADCAST:
		return 1;
	case PACKET_MULTICAST:
		return 2;
	default:
		return 3;
	}
}

/* ---- readers ---- */

static int read_file(const char *path, char *buf, size_t size)
{
	FILE *f = fopen(path, "r");
	size_t n;

	if (!f)
		return -1;
	n = fread(buf, 1, size - 1, f);
	buf[n] = 0;
	if (ferror(f)) {
		fclose(f);
		return -1;
	}
	fclose(f);
	return (int)n;
}

static double now_s(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts))
		return 0;
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

static char big[65536];

static void read_tasks(struct state *s)
{
	DIR *d = opendir("/proc");
	struct dirent *e;
	char path[300], line[512];

	s->n_tasks = 0;
	if (!d)
		return;
	while ((e = readdir(d)) && s->n_tasks < MAX_TASKS) {
		if (e->d_name[0] < '1' || e->d_name[0] > '9')
			continue;
		snprintf(path, sizeof(path), "/proc/%s/stat", e->d_name);
		if (read_file(path, line, sizeof(line)) > 0 && !parse_pid_stat(line, &s->tasks[s->n_tasks]))
			s->n_tasks++;
	}
	closedir(d);
}

static int read_state(struct state *s)
{
	char *l;

	memset(s, 0, sizeof(*s));
	s->t = now_s();
	if (read_file("/proc/stat", big, sizeof(big)) < 0 || !parse_cpu_line(big, s->cpu))
		return -1;
	if ((l = strstr(big, "procs_running ")))
		s->running = strtoull(l + 14, NULL, 10);
	if ((l = strstr(big, "procs_blocked ")))
		s->blocked = strtoull(l + 14, NULL, 10);
	if (read_file("/proc/softirqs", big, sizeof(big)) > 0)
		parse_percpu_table(big, &s->soft);
	if (read_file("/proc/interrupts", big, sizeof(big)) > 0)
		parse_percpu_table(big, &s->irq);
	if (read_file("/proc/net/dev", big, sizeof(big)) > 0)
		parse_net_dev(big, &s->dev);
	s->have_wq = read_file("/sys/kernel/debug/workqueue_stats", big, sizeof(big)) > 0 &&
		     parse_wq_stats(big, &s->wq) >= 0;
	s->have_ct = read_file("/proc/net/stat/nf_conntrack", big, sizeof(big)) > 0 && parse_ct_stat(big, &s->ct);
	if (read_file("/proc/sys/net/netfilter/nf_conntrack_count", big, sizeof(big)) > 0)
		s->ct_count = strtoull(big, NULL, 10);
	read_tasks(s);
	return 0;
}

static const struct row *find_row(const struct table *t, const char *name)
{
	int i;

	for (i = 0; i < t->n; i++)
		if (!strcmp(t->r[i].name, name))
			return &t->r[i];
	return NULL;
}

static unsigned long long task_before(const struct state *a, const struct task *t)
{
	int i;

	for (i = 0; i < a->n_tasks; i++)
		if (a->tasks[i].pid == t->pid)
			return a->tasks[i].ticks;
	return t->ticks;	/* born in the window: its age is unknown, count nothing */
}

/* ---- the VLAN sampler: which VLANs the VLAN parents receive, and whether a netdev claims them ---- */

static int vid_slot(struct vid_count *c, int *n, const char *parent, int vid, int kind, int claimed)
{
	int i;

	for (i = 0; i < *n; i++)
		if (c[i].vid == vid && c[i].kind == kind && !strcmp(c[i].parent, parent))
			return i;
	if (*n == MAX_VIDS)
		return -1;
	snprintf(c[*n].parent, IFNAMSIZ, "%.*s", IFNAMSIZ - 1, parent);
	c[*n].vid = vid;
	c[*n].kind = kind;
	c[*n].claimed = claimed;
	c[*n].n = 0;
	return (*n)++;
}

static int claimed_vid(const struct vlan *v, int n_vlan, const char *parent, int vid)
{
	int i;

	if (vid < 0)
		return 1;	/* untagged belongs to the parent itself */
	for (i = 0; i < n_vlan; i++)
		if (v[i].vid == vid && !strcmp(v[i].parent, parent))
			return 1;
	return 0;
}

/* Sample the VLAN parents for `window` s, at most SAMPLE_CAP frames. -> rows, sampled seconds in *secs */
static int sample_vids(double window, struct vid_count *c, double *secs, char *why, size_t why_len)
{
	static struct vlan v[MAX_ROWS];
	char parents[MAX_PARENTS][IFNAMSIZ];
	struct pollfd pfd[MAX_PARENTS];
	int n_vlan, n_par = 0, n = 0, i, j;
	unsigned long total = 0;
	double t0 = now_s();

	*secs = 0;
	if (read_file("/proc/net/vlan/config", big, sizeof(big)) < 0) {
		snprintf(why, why_len, "no /proc/net/vlan/config");
		return 0;
	}
	n_vlan = parse_vlan_config(big, v, MAX_ROWS);
	for (i = 0; i < n_vlan; i++) {
		for (j = 0; j < n_par && strcmp(parents[j], v[i].parent); j++)
			;
		if (j == n_par && n_par < MAX_PARENTS)
			snprintf(parents[n_par++], IFNAMSIZ, "%s", v[i].parent);
	}
	for (i = 0; i < n_par; i++) {
		struct sockaddr_ll sll = { .sll_family = AF_PACKET, .sll_protocol = htons(ETH_P_ALL) };
		int one = 1;

		pfd[i].events = POLLIN;
		pfd[i].fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
		sll.sll_ifindex = (int)if_nametoindex(parents[i]);
		if (pfd[i].fd < 0 || !sll.sll_ifindex ||
		    setsockopt(pfd[i].fd, SOL_PACKET, PACKET_AUXDATA, &one, sizeof(one)) ||
		    bind(pfd[i].fd, (struct sockaddr *)&sll, sizeof(sll))) {
			snprintf(why, why_len, "packet socket on %s: %s", parents[i], strerror(errno));
			if (pfd[i].fd >= 0)
				close(pfd[i].fd);
			pfd[i].fd = -1;
		}
	}
	while (n_par && total < SAMPLE_CAP && now_s() - t0 < window) {
		int ms = (int)((window - (now_s() - t0)) * 1000) + 1;

		if (poll(pfd, (nfds_t)n_par, ms) < 0) {
			if (errno == EINTR)
				continue;
			snprintf(why, why_len, "poll: %s", strerror(errno));
			break;
		}
		for (i = 0; i < n_par; i++) {
			unsigned char data[64], ctl[CMSG_SPACE(sizeof(struct tpacket_auxdata))];
			struct sockaddr_ll from;
			struct iovec iov = { data, sizeof(data) };
			struct msghdr m = { .msg_name = &from, .msg_namelen = sizeof(from), .msg_iov = &iov,
					    .msg_iovlen = 1, .msg_control = ctl, .msg_controllen = sizeof(ctl) };
			struct cmsghdr *cm;
			int valid = 0, vid, k;
			unsigned int tci = 0;
			ssize_t len;

			if (pfd[i].fd < 0 || !(pfd[i].revents & POLLIN))
				continue;
			len = recvmsg(pfd[i].fd, &m, MSG_TRUNC | MSG_DONTWAIT);
			if (len < 0 || from.sll_pkttype == PACKET_OUTGOING)
				continue;
			for (cm = CMSG_FIRSTHDR(&m); cm; cm = CMSG_NXTHDR(&m, cm))
				if (cm->cmsg_level == SOL_PACKET && cm->cmsg_type == PACKET_AUXDATA) {
					struct tpacket_auxdata aux;

					memcpy(&aux, CMSG_DATA(cm), sizeof(aux));
					valid = !!(aux.tp_status & TP_STATUS_VLAN_VALID);
					tci = aux.tp_vlan_tci;
				}
			vid = frame_vid(valid, tci, data, (size_t)len < sizeof(data) ? (size_t)len : sizeof(data));
			k = vid_slot(c, &n, parents[i], vid, pkttype_kind(from.sll_pkttype),
				     claimed_vid(v, n_vlan, parents[i], vid));
			if (k >= 0)
				c[k].n++;
			total++;
		}
	}
	*secs = now_s() - t0;
	for (i = 0; i < n_par; i++)
		if (pfd[i].fd >= 0)
			close(pfd[i].fd);
	if (!n_par && !*why)
		snprintf(why, why_len, "no VLAN netdev: nothing to attribute");
	return n;
}

/* ---- the report ---- */

static int by_delta_desc(const void *a, const void *b)
{
	const struct row *x = a, *y = b;

	return x->v[0] < y->v[0] ? 1 : x->v[0] > y->v[0] ? -1 : 0;
}

/* rows of b minus rows of a (same name), column `col`, sorted, into out */
static int deltas(const struct table *a, const struct table *b, int col, struct table *out)
{
	int i;

	out->n = 0;
	for (i = 0; i < b->n; i++) {
		const struct row *r = find_row(a, b->r[i].name);
		struct row *o = &out->r[out->n];

		if (!r)
			continue;
		*o = b->r[i];
		o->v[0] = b->r[i].v[col] >= r->v[col] ? b->r[i].v[col] - r->v[col] : 0;
		o->v[1] = b->r[i].v[2] >= r->v[2] ? b->r[i].v[2] - r->v[2] : 0;
		out->n++;
	}
	qsort(out->r, (size_t)out->n, sizeof(out->r[0]), by_delta_desc);
	return out->n;
}

static int by_ticks_desc(const void *a, const void *b)
{
	const struct task *x = a, *y = b;

	return x->ticks < y->ticks ? 1 : x->ticks > y->ticks ? -1 : 0;
}

static int by_count_desc(const void *a, const void *b)
{
	const struct vid_count *x = a, *y = b;

	return x->n < y->n ? 1 : x->n > y->n ? -1 : 0;
}

static void pcsample(FILE *o)
{
	FILE *p;
	char line[256];
	int n = 0;

	if (access("/usr/sbin/pcsample", X_OK)) {
		fprintf(o, "pcsample: absent from this image\n");
		return;
	}
	p = popen("/usr/sbin/pcsample 2 1 2>&1", "r");
	if (!p) {
		fprintf(o, "pcsample: %s\n", strerror(errno));
		return;
	}
	while (fgets(line, sizeof(line), p))
		if (n++ < 16)
			fprintf(o, "pc %s", line);
	if (pclose(p))
		fprintf(o, "pcsample: exited non-zero\n");
}

static struct state sa, sb;
static struct task top[MAX_TASKS];

/* One report over `window` s into a malloc'd buffer: its first lines are the summary */
static char *report(double window, int with_pc)
{
	static struct table d;
	static struct vid_count vc[MAX_VIDS];
	char *buf = NULL, why[160] = "";
	size_t size = 0;
	FILE *o = open_memstream(&buf, &size);
	unsigned long long tot = 0, busy;
	double dt, secs;
	int i, n, n_vid, shown;
	time_t wall = time(NULL);

	if (!o)
		return NULL;
	if (read_state(&sa)) {
		fprintf(o, "cpu-attrib: /proc/stat unreadable\n");
		fclose(o);
		return buf;
	}
	n_vid = sample_vids(window, vc, &secs, why, sizeof(why));
	if (now_s() - sa.t < window)
		usleep((useconds_t)((window - (now_s() - sa.t)) * 1e6));
	if (read_state(&sb)) {
		fprintf(o, "cpu-attrib: /proc/stat unreadable\n");
		fclose(o);
		return buf;
	}
	dt = sb.t - sa.t;
	for (i = 0; i < 10; i++)
		tot += sb.cpu[i] - sa.cpu[i];
	if (!tot)
		tot = 1;
	busy = tot - (sb.cpu[3] - sa.cpu[3]) - (sb.cpu[4] - sa.cpu[4]);
	fprintf(o, "cpu-attrib %ld window %.1fs busy %llu%% usr %llu sys %llu irq %llu softirq %llu | running %llu blocked %llu\n",
		(long)wall, dt, 100 * busy / tot, 100 * (sb.cpu[0] + sb.cpu[1] - sa.cpu[0] - sa.cpu[1]) / tot,
		100 * (sb.cpu[2] - sa.cpu[2]) / tot, 100 * (sb.cpu[5] - sa.cpu[5]) / tot,
		100 * (sb.cpu[6] - sa.cpu[6]) / tot, sb.running, sb.blocked);

	n = 0;
	for (i = 0; i < sb.n_tasks; i++) {
		top[n] = sb.tasks[i];
		top[n].ticks -= task_before(&sa, &sb.tasks[i]);
		if (top[n].ticks || top[n].state == 'R' || top[n].state == 'D')
			n++;
	}
	qsort(top, (size_t)n, sizeof(top[0]), by_ticks_desc);
	fprintf(o, "tasks:");
	for (i = 0, shown = 0; i < n && shown < 6; i++, shown++)
		fprintf(o, " %s %c %.1f%%", top[i].comm, top[i].state, 100.0 * top[i].ticks / tot);
	for (i = 6; i < n; i++)
		if (top[i].state == 'D')
			fprintf(o, " %s D", top[i].comm);
	fprintf(o, "\n");

	fprintf(o, "softirq/s:");
	deltas(&sa.soft, &sb.soft, 0, &d);
	for (i = 0; i < d.n && d.r[i].v[0]; i++)
		fprintf(o, " %s %.0f", d.r[i].name, d.r[i].v[0] / dt);
	fprintf(o, "\n");

	fprintf(o, "wq items/s cpu%%:");
	if (sa.have_wq && sb.have_wq) {
		deltas(&sa.wq, &sb.wq, 0, &d);
		for (i = 0; i < d.n && i < 8 && d.r[i].v[0]; i++)
			fprintf(o, " %s %.0f %.0f%%", d.r[i].name, d.r[i].v[0] / dt, d.r[i].v[1] / dt / 1e4);
	} else {
		fprintf(o, " not readable (/sys/kernel/debug/workqueue_stats)");
	}
	fprintf(o, "\n");

	fprintf(o, "netdev rx/s rxdrop/s tx/s:");
	deltas(&sa.dev, &sb.dev, 0, &d);
	for (i = 0; i < d.n && i < 8 && d.r[i].v[0]; i++) {
		const struct row *ra = find_row(&sa.dev, d.r[i].name), *rb = find_row(&sb.dev, d.r[i].name);

		fprintf(o, " %s %.0f %.0f %.0f", d.r[i].name, d.r[i].v[0] / dt,
			(rb->v[1] - ra->v[1]) / dt, (rb->v[2] - ra->v[2]) / dt);
	}
	fprintf(o, "\n");

	qsort(vc, (size_t)n_vid, sizeof(vc[0]), by_count_desc);
	fprintf(o, "unclaimed vlan rx/s (%.1fs sampled%s%s):", secs, *why ? ", " : "", why);
	for (i = 0, shown = 0; i < n_vid && shown < 6; i++)
		if (!vc[i].claimed && secs > 0) {
			fprintf(o, " %s vid %d %s %.0f", vc[i].parent, vc[i].vid, KIND[vc[i].kind], vc[i].n / secs);
			shown++;
		}
	fprintf(o, "\n");

	fprintf(o, "conntrack %llu", sb.ct_count);
	if (sa.have_ct && sb.have_ct)
		for (i = 0; i < sb.ct.n; i++)
			fprintf(o, " %s/s %.0f", sb.ct.r[i].name, (sb.ct.r[i].v[0] - sa.ct.r[i].v[0]) / dt);
	fprintf(o, "\n");

	/* the detail after the summary */
	fprintf(o, "-- claimed vlan rx/s:");
	for (i = 0; i < n_vid; i++)
		if (vc[i].claimed && secs > 0)
			fprintf(o, " %s vid %d %s %.0f", vc[i].parent, vc[i].vid, KIND[vc[i].kind], vc[i].n / secs);
	fprintf(o, "\n-- irq/s:");
	deltas(&sa.irq, &sb.irq, 0, &d);
	for (i = 0; i < d.n && i < 8 && d.r[i].v[0]; i++)
		fprintf(o, " %s %.0f", d.r[i].name, d.r[i].v[0] / dt);
	fprintf(o, "\n-- tasks:");
	for (i = 6; i < n && i < 20; i++)
		fprintf(o, " %d %s %c %llu", top[i].pid, top[i].comm, top[i].state, top[i].ticks);
	fprintf(o, "\n");
	if (with_pc)
		pcsample(o);
	if (fclose(o))
		return NULL;
	return buf;
}

/* ---- the ring and the kernel log ---- */

static int ring_write(const char *text)
{
	char path[64], tmp[80];
	unsigned long seq = 0;
	FILE *f;

	if (mkdir(RING_DIR, 0755) && errno != EEXIST)
		return -1;
	if (read_file(RING_DIR "/seq", big, sizeof(big)) > 0)
		seq = strtoul(big, NULL, 10) + 1;
	snprintf(path, sizeof(path), RING_DIR "/%lu", seq % RING_N);
	snprintf(tmp, sizeof(tmp), "%s.tmp", path);
	f = fopen(tmp, "w");
	if (!f)
		return -1;
	if (fprintf(f, "#%lu\n%s", seq, text) < 0 || fclose(f) || rename(tmp, path))
		return -1;
	f = fopen(RING_DIR "/seq", "w");
	if (!f)
		return -1;
	if (fprintf(f, "%lu\n", seq) < 0)
		return fclose(f), -1;
	return fclose(f) ? -1 : 0;
}

static void kmsg_summary(const char *text)
{
	FILE *k = fopen("/dev/kmsg", "w");
	const char *p = text;
	int n = 0;

	if (!k) {
		fprintf(stderr, "cpu-attrib: /dev/kmsg: %s\n", strerror(errno));
		return;
	}
	setvbuf(k, NULL, _IONBF, 0);
	while (*p && n++ < KMSG_LINES && strncmp(p, "--", 2)) {
		const char *eol = strchr(p, '\n');
		int len = eol ? (int)(eol - p) : (int)strlen(p);

		/* one write per record; kmsg takes up to ~1000 bytes */
		if (fprintf(k, "<4>cpu-attrib: %.*s\n", len > 900 ? 900 : len, p) < 0)
			fprintf(stderr, "cpu-attrib: kmsg write: %s\n", strerror(errno));
		p = eol ? eol + 1 : p + len;
	}
	fclose(k);
}

static int show(void)
{
	unsigned long seq, s;
	char path[64];

	if (read_file(RING_DIR "/seq", big, sizeof(big)) <= 0) {
		printf("cpu-attrib: no report kept (the CPU never stayed busy, or the watch is not running)\n");
		return 0;
	}
	seq = strtoul(big, NULL, 10);
	for (s = seq >= RING_N - 1 ? seq - (RING_N - 1) : 0; s <= seq; s++) {
		snprintf(path, sizeof(path), RING_DIR "/%lu", s % RING_N);
		if (read_file(path, big, sizeof(big)) > 0)
			fputs(big, stdout);
	}
	return 0;
}

static int watch(int period, int busy_pct, int periods, double window, int holdoff)
{
	unsigned long long a[10], b[10];
	int streak = 0, i;
	double quiet_until = 0;

	if (setpriority(PRIO_PROCESS, 0, -20))
		fprintf(stderr, "cpu-attrib: setpriority: %s (reports may come late under load)\n", strerror(errno));
	if (read_file("/proc/stat", big, sizeof(big)) < 0 || !parse_cpu_line(big, a)) {
		fprintf(stderr, "cpu-attrib: /proc/stat unreadable\n");
		return 1;
	}
	for (;;) {
		unsigned long long tot = 0, idle;

		sleep((unsigned int)period);
		if (read_file("/proc/stat", big, sizeof(big)) < 0 || !parse_cpu_line(big, b)) {
			fprintf(stderr, "cpu-attrib: /proc/stat unreadable\n");
			return 1;
		}
		for (i = 0; i < 10; i++)
			tot += b[i] - a[i];
		idle = b[3] - a[3] + b[4] - a[4];
		memcpy(a, b, sizeof(a));
		streak = tot && 100 * (tot - idle) >= (unsigned long long)busy_pct * tot ? streak + 1 : 0;
		if (streak >= periods && now_s() >= quiet_until) {
			char *r = report(window, 1);

			if (!r) {
				fprintf(stderr, "cpu-attrib: report: %s\n", strerror(errno));
			} else {
				kmsg_summary(r);
				if (ring_write(r))
					fprintf(stderr, "cpu-attrib: ring %s: %s\n", RING_DIR, strerror(errno));
				free(r);
			}
			quiet_until = now_s() + holdoff;
			streak = 0;
			if (read_file("/proc/stat", big, sizeof(big)) < 0 || !parse_cpu_line(big, a))
				return 1;
		}
	}
}

#ifndef CPU_ATTRIB_NO_MAIN
int main(int argc, char **argv)
{
	int period = 5, busy = 90, periods = 3, holdoff = 300, opt;
	double window = 5;

	if (argc < 2) {
		fprintf(stderr, "usage: cpu-attrib watch|snap|show [-p s] [-b %%] [-n periods] [-w s] [-H s]\n");
		return 2;
	}
	optind = 2;
	while ((opt = getopt(argc, argv, "p:b:n:w:H:")) != -1) {
		switch (opt) {
		case 'p': period = atoi(optarg); break;
		case 'b': busy = atoi(optarg); break;
		case 'n': periods = atoi(optarg); break;
		case 'w': window = atof(optarg); break;
		case 'H': holdoff = atoi(optarg); break;
		default: return 2;
		}
	}
	if (period < 1 || busy < 1 || busy > 100 || periods < 1 || window < 1 || window > 60 || holdoff < 0) {
		fprintf(stderr, "cpu-attrib: an option is out of range\n");
		return 2;
	}
	if (!strcmp(argv[1], "show"))
		return show();
	if (!strcmp(argv[1], "snap")) {
		char *r = report(window, 1);

		if (!r) {
			fprintf(stderr, "cpu-attrib: report: %s\n", strerror(errno));
			return 1;
		}
		fputs(r, stdout);
		free(r);
		return 0;
	}
	if (!strcmp(argv[1], "watch"))
		return watch(period, busy, periods, window, holdoff);
	fprintf(stderr, "cpu-attrib: unknown command %s\n", argv[1]);
	return 2;
}
#endif
