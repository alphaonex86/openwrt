// SPDX-License-Identifier: GPL-2.0-or-later
/* TIER: CORE */
/* onu-cwmp: a TR-069 CPE answering from a data model file, on the management interface alone (SO_BINDTODEVICE:
 * the internet's routes are never involved). One session at a time; HTTP as the CPE it stands in for. */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include "cwmp_core.h"
#include "md5.h"

#define MAX_ROUNDS 2000

struct opts {
	const char *url, *user, *pass, *model, *sets, *ifname, *dns, *agent, *inform_names, *state;
	int conn_port, once;
};

/* A Download's outcome, kept until the ACS has its TransferComplete, across the reboot a written firmware makes. */
struct transfer {
	char key[256], why[256], start[64], complete[64];
	int fault;
};

static const char *sysdir;		/* the system commands' directory (-x, a host test); else /sbin */

struct acs {
	char host[256], path[512], ip[64];
	int port, fd;
	char cookie[1024], scheme[16], realm[256], nonce[256], opaque[256], qop[32];
	unsigned int nc;
};

/* @argv[0], a system command run without a shell, its stdout into @out when given. -> its exit status, or -1. */
static int run(char *const argv[], char *out, size_t len)
{
	char path[512];
	int pfd[2] = { -1, -1 }, status;
	size_t n = 0;
	pid_t pid;

	if (sysdir)
		snprintf(path, sizeof(path), "%s/%s", sysdir, argv[0]);
	else
		snprintf(path, sizeof(path), "%s/%s", strcmp(argv[0], "onu-cwmp-firmware") ? "/sbin" : "/usr/libexec",
			 argv[0]);
	if (out && pipe(pfd))
		return -1;
	pid = fork();
	if (pid < 0) {
		syslog(LOG_ERR, "fork for %s: %s", path, strerror(errno));
		if (out) {
			close(pfd[0]);
			close(pfd[1]);
		}
		return -1;
	}
	if (!pid) {
		int null = open("/dev/null", O_RDWR);

		dup2(null, 0);
		dup2(out ? pfd[1] : null, 1);
		dup2(null, 2);
		execv(path, argv);
		_exit(127);
	}
	if (out) {
		char tmp[256];
		ssize_t got;

		close(pfd[1]);
		while ((got = read(pfd[0], tmp, sizeof(tmp))) > 0) {
			if ((size_t)got > len - 1 - n)
				got = len - 1 - n;
			memcpy(out + n, tmp, got);
			n += got;
		}
		out[n] = 0;
		close(pfd[0]);
	}
	if (waitpid(pid, &status, 0) < 0)
		return -1;
	return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

static int uci_get(const char *option, char *out, size_t len)
{
	char arg[300], *argv[] = { (char *)"uci", (char *)"-q", (char *)"get", arg, NULL };

	snprintf(arg, sizeof(arg), "%s", option);
	if (run(argv, out, len))
		return -1;
	out[strcspn(out, "\n")] = 0;
	return 0;
}

/* A new value staged, committed after the session; NULL reverts what this option's config has staged. */
static int uci_set(const char *option, const char *value)
{
	char arg[1300], *set[] = { (char *)"uci", (char *)"set", arg, NULL };
	char *revert[] = { (char *)"uci", (char *)"revert", arg, NULL };

	if (!value) {
		snprintf(arg, sizeof(arg), "%.*s", (int)strcspn(option, "."), option);
		return run(revert, NULL, 0) ? -1 : 0;
	}
	if (snprintf(arg, sizeof(arg), "%s=%s", option, value) >= (int)sizeof(arg))
		return -1;
	if (run(set, NULL, 0)) {
		syslog(LOG_WARNING, "uci refused %s", option);
		return -1;
	}
	return 0;
}

static void iso_now(char *out, size_t len)
{
	time_t t = time(NULL);

	strftime(out, len, "%Y-%m-%dT%H:%M:%S+00:00", gmtime(&t));
}

static int if_ip(const char *ifname, char *out, size_t len)
{
	struct ifaddrs *ifa, *i;
	int rc = -1;

	if (getifaddrs(&ifa))
		return -1;
	for (i = ifa; i; i = i->ifa_next)
		if (i->ifa_addr && i->ifa_addr->sa_family == AF_INET && !strcmp(i->ifa_name, ifname)) {
			inet_ntop(AF_INET, &((struct sockaddr_in *)i->ifa_addr)->sin_addr, out, len);
			rc = 0;
			break;
		}
	freeifaddrs(ifa);
	return rc;
}

static int bound_socket(int type, const char *ifname)
{
	int fd = socket(AF_INET, type, 0);

	if (fd >= 0 && setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, ifname, strlen(ifname) + 1)) {
		syslog(LOG_ERR, "SO_BINDTODEVICE %s: %s", ifname, strerror(errno));
		close(fd);
		return -1;
	}
	return fd;
}

/* One A query to @server, through the management interface. */
static int dns_a(const char *server, const char *name, const char *ifname, char *out, size_t len)
{
	unsigned char q[512], r[512];
	struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(53) };
	struct pollfd pfd;
	size_t n = 12, i;
	int fd, got, tries;
	const char *l = name;

	memset(q, 0, sizeof(q));
	q[0] = 0x2a;
	q[1] = 0x17;
	q[2] = 1;
	q[5] = 1;
	while (*l && n < 400) {
		size_t k = strcspn(l, ".");

		q[n++] = k;
		memcpy(q + n, l, k);
		n += k;
		l += k + (l[k] == '.');
	}
	q[n++] = 0;
	q[n++] = 0, q[n++] = 1, q[n++] = 0, q[n++] = 1;
	if (inet_pton(AF_INET, server, &sa.sin_addr) != 1 || (fd = bound_socket(SOCK_DGRAM, ifname)) < 0)
		return -1;
	for (tries = 0; tries < 3; tries++) {
		sendto(fd, q, n, 0, (struct sockaddr *)&sa, sizeof(sa));
		pfd.fd = fd;
		pfd.events = POLLIN;
		if (poll(&pfd, 1, 3000) <= 0 || (got = recv(fd, r, sizeof(r), 0)) < 12 || r[0] != q[0] || r[1] != q[1])
			continue;
		{
			unsigned int an = r[6] << 8 | r[7];

			i = 12;
			while (i < (size_t)got && r[i] && (r[i] & 0xc0) != 0xc0)
				i += r[i] + 1;
			i += (i < (size_t)got && (r[i] & 0xc0) == 0xc0) ? 2 : 1;
			i += 4;				/* QTYPE, QCLASS */
			while (an-- && i + 10 <= (size_t)got) {
				unsigned int type, rdlen;

				while (i < (size_t)got && r[i] && (r[i] & 0xc0) != 0xc0)
					i += r[i] + 1;
				i += (i < (size_t)got && (r[i] & 0xc0) == 0xc0) ? 2 : 1;
				type = r[i] << 8 | r[i + 1];
				rdlen = r[i + 8] << 8 | r[i + 9];
				i += 10;
				if (type == 1 && rdlen == 4 && i + 4 <= (size_t)got) {
					inet_ntop(AF_INET, r + i, out, len);
					close(fd);
					return 0;
				}
				i += rdlen;
			}
		}
	}
	close(fd);
	return -1;
}

static int url_parse(struct acs *a, const char *url)
{
	const char *h, *c, *p;

	if (strncmp(url, "http://", 7))
		return -1;
	h = url + 7;
	p = strchr(h, '/');
	c = memchr(h, ':', p ? (size_t)(p - h) : strlen(h));
	snprintf(a->host, sizeof(a->host), "%.*s", (int)((c ? c : p ? p : h + strlen(h)) - h), h);
	a->port = c ? atoi(c + 1) : 80;
	snprintf(a->path, sizeof(a->path), "%s", p ? p : "/");
	return 0;
}

static int acs_connect(struct acs *a, const struct opts *o)
{
	struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(a->port) };

	if (inet_pton(AF_INET, a->host, &sa.sin_addr) == 1) {
		snprintf(a->ip, sizeof(a->ip), "%.63s", a->host);
	} else if (!o->dns || dns_a(o->dns, a->host, o->ifname, a->ip, sizeof(a->ip))) {
		syslog(LOG_WARNING, "cannot resolve %s through %s", a->host, o->dns ? o->dns : "(no DNS)");
		return -1;
	}
	inet_pton(AF_INET, a->ip, &sa.sin_addr);
	a->fd = bound_socket(SOCK_STREAM, o->ifname);
	if (a->fd < 0 || connect(a->fd, (struct sockaddr *)&sa, sizeof(sa))) {
		syslog(LOG_WARNING, "cannot reach the ACS %s (%s:%d): %s", a->host, a->ip, a->port, strerror(errno));
		if (a->fd >= 0)
			close(a->fd);
		a->fd = -1;
		return -1;
	}
	return 0;
}

static void header_param(const char *h, const char *key, char *out, size_t len)
{
	const char *p = strstr(h, key);

	out[0] = 0;
	if (!p)
		return;
	p += strlen(key);
	if (*p == '"')
		snprintf(out, len, "%.*s", (int)strcspn(p + 1, "\""), p + 1);
	else
		snprintf(out, len, "%.*s", (int)strcspn(p, ", \r\n"), p);
}

static void authorization(struct acs *a, const char *user, const char *pass, const char *method, struct buf *h)
{
	char ha1[33], ha2[33], resp[33], cnonce[17], tmp[1024];

	if (!a->scheme[0])
		return;
	if (!strcmp(a->scheme, "Basic")) {
		static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
		unsigned char in[512];
		size_t n = snprintf((char *)in, sizeof(in), "%s:%s", user, pass), i;

		buf_add(h, "Authorization: Basic ");
		for (i = 0; i < n; i += 3) {
			unsigned int v = in[i] << 16 | (i + 1 < n ? in[i + 1] << 8 : 0) | (i + 2 < n ? in[i + 2] : 0);

			buf_addf(h, "%c%c%c%c", b64[v >> 18 & 63], b64[v >> 12 & 63], i + 1 < n ? b64[v >> 6 & 63] : '=',
				 i + 2 < n ? b64[v & 63] : '=');
		}
		buf_add(h, "\r\n");
		return;
	}
	snprintf(tmp, sizeof(tmp), "%s:%s:%s", user, a->realm, pass);
	md5_hex(tmp, ha1);
	snprintf(tmp, sizeof(tmp), "%s:%s", method, a->path);
	md5_hex(tmp, ha2);
	snprintf(cnonce, sizeof(cnonce), "%08lx%08x", (unsigned long)time(NULL), ++a->nc);
	if (a->qop[0])
		snprintf(tmp, sizeof(tmp), "%s:%s:%08x:%s:auth:%s", ha1, a->nonce, a->nc, cnonce, ha2);
	else
		snprintf(tmp, sizeof(tmp), "%s:%s:%s", ha1, a->nonce, ha2);
	md5_hex(tmp, resp);
	buf_addf(h, "Authorization: Digest username=\"%s\", realm=\"%s\", nonce=\"%s\", uri=\"%s\", response=\"%s\"",
		 user, a->realm, a->nonce, a->path, resp);
	if (a->opaque[0])
		buf_addf(h, ", opaque=\"%s\"", a->opaque);
	if (a->qop[0])
		buf_addf(h, ", qop=auth, nc=%08x, cnonce=\"%s\"", a->nc, cnonce);
	buf_add(h, "\r\n");
}

/* Body bytes to @sink when given, else to @body. -> 0, or -1 when the file refused them. */
static int body_take(struct buf *body, FILE *sink, const char *p, size_t n)
{
	if (sink)
		return fwrite(p, 1, n, sink) == n ? 0 : -1;
	buf_addn(body, p, n);
	return 0;
}

/* The response: status, its body (into @sink when given); the session's cookie and challenge from the headers. */
static int read_response(struct acs *a, struct buf *body, FILE *sink)
{
	char head[8192];
	size_t hn = 0;
	long clen = -1;
	int status = 0, chunked = 0;
	char *line, *save;

	body->n = 0;
	while (hn + 1 < sizeof(head)) {
		if (recv(a->fd, head + hn, 1, 0) != 1)
			return -1;
		hn++;
		if (hn >= 4 && !memcmp(head + hn - 4, "\r\n\r\n", 4))
			break;
	}
	head[hn] = 0;
	sscanf(head, "HTTP/%*s %d", &status);
	for (line = strtok_r(head, "\r\n", &save); line; line = strtok_r(NULL, "\r\n", &save)) {
		if (!strncasecmp(line, "Content-Length:", 15))
			clen = atol(line + 15);
		else if (!strncasecmp(line, "Transfer-Encoding:", 18) && strstr(line, "chunked"))
			chunked = 1;
		else if (!strncasecmp(line, "Set-Cookie:", 11))
			snprintf(a->cookie, sizeof(a->cookie), "%.*s", (int)strcspn(line + 12, ";"), line + 12);
		else if (!strncasecmp(line, "WWW-Authenticate:", 17)) {
			const char *v = line + 17 + strspn(line + 17, " ");

			snprintf(a->scheme, sizeof(a->scheme), "%.*s", (int)strcspn(v, " "), v);
			header_param(v, "realm=", a->realm, sizeof(a->realm));
			header_param(v, "nonce=", a->nonce, sizeof(a->nonce));
			header_param(v, "opaque=", a->opaque, sizeof(a->opaque));
			header_param(v, "qop=", a->qop, sizeof(a->qop));
			a->nc = 0;
		}
	}
	if (chunked) {
		for (;;) {
			char sz[32];
			size_t k = 0;
			long n;
			char crlf[2];

			while (k + 1 < sizeof(sz) && recv(a->fd, sz + k, 1, 0) == 1 && sz[k] != '\n')
				k++;
			sz[k] = 0;
			n = strtol(sz, NULL, 16);
			if (n <= 0)
				break;
			while (n > 0) {
				char tmp[4096];
				int got = recv(a->fd, tmp, n < (long)sizeof(tmp) ? n : (long)sizeof(tmp), 0);

				if (got <= 0 || body_take(body, sink, tmp, got))
					return -1;
				n -= got;
			}
			if (recv(a->fd, crlf, 2, MSG_WAITALL) != 2)
				return -1;
		}
		recv(a->fd, head, 2, MSG_WAITALL);
	} else {
		while (clen > 0) {
			char tmp[4096];
			int got = recv(a->fd, tmp, clen < (long)sizeof(tmp) ? clen : (long)sizeof(tmp), 0);

			if (got <= 0 || body_take(body, sink, tmp, got))
				return -1;
			clen -= got;
		}
		while (clen < 0 && sink) {		/* no length: the body ends with the connection */
			char tmp[4096];
			int got = recv(a->fd, tmp, sizeof(tmp), 0);

			if (got <= 0)
				break;
			if (body_take(body, sink, tmp, got))
				return -1;
		}
	}
	if (!body->p)
		buf_add(body, "");
	return status;
}

/* One POST in the session, the challenge answered once. -> HTTP status, or -1. */
static int acs_post(struct acs *a, const struct opts *o, const struct buf *soap, struct buf *reply)
{
	int attempt, status = -1;

	for (attempt = 0; attempt < 2; attempt++) {
		struct buf h = { 0 };

		if (a->fd < 0 && acs_connect(a, o))
			return -1;
		buf_addf(&h, "POST %s HTTP/1.1\r\nHost: %s:%d\r\n", a->path, a->host, a->port);
		if (soap && soap->n)
			buf_add(&h, "SoapAction: \r\n");
		buf_add(&h, "Connection: Keep-Alive\r\n");
		if (soap && soap->n)
			buf_add(&h, "Content-Type: text/xml; charset=UTF-8\r\n");
		authorization(a, o->user, o->pass, "POST", &h);
		buf_addf(&h, "User-Agent: %s\r\n", o->agent);
		if (a->cookie[0])
			buf_addf(&h, "Cookie: %s\r\n", a->cookie);
		buf_addf(&h, "Content-Length: %zu\r\n\r\n", soap ? soap->n : 0);
		if (soap && soap->n)
			buf_addn(&h, soap->p, soap->n);
		if (send(a->fd, h.p, h.n, MSG_NOSIGNAL) != (ssize_t)h.n) {
			buf_free(&h);
			close(a->fd);
			a->fd = -1;
			continue;
		}
		buf_free(&h);
		status = read_response(a, reply, NULL);
		if (status < 0) {
			close(a->fd);
			a->fd = -1;
			continue;
		}
		if (status != 401 || !a->scheme[0])
			return status;
		close(a->fd);			/* answer the challenge on a fresh connection */
		a->fd = -1;
	}
	return status;
}

static int load_lines(const char *path, char lines[][256], size_t max)
{
	FILE *f = fopen(path, "r");
	int n = 0;

	if (!f)
		return 0;
	while ((size_t)n < max && fgets(lines[n], 256, f)) {
		lines[n][strcspn(lines[n], "\n")] = 0;
		if (lines[n][0])
			n++;
	}
	fclose(f);
	return n;
}

static void state_path(const struct opts *o, const char *name, char *out, size_t len)
{
	snprintf(out, len, "%s/%s", o->state, name);
}

/* A field kept on one line of a state file. */
static void flat(char *s)
{
	for (; *s; s++)
		if (*s == '\t' || *s == '\n' || *s == '\r')
			*s = ' ';
}

static int transfer_save(const struct opts *o, struct transfer *tr)
{
	char p[512], tmp[520];
	FILE *f;

	state_path(o, "transfer", p, sizeof(p));
	snprintf(tmp, sizeof(tmp), "%s.tmp", p);
	flat(tr->key);
	flat(tr->why);
	f = fopen(tmp, "w");
	if (!f)
		return -1;
	fprintf(f, "%s\t%d\t%s\t%s\t%s\n", tr->key, tr->fault, tr->start, tr->complete, tr->why);
	if (fflush(f) || fsync(fileno(f)) || fclose(f) || rename(tmp, p))
		return -1;
	return 0;
}

static int transfer_load(const struct opts *o, struct transfer *tr)
{
	char p[512], line[1024], *s = line, *f[5];
	FILE *fp;
	int i;

	state_path(o, "transfer", p, sizeof(p));
	fp = fopen(p, "r");
	if (!fp)
		return -1;
	if (!fgets(line, sizeof(line), fp))
		line[0] = 0;
	fclose(fp);
	line[strcspn(line, "\n")] = 0;
	for (i = 0; i < 5; i++)
		f[i] = strsep(&s, "\t");
	if (!f[4]) {
		syslog(LOG_ERR, "%s is not a transfer record: dropped", p);
		unlink(p);
		return -1;
	}
	snprintf(tr->key, sizeof(tr->key), "%s", f[0]);
	tr->fault = atoi(f[1]);
	snprintf(tr->start, sizeof(tr->start), "%s", f[2]);
	snprintf(tr->complete, sizeof(tr->complete), "%s", f[3]);
	snprintf(tr->why, sizeof(tr->why), "%s", f[4]);
	return 0;
}

/* One whole session: Inform, the pending TransferComplete, then the ACS's requests until it has none.
 * -> 0, or -1 to retry. */
static int session(const struct opts *o, struct model *m, const char *const *events, size_t n_events,
		   unsigned int retry, struct todo *t, const struct transfer *tc)
{
	static char names[64][256];
	const char *nm[64];
	char ip[64] = "", up[32], now[64], id[16];
	struct live lv = { ip, up, now, uci_get, uci_set };
	struct acs a = { .fd = -1 };
	struct buf out = { 0 }, in = { 0 };
	int n = load_lines(o->inform_names, names, 64), i, rounds, status, changed = 0, rc = -1;
	FILE *f;
	double secs = 0;

	for (i = 0; i < n; i++)
		nm[i] = names[i];
	if (if_ip(o->ifname, ip, sizeof(ip)) || url_parse(&a, o->url))
		return -1;
	f = fopen("/proc/uptime", "r");
	if (f) {
		if (fscanf(f, "%lf", &secs) != 1)
			secs = 0;
		fclose(f);
	}
	snprintf(up, sizeof(up), "%lu", (unsigned long)secs);
	iso_now(now, sizeof(now));
	snprintf(id, sizeof(id), "%u", (unsigned int)(time(NULL) % 100000));
	soap_inform(&out, id, m, &lv, events, n_events, nm, n, retry);
	status = acs_post(&a, o, &out, &in);
	if (status != 200 || !strstr(in.p, "InformResponse")) {
		syslog(LOG_WARNING, "Inform not accepted (HTTP %d)", status);
		goto done;
	}
	out.n = 0;
	if (tc) {
		soap_transfer_complete(&out, id, tc->key, tc->fault, tc->why, tc->start, tc->complete);
		status = acs_post(&a, o, &out, &in);
		if (status != 200 || !strstr(in.p, "TransferCompleteResponse")) {
			syslog(LOG_WARNING, "TransferComplete not accepted (HTTP %d)", status);
			goto done;
		}
		out.n = 0;
	}
	for (rounds = 0; rounds < MAX_ROUNDS; rounds++) {
		status = acs_post(&a, o, &out, &in);
		if (status == 204 || (status == 200 && !in.n)) {
			rc = 0;
			break;
		}
		if (status != 200) {
			syslog(LOG_WARNING, "ACS answered HTTP %d mid-session", status);
			break;
		}
		out.n = 0;
		if (soap_answer(&out, m, &lv, in.p, NULL, t, &changed)) {
			syslog(LOG_WARNING, "an ACS request this CPE does not understand: %.120s", in.p);
			break;
		}
	}
done:
	if (changed) {
		char p[512];

		state_path(o, "sets", p, sizeof(p));
		if (model_save_sets(m, o->sets ? o->sets : p))
			syslog(LOG_ERR, "the values the ACS set could not be saved");
	}
	if (a.fd >= 0)
		close(a.fd);
	buf_free(&out);
	buf_free(&in);
	return rc;
}

/* A Download's file over HTTP into @path, through the management interface. -> 0, else its TR-069 fault code. */
static int fetch(const struct opts *o, const struct todo *t, const char *path, char *why, size_t len)
{
	struct acs a = { .fd = -1 };
	struct buf body = { 0 };
	int attempt, status = -1;
	long got = -1;
	FILE *f = NULL;

	if (url_parse(&a, t->url)) {
		snprintf(why, len, "not an http:// URL");
		return 9013;
	}
	for (attempt = 0; attempt < 2; attempt++) {
		struct buf h = { 0 };

		f = fopen(path, "w");
		if (!f) {
			snprintf(why, len, "%s: %s", path, strerror(errno));
			return 9010;
		}
		if (acs_connect(&a, o)) {
			fclose(f);
			snprintf(why, len, "cannot reach %.200s:%d", a.host, a.port);
			return 9015;
		}
		buf_addf(&h, "GET %s HTTP/1.1\r\nHost: %s:%d\r\n", a.path, a.host, a.port);
		authorization(&a, t->user, t->pass, "GET", &h);
		buf_addf(&h, "User-Agent: %s\r\nConnection: close\r\n\r\n", o->agent);
		status = send(a.fd, h.p, h.n, MSG_NOSIGNAL) == (ssize_t)h.n ? read_response(&a, &body, f) : -1;
		buf_free(&h);
		close(a.fd);
		a.fd = -1;
		got = ftell(f);
		if (fclose(f))
			status = -1;
		if (status != 401 || !a.scheme[0] || !t->user[0])
			break;
	}
	buf_free(&body);
	if (status != 200) {
		snprintf(why, len, "the file server answered HTTP %d", status);
		return status == 401 ? 9019 : status == 404 ? 9016 : 9010;
	}
	if (t->size > 0 && got != t->size) {
		snprintf(why, len, "%ld bytes received, the ACS announced %ld", got, t->size);
		return 9017;
	}
	return 0;
}

/* A Download fetched, then handed to onu-cwmp-firmware, which writes it unless the flash already holds it. Its
 * outcome is saved before the write: the TransferComplete survives the reboot a written firmware makes. */
static void firmware(const struct opts *o, const struct todo *t)
{
	static const char path[] = "/tmp/onu-cwmp.firmware";
	char *helper[] = { (char *)"onu-cwmp-firmware", (char *)path, NULL }, out[256] = "";
	struct transfer tr = { .fault = 0 };
	int rc;

	snprintf(tr.key, sizeof(tr.key), "%s", t->download_key);
	iso_now(tr.start, sizeof(tr.start));
	tr.fault = fetch(o, t, path, tr.why, sizeof(tr.why));
	iso_now(tr.complete, sizeof(tr.complete));
	if (!tr.fault && transfer_save(o, &tr)) {
		tr.fault = 9002;
		snprintf(tr.why, sizeof(tr.why), "the transfer record could not be written: firmware not applied");
	}
	if (!tr.fault) {
		rc = run(helper, out, sizeof(out));
		out[strcspn(out, "\n")] = 0;
		if (rc) {
			tr.fault = rc == 1 ? 9018 : 9010;
			snprintf(tr.why, sizeof(tr.why), "%s", out[0] ? out : "onu-cwmp-firmware failed");
		}
		iso_now(tr.complete, sizeof(tr.complete));
	}
	unlink(path);
	if (transfer_save(o, &tr))
		syslog(LOG_ERR, "the Download's outcome could not be saved: no TransferComplete will tell the ACS");
	syslog(tr.fault ? LOG_WARNING : LOG_NOTICE, "Download %s: fault %d %s %s", t->url, tr.fault, tr.why, out);
}

/* What the ACS asked for, done once its session is over. -> 1 when a reboot is under way. */
static int after_session(const struct opts *o, const struct todo *t, const char *reboot_key_path, time_t *next)
{
	char *commit[] = { (char *)"uci", (char *)"commit", NULL }, *reload[] = { (char *)"reload_config", NULL };
	char *reset[] = { (char *)"firstboot", (char *)"-y", NULL }, *reboot[] = { (char *)"reboot", NULL };
	FILE *f;

	if (t->uci_changed && run(commit, NULL, 0))
		syslog(LOG_ERR, "uci commit failed: what the ACS set stays staged, lost at the next boot");
	if (t->url[0]) {
		firmware(o, t);
		*next = 0;			/* its TransferComplete now, in a session of its own */
	}
	if (t->factory_reset) {
		syslog(LOG_NOTICE, "the ACS asked for a factory reset");
		if (run(reset, NULL, 0)) {
			syslog(LOG_ERR, "firstboot -y failed: the configuration is kept, no reboot");
		} else {
			if (run(reboot, NULL, 0))
				syslog(LOG_ERR, "reboot failed after the factory reset");
			return 1;
		}
	}
	if (t->reboot) {
		f = fopen(reboot_key_path, "w");
		if (!f || fputs(t->reboot_key, f) < 0 || fclose(f))
			syslog(LOG_ERR, "the Reboot's CommandKey could not be saved: its M Reboot loses the key");
		syslog(LOG_NOTICE, "the ACS asked for a reboot");
		if (run(reboot, NULL, 0))
			syslog(LOG_ERR, "reboot failed");
		return 1;
	}
	if (t->uci_changed && run(reload, NULL, 0))
		syslog(LOG_ERR, "reload_config failed: what the ACS set applies at the next boot");
	return 0;
}

static int listener(const struct opts *o)
{
	struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(o->conn_port) };
	int fd = bound_socket(SOCK_STREAM, o->ifname), one = 1;

	if (fd < 0)
		return -1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) || listen(fd, 4)) {
		close(fd);
		return -1;
	}
	return fd;
}

/* A connection request: the path of our ConnectionRequestURL, Digest-authenticated when it has a password. */
static int connection_request(int lfd, struct model *m)
{
	struct param *u = model_find(m, "InternetGatewayDevice.ManagementServer.ConnectionRequestURL");
	struct param *un = model_find(m, "InternetGatewayDevice.ManagementServer.ConnectionRequestUsername");
	struct param *pw = model_find(m, "InternetGatewayDevice.ManagementServer.ConnectionRequestPassword");
	const char *want = u ? strchr(u->value + (strncmp(u->value, "http://", 7) ? 0 : 7), '/') : NULL;
	char req[4096], nonce[33], tmp[1024], ha1[33], ha2[33], resp[33], got[64], gnonce[64], uri[512], user[256];
	int fd = accept(lfd, NULL, NULL), n, ok = 0;

	if (fd < 0)
		return 0;
	n = recv(fd, req, sizeof(req) - 1, 0);
	req[n > 0 ? n : 0] = 0;
	header_param(req, "uri=", uri, sizeof(uri));
	header_param(req, "response=", got, sizeof(got));
	header_param(req, "nonce=", gnonce, sizeof(gnonce));
	header_param(req, "username=", user, sizeof(user));
	if (want && strncmp(req + 4, want, strlen(want))) {
		send(fd, "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n", 45, MSG_NOSIGNAL);
	} else if (!pw || !pw->value[0]) {
		ok = 1;
	} else if (got[0]) {
		snprintf(tmp, sizeof(tmp), "%s:IGD:%s", un ? un->value : "", pw->value);
		md5_hex(tmp, ha1);
		snprintf(tmp, sizeof(tmp), "GET:%s", uri);
		md5_hex(tmp, ha2);
		snprintf(tmp, sizeof(tmp), "%s:%s:%s", ha1, gnonce, ha2);
		md5_hex(tmp, resp);
		ok = !strcmp(resp, got) && un && !strcmp(user, un->value);
	}
	if (ok) {
		send(fd, "HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n", 38, MSG_NOSIGNAL);
	} else if (!want || !strncmp(req + 4, want, strlen(want))) {
		snprintf(tmp, sizeof(tmp), "%ld", (long)time(NULL));
		md5_hex(tmp, nonce);
		n = snprintf(req, sizeof(req), "HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Digest realm=\"IGD\", "
			     "nonce=\"%s\"\r\nContent-Length: 0\r\n\r\n", nonce);
		send(fd, req, n, MSG_NOSIGNAL);
	}
	close(fd);
	return ok;
}

static unsigned long interval_of(struct model *m)
{
	struct param *p = model_find(m, "InternetGatewayDevice.ManagementServer.PeriodicInformInterval");
	struct param *e = model_find(m, "InternetGatewayDevice.ManagementServer.PeriodicInformEnable");

	if (e && !strcmp(e->value, "0"))
		return 0;
	return p && atol(p->value) > 0 ? (unsigned long)atol(p->value) : 0;
}

static void usage(void)
{
	fputs("usage: onu-cwmp -u <ACS URL> -U <user> -P <password> -m <model> -n <inform names> -i <ifname>\n"
	      "                [-d <DNS server>] [-a <User-Agent>] [-s <state dir>] [-p <conn req port>] [-1]\n"
	      "                [-x <dir of uci, reload_config, firstboot, reboot, onu-cwmp-firmware: a host test>]\n",
	      stderr);
}

static void read_line(const char *path, char *out, size_t len)
{
	FILE *f = fopen(path, "r");

	out[0] = 0;
	if (!f)
		return;
	if (!fgets(out, len, f))
		out[0] = 0;
	out[strcspn(out, "\n")] = 0;
	fclose(f);
}

int main(int argc, char **argv)
{
	struct opts o = { .agent = "onu-cwmp", .state = "/etc/cwmp", .conn_port = 7547 };
	struct model m = { 0 };
	char path[512], rkey[512], boot_path[512], tpath[512], bmark[512], ip[64] = "", last_ip[64] = "", key[256] = "";
	char boot_url[1024], evm[300], evd[300];
	const char *events[8];
	unsigned int retry = 0;
	time_t next = 0, backoff = 0;
	int opt, lfd = -1, booted;
	FILE *f;

	while ((opt = getopt(argc, argv, "u:U:P:m:n:i:d:a:s:p:x:1")) != -1) {
		switch (opt) {
		case 'u': o.url = optarg; break;
		case 'U': o.user = optarg; break;
		case 'P': o.pass = optarg; break;
		case 'm': o.model = optarg; break;
		case 'n': o.inform_names = optarg; break;
		case 'i': o.ifname = optarg; break;
		case 'd': o.dns = optarg; break;
		case 'a': o.agent = optarg; break;
		case 's': o.state = optarg; break;
		case 'p': o.conn_port = atoi(optarg); break;
		case 'x': sysdir = optarg; break;
		case '1': o.once = 1; break;
		default: usage(); return 2;
		}
	}
	if (!o.url || !o.user || !o.pass || !o.model || !o.inform_names || !o.ifname) {
		usage();
		return 2;
	}
	openlog("onu-cwmp", LOG_PID, LOG_DAEMON);
	signal(SIGPIPE, SIG_IGN);
	if (model_load(&m, o.model)) {
		syslog(LOG_ERR, "no data model in %s", o.model);
		return 1;
	}
	mkdir(o.state, 0700);
	state_path(&o, "sets", path, sizeof(path));
	model_load_sets(&m, path);
	snprintf(bmark, sizeof(bmark), "%s/onu-cwmp.booted", sysdir ? sysdir : "/tmp");	/* RAM: a reboot clears it */
	booted = access(bmark, F_OK) != 0;
	state_path(&o, "reboot.key", rkey, sizeof(rkey));
	state_path(&o, "bootstrap", boot_path, sizeof(boot_path));
	state_path(&o, "transfer", tpath, sizeof(tpath));
	read_line(rkey, key, sizeof(key));
	read_line(boot_path, boot_url, sizeof(boot_url));	/* the ACS this CPE last bootstrapped with */
	for (;;) {
		struct todo t = { 0 };
		struct transfer tr;
		size_t n_events = 0;
		struct pollfd pfd;
		int conn = 0, has_tc;

		if (if_ip(o.ifname, ip, sizeof(ip))) {
			sleep(5);
			continue;
		}
		if (lfd < 0)
			lfd = listener(&o);
		if (!booted && last_ip[0] && strcmp(ip, last_ip) == 0 && time(NULL) < next) {
			pfd.fd = lfd;
			pfd.events = POLLIN;
			if (lfd >= 0 && poll(&pfd, 1, 5000) > 0)
				conn = connection_request(lfd, &m);
			else if (lfd < 0)
				sleep(5);
			if (!conn)
				continue;
		}
		if (backoff && time(NULL) < backoff && !conn) {
			pfd.fd = lfd;
			pfd.events = POLLIN;
			if (lfd < 0 || poll(&pfd, 1, 5000) <= 0 || !(conn = connection_request(lfd, &m)))
				continue;
		}
		has_tc = !transfer_load(&o, &tr);
		if (strcmp(boot_url, o.url))
			events[n_events++] = "0 BOOTSTRAP|";
		if (booted) {
			events[n_events++] = "1 BOOT|";
			if (key[0] || access(rkey, F_OK) == 0) {
				snprintf(evm, sizeof(evm), "M Reboot|%s", key);
				events[n_events++] = evm;
			}
		}
		if (has_tc) {
			events[n_events++] = "7 TRANSFER COMPLETE|";
			snprintf(evd, sizeof(evd), "M Download|%s", tr.key);
			events[n_events++] = evd;
		}
		if (conn)
			events[n_events++] = "6 CONNECTION REQUEST|";
		if (!n_events)
			events[n_events++] = !last_ip[0] || strcmp(ip, last_ip) ? "4 VALUE CHANGE|" : "2 PERIODIC|";
		if (session(&o, &m, events, n_events, retry, &t, has_tc ? &tr : NULL)) {
			retry++;
			backoff = time(NULL) + (retry < 7 ? 5 << retry : 600);
			if (o.once)
				return 1;
			continue;
		}
		retry = 0;
		backoff = 0;
		if (has_tc && unlink(tpath))
			syslog(LOG_ERR, "%s: %s: the TransferComplete will be sent again", tpath, strerror(errno));
		if (strcmp(boot_url, o.url)) {
			f = fopen(boot_path, "w");
			if (!f || fputs(o.url, f) < 0 || fclose(f))
				syslog(LOG_ERR, "%s could not be written: the next session bootstraps again", boot_path);
			snprintf(boot_url, sizeof(boot_url), "%s", o.url);
		}
		if (booted) {
			f = fopen(bmark, "w");
			if (!f || fclose(f))
				syslog(LOG_ERR, "%s could not be written: the next start informs BOOT again", bmark);
			booted = 0;
			unlink(rkey);
			key[0] = 0;
		}
		snprintf(last_ip, sizeof(last_ip), "%s", ip);
		next = interval_of(&m) ? time(NULL) + interval_of(&m) : (time_t)1 << 40;
		if (after_session(&o, &t, rkey, &next))
			return 0;
		if (o.once && !access(tpath, F_OK))
			continue;		/* -1: the TransferComplete's own session first */
		if (o.once)
			return 0;
	}
}
