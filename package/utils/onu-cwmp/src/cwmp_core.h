/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE */
/* The CWMP (TR-069) CPE core: a data model read from a file, the SOAP the CPE sends, the ACS requests it
 * answers. Pure: no socket, no clock, no file but the model's -- host-testable. */
#ifndef CWMP_CORE_H
#define CWMP_CORE_H

#include <stddef.h>

struct buf {
	char *p;
	size_t n, cap;
};

void buf_add(struct buf *b, const char *s);
void buf_addn(struct buf *b, const char *s, size_t n);
void buf_addf(struct buf *b, const char *fmt, ...) __attribute__((format(printf, 2, 3), nonnull(2)));
void buf_add_xml(struct buf *b, const char *s);
void buf_free(struct buf *b);

/* One parameter or object (name ends with '.'), in the CPE's own order. */
struct param {
	char *name, *type, *value;
	int writable, set;		/* set: the ACS gave it its value, saved apart from the model */
};

struct model {
	struct param *p;
	size_t n, cap;
};

/* Placeholders a model value may carry, filled at answer time. A value "{uci:<option>}" lives in UCI: get -> 0
 * with it, set stages a new one (a NULL value reverts what was staged), so the unit itself changes. */
struct live {
	const char *ip;			/* the CPE's management address */
	const char *uptime;		/* seconds */
	const char *now;		/* ISO 8601, the CurrentTime of an Inform */
	int (*uci_get)(const char *option, char *out, size_t len);
	int (*uci_set)(const char *option, const char *value);
};

/* What the ACS asked for that the CPE does once the session is over. */
struct todo {
	int reboot, factory_reset, uci_changed;
	char reboot_key[256];
	char download_key[256], url[1024], user[256], pass[256];	/* a firmware Download: url empty when none */
	long size;
};

int model_load(struct model *m, const char *path);
int model_load_sets(struct model *m, const char *path);
int model_save_sets(const struct model *m, const char *path);
struct param *model_find(struct model *m, const char *name);
const char *model_value(const struct model *m, const struct param *p, const struct live *lv, char *out, size_t len);
void model_free(struct model *m);

/* The ACS request's RPC name and its cwmp:ID. -> 0, or -1 when no RPC is in the body. */
int soap_rpc(const char *body, char *rpc, size_t rpc_len, char *id, size_t id_len);

/* The CPE's Inform: @events is "code|commandkey" per entry, @inform_names the ParameterList. */
void soap_inform(struct buf *b, const char *id, const struct model *m, const struct live *lv,
		 const char *const *events, size_t n_events, const char *const *inform_names, size_t n_names,
		 unsigned int retry);

/* The answer to one ACS request; @t gathers what it asked for after the session. -> 0, or -1 when not understood. */
int soap_answer(struct buf *b, struct model *m, const struct live *lv, const char *body, const char *id,
		struct todo *t, int *changed);

/* The TransferComplete of the Download @key: @fault 0 when it succeeded. */
void soap_transfer_complete(struct buf *b, const char *id, const char *key, int fault, const char *why,
			    const char *start, const char *complete);

#endif
