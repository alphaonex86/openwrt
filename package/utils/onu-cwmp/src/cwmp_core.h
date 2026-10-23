/* SPDX-License-Identifier: GPL-2.0-or-later */
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
	char reboot_key[256];		/* the CommandKey of the last Reboot asked */
};

/* Placeholders a model value may carry, filled at answer time. */
struct live {
	const char *ip;			/* the CPE's management address */
	const char *uptime;		/* seconds */
	const char *now;		/* ISO 8601, the CurrentTime of an Inform */
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

/* The answer to one ACS request; @reboot set when it asked for one. -> 0, or -1 when not understood. */
int soap_answer(struct buf *b, struct model *m, const struct live *lv, const char *body, const char *id,
		int *reboot, int *changed);

#endif
