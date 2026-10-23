// SPDX-License-Identifier: GPL-2.0-or-later
/* The CWMP CPE core -- see cwmp_core.h. The wire format follows the CPE this unit stands in for, line by line:
 * one element per line, an empty value self-closed, Writable as 0/1, an answer carrying its request's ID. */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cwmp_core.h"

#define ENVELOPE "<SOAP-ENV:Envelope xmlns:SOAP-ENV=\"http://schemas.xmlsoap.org/soap/envelope/\" " \
	"xmlns:SOAP-ENC=\"http://schemas.xmlsoap.org/soap/encoding/\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\" " \
	"xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:cwmp=\"urn:dslforum-org:cwmp-1-0\">\n"
#define PARAM_KEY "InternetGatewayDevice.ManagementServer.ParameterKey"

static void buf_grow(struct buf *b, size_t more)
{
	if (b->n + more + 1 <= b->cap)
		return;
	b->cap = (b->n + more + 1) * 2;
	b->p = realloc(b->p, b->cap);
	if (!b->p) {
		perror("cwmp: out of memory");
		exit(1);
	}
}

void buf_addn(struct buf *b, const char *s, size_t n)
{
	buf_grow(b, n);
	memcpy(b->p + b->n, s, n);
	b->n += n;
	b->p[b->n] = 0;
}

void buf_add(struct buf *b, const char *s)
{
	buf_addn(b, s, strlen(s));
}

void buf_addf(struct buf *b, const char *fmt, ...)
{
	va_list ap;
	char probe[1];
	int n;

	va_start(ap, fmt);
	n = vsnprintf(probe, sizeof(probe), fmt, ap);
	va_end(ap);
	buf_grow(b, n);
	va_start(ap, fmt);
	vsnprintf(b->p + b->n, n + 1, fmt, ap);
	va_end(ap);
	b->n += n;
}

void buf_add_xml(struct buf *b, const char *s)
{
	for (; *s; s++) {
		switch (*s) {
		case '&': buf_add(b, "&amp;"); break;
		case '<': buf_add(b, "&lt;"); break;
		case '>': buf_add(b, "&gt;"); break;
		case '"': buf_add(b, "&quot;"); break;
		default: buf_addn(b, s, 1);
		}
	}
}

void buf_free(struct buf *b)
{
	free(b->p);
	memset(b, 0, sizeof(*b));
}

/* A model file field: \t \n \\ escaped. In place. */
static char *unescape(char *s)
{
	char *r = s, *w = s;

	for (; *r; r++, w++) {
		if (*r == '\\' && r[1]) {
			r++;
			*w = *r == 't' ? '\t' : *r == 'n' ? '\n' : *r;
		} else {
			*w = *r;
		}
	}
	*w = 0;
	return s;
}

static void escape_to(FILE *f, const char *s)
{
	for (; *s; s++)
		fputs(*s == '\t' ? "\\t" : *s == '\n' ? "\\n" : *s == '\\' ? "\\\\" : (char[]){ *s, 0 }, f);
}

static struct param *model_add(struct model *m, const char *name, int writable, const char *type, const char *value)
{
	struct param *p;

	if (m->n == m->cap) {
		m->cap = m->cap ? m->cap * 2 : 1024;
		m->p = realloc(m->p, m->cap * sizeof(*m->p));
		if (!m->p) {
			perror("cwmp: out of memory");
			exit(1);
		}
	}
	p = &m->p[m->n++];
	p->name = strdup(name);
	p->type = strdup(type);
	p->value = strdup(value);
	p->writable = writable;
	p->set = 0;
	return p;
}

int model_load(struct model *m, const char *path)
{
	char *line = NULL;
	size_t cap = 0;
	FILE *f = fopen(path, "r");

	if (!f)
		return -1;
	while (getline(&line, &cap, f) > 0) {
		char *f1, *f2, *f3, *f4;

		line[strcspn(line, "\n")] = 0;
		f1 = strtok(line, "\t");
		f2 = strtok(NULL, "\t");
		f3 = strtok(NULL, "\t");
		f4 = strtok(NULL, "");
		if (!f1 || !f2 || *f1 == '#')
			continue;
		model_add(m, unescape(f1), atoi(f2), f3 ? unescape(f3) : "", f4 ? unescape(f4) : "");
	}
	free(line);
	fclose(f);
	return m->n ? 0 : -1;
}

struct param *model_find(struct model *m, const char *name)
{
	size_t i;

	for (i = 0; i < m->n; i++)
		if (!strcmp(m->p[i].name, name))
			return &m->p[i];
	return NULL;
}

static void param_set(struct param *p, const char *value)
{
	free(p->value);
	p->value = strdup(value);
	p->set = 1;
}

/* What the ACS set, kept apart from the model: "name\tvalue" lines, applied after it. */
int model_load_sets(struct model *m, const char *path)
{
	char *line = NULL;
	size_t cap = 0;
	FILE *f = fopen(path, "r");

	if (!f)
		return 0;
	while (getline(&line, &cap, f) > 0) {
		char *tab = strchr(line, '\t');
		struct param *p;

		line[strcspn(line, "\n")] = 0;
		if (!tab)
			continue;
		*tab = 0;
		p = model_find(m, unescape(line));
		if (p)
			param_set(p, unescape(tab + 1));
	}
	free(line);
	fclose(f);
	return 0;
}

int model_save_sets(const struct model *m, const char *path)
{
	char tmp[512];
	FILE *f;
	size_t i;

	snprintf(tmp, sizeof(tmp), "%s.tmp", path);
	f = fopen(tmp, "w");
	if (!f)
		return -1;
	for (i = 0; i < m->n; i++) {
		if (!m->p[i].set)
			continue;
		escape_to(f, m->p[i].name);
		fputc('\t', f);
		escape_to(f, m->p[i].value);
		fputc('\n', f);
	}
	if (fclose(f) || rename(tmp, path))
		return -1;
	return 0;
}

const char *model_value(const struct model *m, const struct param *p, const struct live *lv, char *out, size_t len)
{
	const char *s = p->value;
	size_t n = 0;

	(void)m;
	while (*s && n + 1 < len) {
		const char *rep = NULL;
		size_t skip = 0;

		if (!strncmp(s, "{ip}", 4))
			rep = lv->ip, skip = 4;
		else if (!strncmp(s, "{uptime}", 8))
			rep = lv->uptime, skip = 8;
		else if (!strncmp(s, "{now}", 5))
			rep = lv->now, skip = 5;
		if (skip) {
			n += snprintf(out + n, len - n, "%s", rep ? rep : "");
			s += skip;
		} else {
			out[n++] = *s++;
		}
	}
	out[n < len ? n : len - 1] = 0;
	return out;
}

void model_free(struct model *m)
{
	size_t i;

	for (i = 0; i < m->n; i++) {
		free(m->p[i].name);
		free(m->p[i].type);
		free(m->p[i].value);
	}
	free(m->p);
	memset(m, 0, sizeof(*m));
}

/* The next <tag ...>content</tag> (any namespace prefix) after @s: a malloc'd, unescaped copy; @s moves past it. */
static char *xml_take(const char **s, const char *tag)
{
	size_t tl = strlen(tag);
	const char *p = *s;

	while ((p = strchr(p, '<'))) {
		const char *name = p + 1, *colon, *gt, *end;
		char *out, *w;

		if (*name == '/' || *name == '?' || *name == '!') {
			p++;
			continue;
		}
		gt = strchr(name, '>');
		if (!gt)
			return NULL;
		colon = memchr(name, ':', gt - name);
		if (colon && colon < name + strcspn(name, " />"))
			name = colon + 1;
		if (strncmp(name, tag, tl) || !strchr(" />", name[tl])) {
			p = gt;
			continue;
		}
		if (gt[-1] == '/') {			/* <tag/> */
			*s = gt + 1;
			return strdup("");
		}
		end = strstr(gt + 1, "</");
		while (end) {
			const char *en = end + 2, *ec = strchr(en, '>'), *c2 = ec ? memchr(en, ':', ec - en) : NULL;

			if (c2)
				en = c2 + 1;
			if (ec && !strncmp(en, tag, tl) && en + tl == ec)
				break;
			end = strstr(end + 2, "</");
		}
		if (!end)
			return NULL;
		out = w = malloc(end - gt);
		for (p = gt + 1; p < end; p++) {
			static const struct { const char *e; char c; } ents[] = {
				{ "&amp;", '&' }, { "&lt;", '<' }, { "&gt;", '>' }, { "&quot;", '"' }, { "&apos;", '\'' } };
			size_t k;

			for (k = 0; k < sizeof(ents) / sizeof(ents[0]); k++)
				if (!strncmp(p, ents[k].e, strlen(ents[k].e)))
					break;
			if (k < sizeof(ents) / sizeof(ents[0])) {
				*w++ = ents[k].c;
				p += strlen(ents[k].e) - 1;
			} else {
				*w++ = *p;
			}
		}
		*w = 0;
		*s = strchr(end, '>') + 1;
		return out;
	}
	return NULL;
}

int soap_rpc(const char *body, char *rpc, size_t rpc_len, char *id, size_t id_len)
{
	const char *s = body, *b;
	char *v;
	size_t n;

	v = xml_take(&s, "ID");
	snprintf(id, id_len, "%s", v ? v : "");
	free(v);
	b = strstr(body, "Body>");
	if (!b)
		return -1;
	b = strchr(b, '<');
	while (b && (b[1] == '/' || b[1] == '!'))
		b = strchr(b + 1, '<');
	if (!b)
		return -1;
	b++;
	if (memchr(b, ':', strcspn(b, " />")))
		b = strchr(b, ':') + 1;
	n = strcspn(b, " />");
	if (!n || n >= rpc_len)
		return -1;
	memcpy(rpc, b, n);
	rpc[n] = 0;
	return 0;
}

static void envelope_open(struct buf *b, const char *id)
{
	buf_add(b, ENVELOPE "<SOAP-ENV:Header>\n<cwmp:ID SOAP-ENV:mustUnderstand=\"1\">");
	buf_add_xml(b, id);
	buf_add(b, "</cwmp:ID>\n</SOAP-ENV:Header>\n<SOAP-ENV:Body>\n");
}

static void envelope_close(struct buf *b)
{
	buf_add(b, "</SOAP-ENV:Body>\n</SOAP-ENV:Envelope>\n");
}

static void value_struct(struct buf *b, const struct model *m, const struct param *p, const struct live *lv)
{
	size_t len = strlen(p->value) + 256;
	char *v = malloc(len);

	model_value(m, p, lv, v, len);
	buf_add(b, "<ParameterValueStruct>\n<Name>");
	buf_add_xml(b, p->name);
	buf_addf(b, "</Name>\n<Value xsi:type=\"%s\"", p->type[0] ? p->type : "xsd:string");
	if (v[0]) {
		buf_add(b, ">");
		buf_add_xml(b, v);
		buf_add(b, "</Value>\n");
	} else {
		buf_add(b, "/>\n");
	}
	buf_add(b, "</ParameterValueStruct>\n");
	free(v);
}

void soap_inform(struct buf *b, const char *id, const struct model *m, const struct live *lv,
		 const char *const *events, size_t n_events, const char *const *inform_names, size_t n_names,
		 unsigned int retry)
{
	static const char *const dev[][2] = {
		{ "Manufacturer", "InternetGatewayDevice.DeviceInfo.Manufacturer" },
		{ "OUI", "InternetGatewayDevice.DeviceInfo.ManufacturerOUI" },
		{ "ProductClass", "InternetGatewayDevice.DeviceInfo.ProductClass" },
		{ "SerialNumber", "InternetGatewayDevice.DeviceInfo.SerialNumber" } };
	size_t i, shown = 0;
	char v[1024];

	envelope_open(b, id);
	buf_add(b, "<cwmp:Inform>\n<DeviceId>\n");
	for (i = 0; i < 4; i++) {
		struct param *p = model_find((struct model *)m, dev[i][1]);

		buf_addf(b, "<%s>", dev[i][0]);
		if (p)
			buf_add_xml(b, model_value(m, p, lv, v, sizeof(v)));
		buf_addf(b, "</%s>\n", dev[i][0]);
	}
	buf_addf(b, "</DeviceId>\n<Event SOAP-ENC:arrayType=\"cwmp:EventStruct[%zu]\">\n", n_events);
	for (i = 0; i < n_events; i++) {
		const char *bar = strchr(events[i], '|');

		buf_add(b, "<EventStruct>\n<EventCode>");
		buf_addn(b, events[i], bar ? (size_t)(bar - events[i]) : strlen(events[i]));
		buf_add(b, "</EventCode>\n");
		if (bar && bar[1]) {
			buf_add(b, "<CommandKey>");
			buf_add_xml(b, bar + 1);
			buf_add(b, "</CommandKey>\n");
		} else {
			buf_add(b, "<CommandKey/>\n");
		}
		buf_add(b, "</EventStruct>\n");
	}
	for (i = 0; i < n_names; i++)
		shown += model_find((struct model *)m, inform_names[i]) != NULL;
	buf_addf(b, "</Event>\n<MaxEnvelopes>1</MaxEnvelopes>\n<CurrentTime>%s</CurrentTime>\n<RetryCount>%u</RetryCount>\n"
		 "<ParameterList SOAP-ENC:arrayType=\"cwmp:ParameterValueStruct[%zu]\">\n", lv->now, retry, shown);
	for (i = 0; i < n_names; i++) {
		struct param *p = model_find((struct model *)m, inform_names[i]);

		if (p)
			value_struct(b, m, p, lv);
	}
	buf_add(b, "</ParameterList>\n</cwmp:Inform>\n");
	envelope_close(b);
}

static void fault(struct buf *b, const char *id, int code, const char *why, const struct buf *spv_faults)
{
	envelope_open(b, id);
	buf_addf(b, "<SOAP-ENV:Fault>\n<faultcode>Client</faultcode>\n<faultstring>CWMP fault</faultstring>\n<detail>\n"
		 "<cwmp:Fault>\n<FaultCode>%d</FaultCode>\n<FaultString>%s</FaultString>\n", code, why);
	if (spv_faults && spv_faults->n)
		buf_add(b, spv_faults->p);
	buf_add(b, "</cwmp:Fault>\n</detail>\n</SOAP-ENV:Fault>\n");
	envelope_close(b);
}

static int is_object(const struct param *p)
{
	size_t n = strlen(p->name);

	return n && p->name[n - 1] == '.';
}

/* Is @p a child of @path, one level down when @next_level? */
static int under(const struct param *p, const char *path, int next_level)
{
	size_t pl = strlen(path);
	const char *rest;

	if (strncmp(p->name, path, pl) || !p->name[pl])
		return 0;
	if (!next_level)
		return 1;
	rest = p->name + pl;
	return !strchr(rest, '.') || strchr(rest, '.')[1] == 0;
}

static int gpn(struct buf *b, struct model *m, const char *body, const char *id)
{
	const char *s = body;
	char *path = xml_take(&s, "ParameterPath"), *nl;
	int next_level, any = 0;
	size_t i, n = 0;
	struct buf list = { 0 };

	s = body;
	nl = xml_take(&s, "NextLevel");
	next_level = nl && (!strcmp(nl, "true") || !strcmp(nl, "1"));
	free(nl);
	if (!path)
		path = strdup("");
	for (i = 0; i < m->n; i++) {
		const struct param *p = &m->p[i];

		if (path[0] && path[strlen(path) - 1] != '.') {
			if (strcmp(p->name, path))
				continue;
		} else if (path[0] && !under(p, path, next_level)) {
			continue;
		} else if (!path[0] && next_level && strchr(p->name, '.') != p->name + strlen(p->name) - 1) {
			continue;
		}
		any = 1;
		n++;
		buf_add(&list, "<ParameterInfoStruct>\n<Name>");
		buf_add_xml(&list, p->name);
		buf_addf(&list, "</Name>\n<Writable>%d</Writable>\n</ParameterInfoStruct>\n", p->writable);
	}
	if (!any && path[0] && !model_find(m, path)) {
		fault(b, id, 9005, "Invalid parameter name", NULL);
	} else {
		envelope_open(b, id);
		buf_addf(b, "<cwmp:GetParameterNamesResponse>\n<ParameterList SOAP-ENC:arrayType=\"cwmp:ParameterInfoStruct[%zu]\">\n", n);
		if (list.n)
			buf_add(b, list.p);
		buf_add(b, "</ParameterList>\n</cwmp:GetParameterNamesResponse>\n");
		envelope_close(b);
	}
	buf_free(&list);
	free(path);
	return 0;
}

static int gpv(struct buf *b, struct model *m, const struct live *lv, const char *body, const char *id)
{
	const char *s = body;
	char *name;
	struct buf list = { 0 };
	size_t n = 0, i;

	while ((name = xml_take(&s, "string"))) {
		int any = 0;

		for (i = 0; i < m->n; i++) {
			const struct param *p = &m->p[i];
			size_t nl = strlen(name);

			if (is_object(p) || (nl && name[nl - 1] == '.' ? strncmp(p->name, name, nl) : strcmp(p->name, name)))
				continue;
			value_struct(&list, m, p, lv);
			n++;
			any = 1;
		}
		if (!any) {
			fault(b, id, 9005, "Invalid parameter name", NULL);
			free(name);
			buf_free(&list);
			return 0;
		}
		free(name);
	}
	envelope_open(b, id);
	buf_addf(b, "<cwmp:GetParameterValuesResponse>\n<ParameterList SOAP-ENC:arrayType=\"cwmp:ParameterValueStruct[%zu]\">\n", n);
	if (list.n)
		buf_add(b, list.p);
	buf_add(b, "</ParameterList>\n</cwmp:GetParameterValuesResponse>\n");
	envelope_close(b);
	buf_free(&list);
	return 0;
}

static void set_param_key(struct model *m, const char *body)
{
	const char *s = body;
	char *key = xml_take(&s, "ParameterKey");
	struct param *p = model_find(m, PARAM_KEY);

	if (key && p)
		param_set(p, key);
	free(key);
}

static int spv(struct buf *b, struct model *m, const char *body, const char *id, int *changed)
{
	const char *s = body;
	struct buf bad = { 0 };
	char *names[256], *values[256];
	size_t n = 0, i;

	while (n < 256) {
		char *name = xml_take(&s, "Name"), *value;

		if (!name)
			break;
		value = xml_take(&s, "Value");
		names[n] = name;
		values[n++] = value ? value : strdup("");
	}
	for (i = 0; i < n; i++) {
		struct param *p = model_find(m, names[i]);
		int code = !p || is_object(p) ? 9005 : !p->writable ? 9008 : 0;

		if (code) {
			buf_add(&bad, "<SetParameterValuesFault>\n<ParameterName>");
			buf_add_xml(&bad, names[i]);
			buf_addf(&bad, "</ParameterName>\n<FaultCode>%d</FaultCode>\n<FaultString>%s</FaultString>\n"
				 "</SetParameterValuesFault>\n", code, code == 9005 ? "Invalid parameter name" :
				 "Attempt to set a non-writable parameter");
		}
	}
	if (bad.n) {
		fault(b, id, 9003, "Invalid arguments", &bad);
	} else {
		for (i = 0; i < n; i++)
			param_set(model_find(m, names[i]), values[i]);
		set_param_key(m, body);
		*changed = 1;
		envelope_open(b, id);
		buf_add(b, "<cwmp:SetParameterValuesResponse>\n<Status>0</Status>\n</cwmp:SetParameterValuesResponse>\n");
		envelope_close(b);
	}
	for (i = 0; i < n; i++) {
		free(names[i]);
		free(values[i]);
	}
	buf_free(&bad);
	return 0;
}

static int gpa(struct buf *b, struct model *m, const char *body, const char *id)
{
	const char *s = body;
	struct buf list = { 0 };
	size_t n = 0, i;
	char *name;

	while ((name = xml_take(&s, "string"))) {
		size_t nl = strlen(name);

		for (i = 0; i < m->n; i++) {
			const struct param *p = &m->p[i];

			if (is_object(p) || (nl && name[nl - 1] == '.' ? strncmp(p->name, name, nl) : strcmp(p->name, name)))
				continue;
			buf_add(&list, "<ParameterAttributeStruct>\n<Name>");
			buf_add_xml(&list, p->name);
			buf_add(&list, "</Name>\n<Notification>0</Notification>\n"
				"<AccessList SOAP-ENC:arrayType=\"xsd:string[0]\">\n</AccessList>\n</ParameterAttributeStruct>\n");
			n++;
		}
		free(name);
	}
	envelope_open(b, id);
	buf_addf(b, "<cwmp:GetParameterAttributesResponse>\n<ParameterList SOAP-ENC:arrayType=\"cwmp:ParameterAttributeStruct[%zu]\">\n", n);
	if (list.n)
		buf_add(b, list.p);
	buf_add(b, "</ParameterList>\n</cwmp:GetParameterAttributesResponse>\n");
	envelope_close(b);
	buf_free(&list);
	return 0;
}

/* AddObject: the new instance copies the first existing instance's subtree, values and all. */
static int add_object(struct buf *b, struct model *m, const char *body, const char *id, int *changed)
{
	const char *s = body;
	char *obj = xml_take(&s, "ObjectName"), first[512] = "", inst[512];
	size_t ol, i, base_n;
	long max = 0;

	if (!obj || !(ol = strlen(obj)) || obj[ol - 1] != '.' || !model_find(m, obj)) {
		fault(b, id, 9005, "Invalid parameter name", NULL);
		free(obj);
		return 0;
	}
	for (i = 0; i < m->n; i++) {
		char *end;
		long k;

		if (strncmp(m->p[i].name, obj, ol))
			continue;
		k = strtol(m->p[i].name + ol, &end, 10);
		if (end != m->p[i].name + ol && *end == '.' && k > 0) {
			if (!first[0])
				snprintf(first, sizeof(first), "%.*s", (int)(end + 1 - m->p[i].name), m->p[i].name);
			if (k > max)
				max = k;
		}
	}
	snprintf(inst, sizeof(inst), "%s%ld.", obj, max + 1);
	base_n = m->n;
	model_add(m, inst, 1, "", "");
	for (i = 0; first[0] && i < base_n; i++) {
		size_t fl = strlen(first);
		char nm[1024];

		if (strncmp(m->p[i].name, first, fl) || !m->p[i].name[fl])
			continue;
		snprintf(nm, sizeof(nm), "%s%s", inst, m->p[i].name + fl);
		model_add(m, nm, m->p[i].writable, m->p[i].type, m->p[i].value);
	}
	set_param_key(m, body);
	*changed = 1;
	envelope_open(b, id);
	buf_addf(b, "<cwmp:AddObjectResponse>\n<InstanceNumber>%ld</InstanceNumber>\n<Status>0</Status>\n"
		 "</cwmp:AddObjectResponse>\n", max + 1);
	envelope_close(b);
	free(obj);
	return 0;
}

static int delete_object(struct buf *b, struct model *m, const char *body, const char *id, int *changed)
{
	const char *s = body;
	char *obj = xml_take(&s, "ObjectName");
	size_t ol, i, w = 0;

	if (!obj || !(ol = strlen(obj)) || obj[ol - 1] != '.' || !model_find(m, obj)) {
		fault(b, id, 9005, "Invalid parameter name", NULL);
		free(obj);
		return 0;
	}
	for (i = 0; i < m->n; i++) {
		if (!strncmp(m->p[i].name, obj, ol)) {
			free(m->p[i].name);
			free(m->p[i].type);
			free(m->p[i].value);
			continue;
		}
		m->p[w++] = m->p[i];
	}
	m->n = w;
	set_param_key(m, body);
	*changed = 1;
	envelope_open(b, id);
	buf_add(b, "<cwmp:DeleteObjectResponse>\n<Status>0</Status>\n</cwmp:DeleteObjectResponse>\n");
	envelope_close(b);
	free(obj);
	return 0;
}

int soap_answer(struct buf *b, struct model *m, const struct live *lv, const char *body, const char *id,
		int *reboot, int *changed)
{
	static const char *const methods[] = { "GetRPCMethods", "SetParameterValues", "GetParameterValues",
		"GetParameterNames", "SetParameterAttributes", "GetParameterAttributes", "AddObject", "DeleteObject",
		"Reboot", "Download", "Upload", "FactoryReset" };
	char rpc[64], rid[64];
	size_t i;

	if (soap_rpc(body, rpc, sizeof(rpc), rid, sizeof(rid)))
		return -1;
	if (!id)
		id = rid;
	if (!strcmp(rpc, "GetParameterNames"))
		return gpn(b, m, body, id);
	if (!strcmp(rpc, "GetParameterValues"))
		return gpv(b, m, lv, body, id);
	if (!strcmp(rpc, "SetParameterValues"))
		return spv(b, m, body, id, changed);
	if (!strcmp(rpc, "GetParameterAttributes"))
		return gpa(b, m, body, id);
	if (!strcmp(rpc, "AddObject"))
		return add_object(b, m, body, id, changed);
	if (!strcmp(rpc, "DeleteObject"))
		return delete_object(b, m, body, id, changed);
	envelope_open(b, id);
	if (!strcmp(rpc, "SetParameterAttributes")) {
		buf_add(b, "<cwmp:SetParameterAttributesResponse/>\n");
	} else if (!strcmp(rpc, "GetRPCMethods")) {
		buf_addf(b, "<cwmp:GetRPCMethodsResponse>\n<MethodList SOAP-ENC:arrayType=\"xsd:string[%zu]\">\n",
			 sizeof(methods) / sizeof(methods[0]));
		for (i = 0; i < sizeof(methods) / sizeof(methods[0]); i++)
			buf_addf(b, "<string>%s</string>\n", methods[i]);
		buf_add(b, "</MethodList>\n</cwmp:GetRPCMethodsResponse>\n");
	} else if (!strcmp(rpc, "Reboot")) {
		const char *s = body;
		char *key = xml_take(&s, "CommandKey");

		snprintf(m->reboot_key, sizeof(m->reboot_key), "%s", key ? key : "");
		free(key);
		buf_add(b, "<cwmp:RebootResponse/>\n");
		*reboot = 1;
	} else {
		b->n = 0;
		fault(b, id, !strcmp(rpc, "Download") || !strcmp(rpc, "Upload") || !strcmp(rpc, "FactoryReset") ?
		      9001 : 9000, "Request denied", NULL);
		return 0;
	}
	envelope_close(b);
	return 0;
}
