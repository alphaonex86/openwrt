/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef CWMP_MD5_H
#define CWMP_MD5_H

/* The MD5 of a C string, as 32 lowercase hex digits. */
void md5_hex(const char *s, char out[33]);

#endif
