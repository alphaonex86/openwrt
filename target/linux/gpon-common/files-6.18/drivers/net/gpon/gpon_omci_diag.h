/* SPDX-License-Identifier: GPL-2.0-or-later */
/* TIER: CORE, strict host-buildable subset -- G.988 byte math ...
 * dev/MEASURED-gpon_omci_diag.h.md sec 1. */
#ifndef GPON_OMCI_DIAG_H
#define GPON_OMCI_DIAG_H

#include <linux/types.h>

/* Pass as `resp_len` when the shell has no responder output to report here.
 * Distinct from 0, which means "a response was expected and none was built". */
#define GPON_OMCI_NO_RESP_REPORT	(-1)

/* The whole trace line body: what gpon_omci_describe() says ...
 * dev/MEASURED-gpon_omci_diag.h.md sec 2. */
int gpon_omci_diag_line(const u8 *pdu, unsigned int len,
			const u8 *resp, int resp_len, char *out, size_t sz);

#endif /* GPON_OMCI_DIAG_H */
