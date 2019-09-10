/*
 * obcsim_transactions.h
 *
 * Defines the functions necessary handle different sequences.
 */

#ifndef OBCSIM_TRANSACTIONS_H
#define OBCSIM_TRANSACTIONS_H

#include "msp_obc.h"

enum PrintStyle {
	NONE,
	BYTES,
	BITS,
	STRING
};

void invoke_syscommand(msp_link_t *lnk, unsigned char opcode);
void invoke_send(msp_link_t *lnk, unsigned char opcode, unsigned char *data, unsigned long len, PrintStyle pstyle);
void invoke_send_repeat(msp_link_t *lnk, unsigned char opcode, unsigned char value, unsigned long times, PrintStyle pstyle);
void invoke_request(msp_link_t *lnk, unsigned char opcode, PrintStyle pstyle);

#endif /* OBCSIM_TRANSACTIONS_H */
