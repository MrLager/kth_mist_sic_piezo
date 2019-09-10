/*
 * MSP Experiment Test 13
 * Author: John Wikman
 */

#include "test_exp.h"

static int seq = 0;

#define CUSTOM_OP_SYS_A 0x50
#define CUSTOM_OP_SYS_B 0x57
#define CUSTOM_OP_SYS_C 0x5F

void test(void)
{
	static unsigned char buf[1000];
	unsigned long fcs;
	unsigned long len;
	int code;

	/* Test two transactions each for three custom syscommand opcodes. */

	/* Set previous transaction-ID's to 1 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C, 1);


	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "sequence flag (C)");

	/* Send a SYS COMMAND A header */
	buf[0] = CUSTOM_OP_SYS_A;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get the T_ACK frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "T_ACK frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for T_ACK frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of T_ACK frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of T_ACK frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 0, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "sequence flag (C)");

	/* Send another SYS COMMAND A header */
	buf[0] = CUSTOM_OP_SYS_A | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get the T_ACK frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "T_ACK frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for T_ACK frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of T_ACK frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of T_ACK frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "sequence flag (C)");

	/* Send a duplicate SYS COMMAND A header */
	buf[0] = CUSTOM_OP_SYS_A | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get the T_ACK frame (no handler should be called here) */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "T_ACK frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for T_ACK frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of T_ACK frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of T_ACK frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "sequence flag (C)");

	/* Send a SYS COMMAND B header */
	buf[0] = CUSTOM_OP_SYS_B;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get the T_ACK frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "T_ACK frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for T_ACK frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of T_ACK frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of T_ACK frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 0, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "sequence flag (C)");

	/* Send a SYS COMMAND C header */
	buf[0] = CUSTOM_OP_SYS_C;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get the T_ACK frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "T_ACK frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for T_ACK frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of T_ACK frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of T_ACK frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 0, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 0, "sequence flag (C)");

	/* Send another SYS COMMAND B header */
	buf[0] = CUSTOM_OP_SYS_B | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get the T_ACK frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "T_ACK frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for T_ACK frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of T_ACK frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of T_ACK frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 0, "sequence flag (C)");

	/* Send a SYS COMMAND C header */
	buf[0] = CUSTOM_OP_SYS_C | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get the T_ACK frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "T_ACK frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for T_ACK frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of T_ACK frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of T_ACK frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "sequence flag (C)");

	test_assert(seq == 6, "6 handlers should have been called");
	return;
}


void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(0, "msp_expsend_start should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_expsend_data should be unreachable");
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(0, "msp_expsend_complete should be unreachable");
}
void msp_expsend_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_expsend_error should be unreachable");
}


void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_exprecv_data should be unreachable");
}
void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
}
void msp_exprecv_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_exprecv_error should be unreachable");
}

void msp_exprecv_syscommand(unsigned char opcode)
{
	if (seq < 2) {
		test_assert(opcode == CUSTOM_OP_SYS_A, "");
	} else if (seq == 2) {
		test_assert(opcode == CUSTOM_OP_SYS_B, "");
	} else if (seq == 3) {
		test_assert(opcode == CUSTOM_OP_SYS_C, "");
	} else if (seq == 4) {
		test_assert(opcode == CUSTOM_OP_SYS_B, "");
	} else if (seq == 5) {
		test_assert(opcode == CUSTOM_OP_SYS_C, "");
	} else {
		test_assert(0, "Unexpected call to msp_exprecv_syscommand");
	}

	seq++;
}
