/*
 * MSP Experiment Test 15
 * Author: John Wikman
 */

#include "test_exp.h"

static int seq = 0;

#define CUSTOM_OP_SEND_A 0x70
#define CUSTOM_OP_SEND_B 0x74
#define CUSTOM_OP_SEND_C 0x7F

void test(void)
{
	static unsigned char buf[1000];
	unsigned long fcs;
	unsigned long len;
	int code;

	/* Test two transactions each for three custom OBC send opcodes. */

	/* Set previous transaction-ID's to 1 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C, 1);


	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "sequence flag (C)");

	/* SEND A header */
	buf[0] = CUSTOM_OP_SEND_A;
	msp_to_bigendian32(buf+1, 1); /* DL = 1 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the F_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send data frame */
	buf[0] = MSP_OP_DATA_FRAME | 0x80;
	buf[1] = 0x99; /* data field */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 2);
	msp_to_bigendian32(buf+2, fcs);
	code = msp_recv_callback(buf, 6);
	test_assert(code == 0, "");
	/* Get the T_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 0, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "sequence flag (C)");

	/* Another SEND A header */
	buf[0] = CUSTOM_OP_SEND_A | 0x80;
	msp_to_bigendian32(buf+1, 1); /* DL = 1 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the F_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send data frame */
	buf[0] = MSP_OP_DATA_FRAME;
	buf[1] = 0x99; /* data field */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 2);
	msp_to_bigendian32(buf+2, fcs);
	code = msp_recv_callback(buf, 6);
	test_assert(code == 0, "");
	/* Get the T_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "sequence flag (C)");

	/* Duplicate SEND A header */
	buf[0] = CUSTOM_OP_SEND_A | 0x80;
	msp_to_bigendian32(buf+1, 1); /* DL = 1 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the T_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "sequence flag (C)");

	/* SEND B header */
	buf[0] = CUSTOM_OP_SEND_B;
	msp_to_bigendian32(buf+1, 1); /* DL = 1 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the F_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send data frame */
	buf[0] = MSP_OP_DATA_FRAME | 0x80;
	buf[1] = 0x99; /* data field */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 2);
	msp_to_bigendian32(buf+2, fcs);
	code = msp_recv_callback(buf, 6);
	test_assert(code == 0, "");
	/* Get the T_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 0, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "sequence flag (C)");

	/* SEND C header */
	buf[0] = CUSTOM_OP_SEND_C;
	msp_to_bigendian32(buf+1, 1); /* DL = 1 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the F_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send data frame */
	buf[0] = MSP_OP_DATA_FRAME | 0x80;
	buf[1] = 0x99; /* data field */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 2);
	msp_to_bigendian32(buf+2, fcs);
	code = msp_recv_callback(buf, 6);
	test_assert(code == 0, "");
	/* Get the T_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 0, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 0, "sequence flag (C)");

	/* Another SEND B header */
	buf[0] = CUSTOM_OP_SEND_B | 0x80;
	msp_to_bigendian32(buf+1, 1); /* DL = 1 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the F_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send data frame */
	buf[0] = MSP_OP_DATA_FRAME;
	buf[1] = 0x99; /* data field */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 2);
	msp_to_bigendian32(buf+2, fcs);
	code = msp_recv_callback(buf, 6);
	test_assert(code == 0, "");
	/* Get the T_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 0, "sequence flag (C)");

	/* Another SEND C header */
	buf[0] = CUSTOM_OP_SEND_C | 0x80;
	msp_to_bigendian32(buf+1, 1); /* DL = 1 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the F_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send data frame */
	buf[0] = MSP_OP_DATA_FRAME;
	buf[1] = 0x99; /* data field */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 2);
	msp_to_bigendian32(buf+2, fcs);
	code = msp_recv_callback(buf, 6);
	test_assert(code == 0, "");
	/* Get the T_ACK */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 0, "DL = 0");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "sequence flag (C)");

	test_assert(seq == 18, "18 handlers should have been called");
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
	switch (seq) {
	case 0:
		test_assert(opcode == CUSTOM_OP_SEND_A, "");
		break;
	case 3:
		test_assert(opcode == CUSTOM_OP_SEND_A, "");
		break;
	case 6:
		test_assert(opcode == CUSTOM_OP_SEND_B, "");
		break;
	case 9:
		test_assert(opcode == CUSTOM_OP_SEND_C, "");
		break;
	case 12:
		test_assert(opcode == CUSTOM_OP_SEND_B, "");
		break;
	case 15:
		test_assert(opcode == CUSTOM_OP_SEND_C, "");
		break;
	default:
		test_assert(0, "unexpected call to msp_exprecv_start");
		break;
	}

	test_assert(len == 1, "");

	seq++;
}
void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	switch (seq) {
	case 1:
		test_assert(opcode == CUSTOM_OP_SEND_A, "");
		break;
	case 4:
		test_assert(opcode == CUSTOM_OP_SEND_A, "");
		break;
	case 7:
		test_assert(opcode == CUSTOM_OP_SEND_B, "");
		break;
	case 10:
		test_assert(opcode == CUSTOM_OP_SEND_C, "");
		break;
	case 13:
		test_assert(opcode == CUSTOM_OP_SEND_B, "");
		break;
	case 16:
		test_assert(opcode == CUSTOM_OP_SEND_C, "");
		break;
	default:
		test_assert(0, "unexpected call to msp_exprecv_data");
		break;
	}

	test_assert(len == 1, "");
	test_assert(offset == 0, "");
	test_assert(buf[0] == 0x99, "");

	seq++;
}
void msp_exprecv_complete(unsigned char opcode)
{
	switch (seq) {
	case 2:
		test_assert(opcode == CUSTOM_OP_SEND_A, "");
		break;
	case 5:
		test_assert(opcode == CUSTOM_OP_SEND_A, "");
		break;
	case 8:
		test_assert(opcode == CUSTOM_OP_SEND_B, "");
		break;
	case 11:
		test_assert(opcode == CUSTOM_OP_SEND_C, "");
		break;
	case 14:
		test_assert(opcode == CUSTOM_OP_SEND_B, "");
		break;
	case 17:
		test_assert(opcode == CUSTOM_OP_SEND_C, "");
		break;
	default:
		test_assert(0, "unexpected call to msp_exprecv_complete");
		break;
	}

	seq++;
}
void msp_exprecv_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_exprecv_error should be unreachable");
}

void msp_exprecv_syscommand(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_syscommand should be unreachable");
}
