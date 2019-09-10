/*
 * MSP Experiment Test 14
 * Author: John Wikman
 */

#include "test_exp.h"

static int seq = 0;

#define CUSTOM_OP_REQ_A 0x60
#define CUSTOM_OP_REQ_B 0x64
#define CUSTOM_OP_REQ_C 0x6F

void test(void)
{
	static unsigned char buf[1000];
	unsigned long fcs;
	unsigned long len;
	int code;

	/* Test two transactions each for three custom OBC request opcodes. */

	/* Set previous transaction-ID's to 1 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C, 1);


	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 1, "sequence flag (C)");

	/* Send a REQUEST A header */
	buf[0] = CUSTOM_OP_REQ_A;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the response frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 1, "DL field: send one byte");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame (and transaction-ID)");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send F_ACK */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 6, "");
	test_assert(buf[1] == 0xAA, "value of data field");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 2) == msp_from_bigendian32(buf+2), "FCS calculation");
	/* Send T_ACK */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 0, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 1, "sequence flag (C)");


	/* Send another REQUEST A header */
	buf[0] = CUSTOM_OP_REQ_A;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the response frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 1, "DL field: send one byte");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame (and transaction-ID)");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send F_ACK */
	buf[0] = MSP_OP_F_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 6, "");
	test_assert(buf[1] == 0xAA, "value of data field");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 2) == msp_from_bigendian32(buf+2), "FCS calculation");
	/* Send T_ACK */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 1, "sequence flag (C)");

	/* Send a REQUEST B header */
	buf[0] = CUSTOM_OP_REQ_B;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the response frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 1, "DL field: send one byte");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame (and transaction-ID)");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send F_ACK */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 6, "");
	test_assert(buf[1] == 0xAA, "value of data field");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 2) == msp_from_bigendian32(buf+2), "FCS calculation");
	/* Send T_ACK */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 0, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 1, "sequence flag (C)");

	/* Send a REQUEST C header */
	buf[0] = CUSTOM_OP_REQ_C;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the response frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 1, "DL field: send one byte");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame (and transaction-ID)");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send F_ACK */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 6, "");
	test_assert(buf[1] == 0xAA, "value of data field");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 2) == msp_from_bigendian32(buf+2), "FCS calculation");
	/* Send T_ACK */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 0, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 0, "sequence flag (C)");

	/* Send another REQUEST B header */
	buf[0] = CUSTOM_OP_REQ_B;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the response frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 1, "DL field: send one byte");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame (and transaction-ID)");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send F_ACK */
	buf[0] = MSP_OP_F_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 6, "");
	test_assert(buf[1] == 0xAA, "value of data field");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 2) == msp_from_bigendian32(buf+2), "FCS calculation");
	/* Send T_ACK */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 0, "sequence flag (C)");

	/* Send another REQUEST C header */
	buf[0] = CUSTOM_OP_REQ_C;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the response frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 9, "");
	test_assert(msp_from_bigendian32(buf+1) == 1, "DL field: send one byte");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of frame (and transaction-ID)");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
	/* Send F_ACK */
	buf[0] = MSP_OP_F_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Get the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "");
	test_assert(len == 6, "");
	test_assert(buf[1] == 0xAA, "value of data field");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 2) == msp_from_bigendian32(buf+2), "FCS calculation");
	/* Send T_ACK */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "");
	/* Verify sequence flags for A, B and C. */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "sequence flag (A)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "sequence flag (B)");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 1, "sequence flag (C)");

	test_assert(seq == 18, "18 handlers should have been called");
	return;
}


void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	switch (seq) {
	case 0:
		test_assert(opcode == CUSTOM_OP_REQ_A, "");
		break;
	case 3:
		test_assert(opcode == CUSTOM_OP_REQ_A, "");
		break;
	case 6:
		test_assert(opcode == CUSTOM_OP_REQ_B, "");
		break;
	case 9:
		test_assert(opcode == CUSTOM_OP_REQ_C, "");
		break;
	case 12:
		test_assert(opcode == CUSTOM_OP_REQ_B, "");
		break;
	case 15:
		test_assert(opcode == CUSTOM_OP_REQ_C, "");
		break;
	default:
		test_assert(0, "unexpected call to msp_expsend_start");
	}

	/* Always send one byte for simplicity */
	*len = 1;

	seq++;
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	switch (seq) {
	case 1:
		test_assert(opcode == CUSTOM_OP_REQ_A, "");
		break;
	case 4:
		test_assert(opcode == CUSTOM_OP_REQ_A, "");
		break;
	case 7:
		test_assert(opcode == CUSTOM_OP_REQ_B, "");
		break;
	case 10:
		test_assert(opcode == CUSTOM_OP_REQ_C, "");
		break;
	case 13:
		test_assert(opcode == CUSTOM_OP_REQ_B, "");
		break;
	case 16:
		test_assert(opcode == CUSTOM_OP_REQ_C, "");
		break;
	default:
		test_assert(0, "unexpected call to msp_expsend_data");
	}

	/* We should only send a single data frame with one byte in every transaction */
	test_assert(len == 1, "");
	test_assert(offset == 0, "");
	buf[0] = 0xAA;

	seq++;
}
void msp_expsend_complete(unsigned char opcode)
{
	switch (seq) {
	case 2:
		test_assert(opcode == CUSTOM_OP_REQ_A, "");
		break;
	case 5:
		test_assert(opcode == CUSTOM_OP_REQ_A, "");
		break;
	case 8:
		test_assert(opcode == CUSTOM_OP_REQ_B, "");
		break;
	case 11:
		test_assert(opcode == CUSTOM_OP_REQ_C, "");
		break;
	case 14:
		test_assert(opcode == CUSTOM_OP_REQ_B, "");
		break;
	case 17:
		test_assert(opcode == CUSTOM_OP_REQ_C, "");
		break;
	default:
		test_assert(0, "unexpected call to msp_expsend_complete");
	}

	seq++;
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
	test_assert(0, "msp_exprecv_syscommand should be unreachable");
}
