/*
 * MSP Experiment Test 06
 * Author: John Wikman
 */

#include "test_exp.h"

static int seq = 0;

void test(void)
{
	static unsigned char buf[1000];
	unsigned long fcs;
	unsigned long len;
	int code;

	/* Testing to receive two transactions in a row with same opcode
	 * we expect to get the following call sequence:
	 * msp_exprecv_start()
	 * msp_exprecv_data()
	 * msp_exprecv_complete()
	 * msp_exprecv_start()
	 * msp_exprecv_data()
	 * msp_exprecv_complete()
	 */

	/* Set 0 to the previous transaction-ID */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_SEND_TIME, 0);

	/* Format send frame */
	buf[0] = MSP_OP_SEND_TIME | 0x80;
	msp_to_bigendian32(buf+1, 4);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (1)");

	/* Acknowledge header */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (2)");
	test_assert(len == 9, "length of ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of ack frame (1)");
	test_assert((buf[0] & 0x80) == 0x80, "frame-ID of ack frame should be 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* OBC sends time (frame-ID = 0) */
	buf[0] = MSP_OP_DATA_FRAME;
	buf[1] = 0x19;
	buf[2] = 0x93;
	buf[3] = 0x09;
	buf[4] = 0x16;
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (3)");

	/* Transaction acknowledge header */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (4)");
	test_assert(len == 9, "length of transaction ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of first transaction ack frame");
	test_assert((buf[0] & 0x80) == 0x80, "frame-ID of first T_ACK frame should be 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (2)");


	/*** NEXT TRANSACTION ***/

	/* Format send frame */
	buf[0] = MSP_OP_SEND_TIME;
	msp_to_bigendian32(buf+1, 4);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (5)");

	/* Acknowledge header */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (6)");
	test_assert(len == 9, "length of ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of ack frame (2)");
	test_assert((buf[0] & 0x80) == 0x00, "frame-ID of ack frame should be 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (3)");

	/* OBC sends time (frame-ID = 1) */
	buf[0] = MSP_OP_DATA_FRAME | 0x80;
	buf[1] = 0x20;
	buf[2] = 0x18;
	buf[3] = 0x09;
	buf[4] = 0x24;
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (7)");
	DEBUG_EXEC(if (code) printf("code (7): %d\n", code));

	/* Final transaction acknowledge */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (8)");
	test_assert(len == 9, "length of transaction ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of second transaction ack frame");
	test_assert((buf[0] & 0x80) == 0x00, "frame-ID of second T_ACK frame should be 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (4)");

	test_assert(seq == 6, "should have called 6 handlers");
	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(seq == 1 || seq == 4, "msp_exprecv_data should be called 2nd or 5th");
	test_assert(opcode == MSP_OP_SEND_TIME, "opcode should be SEND_TIME");
	test_assert(len == 4, "length should be 4 (2)");
	test_assert(offset == 0, "offset should be 0");

	if (seq == 1) {
		test_assert(buf[0] == 0x19, "value of byte 0 (1)");
		test_assert(buf[1] == 0x93, "value of byte 1 (1)");
		test_assert(buf[2] == 0x09, "value of byte 2 (1)");
		test_assert(buf[3] == 0x16, "value of byte 3 (1)");
	} else if (seq == 4) {
		test_assert(buf[0] == 0x20, "value of byte 0 (2)");
		test_assert(buf[1] == 0x18, "value of byte 1 (2)");
		test_assert(buf[2] == 0x09, "value of byte 2 (2)");
		test_assert(buf[3] == 0x24, "value of byte 3 (2)");
	}

	seq++;
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_expsend_data should be unreachable");
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(seq == 0 || seq == 3, "msp_exprecv_start should be called 1st or 4th");
	test_assert(opcode == MSP_OP_SEND_TIME, "opcode should be SEND_TIME");
	test_assert(len == 4, "length should be 4 (1)");

	seq++;
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(0, "msp_expsend_start should be unreachable");
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(seq == 2 || seq == 5, "msp_exprecv_complete should be called 3rd or 6th");
	test_assert(opcode == MSP_OP_SEND_TIME, "opcode in msp_exprecv_complete");

	seq++;
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(0, "msp_expsend_complete should be unreachable");
}

void msp_exprecv_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_exprecv_error should be unreachable");
}
void msp_expsend_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_expsend_error should be unreachable");
}

void msp_exprecv_syscommand(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_syscommand should be unreachable");
}
