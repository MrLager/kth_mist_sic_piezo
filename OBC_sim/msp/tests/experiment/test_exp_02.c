/*
 * MSP Experiment Test 02
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

	/* Testing sending time (4 bytes)
	 * we expect to get the following call sequence:
	 * msp_exprecv_start()
	 * msp_exprecv_data()
	 * msp_exprecv_complete()
	 */

	/* Format send frame */
	buf[0] = MSP_OP_SEND_TIME;
	msp_to_bigendian32(buf+1, 4);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (1)");

	/* Get acknowledge header */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (2)");
	test_assert(len == 9, "length of ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of ack frame");
	test_assert((buf[0] & 0x80) == 0x00, "frame-ID of ack frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* OBC sends time (frame-ID = 1) */
	buf[0] = MSP_OP_DATA_FRAME | 0x80;
	buf[1] = 0x19;
	buf[2] = 0x93;
	buf[3] = 0x09;
	buf[4] = 0x16;
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (3)");

	/* Get transaction ack */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (4)");
	DEBUG_EXEC(if (code) printf("code: %d\n", code));
	test_assert(len == 9, "length of transaction ack");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of transaction ack");
	test_assert((buf[0] & 0x80) == 0x00, "frame-ID of transaction ack");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (2)");

	test_assert(seq == 3, "should call 3 handlers");
	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(seq == 1, "msp_exprecv_data should be the second called function");
	test_assert(len == 4, "should only receive 4 bytes of data.");
	test_assert(offset == 0, "offset should be set to 0");

	test_assert(buf[0] == 0x19, "value of data byte 0");
	test_assert(buf[1] == 0x93, "value of data byte 1");
	test_assert(buf[2] == 0x09, "value of data byte 2");
	test_assert(buf[3] == 0x16, "value of data byte 3");

	seq++;
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_expsend_data should be unreachable");
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(seq == 0, "msp_exprecv_start should be called first");
	test_assert(opcode == MSP_OP_SEND_TIME, "start has correct opcode");
	test_assert(len == 4, "SEND_TIME should have 4 bytes of data");

	seq++;
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(0, "msp_expsend_start should be unreachable");
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(seq == 2, "msp_exprecv_complete should be called third");
	test_assert(opcode == MSP_OP_SEND_TIME, "opcode in complete");

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
