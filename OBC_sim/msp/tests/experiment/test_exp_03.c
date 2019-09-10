/*
 * MSP Experiment Test 03
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

	/* Testing OBC signalling the experiment to sleep
	 * we expect to get the following call sequence:
	 * msp_exprecv_syscommand()
	 */

	/* Format send frame */
	buf[0] = MSP_OP_SLEEP;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (1)");

	/* Read transaction ack */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (2)");
	DEBUG_EXEC(if (code) printf("code: %d\n", code));
	test_assert(len == 9, "length of transaction ack");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of transaction ack");
	test_assert((buf[0] & 0x80) == 0x00, "frame-ID of transaction ack");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");

	test_assert(seq == 1, "should call one handler");
	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_exprecv_data should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_expsend_data should be unreachable");
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(0, "msp_expsend_start should be unreachable");
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
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
	test_assert(seq == 0, "msp_exprecv_syscommand is called first");
	test_assert(opcode == MSP_OP_SLEEP, "opcode of syscommand");

	seq++;
}
