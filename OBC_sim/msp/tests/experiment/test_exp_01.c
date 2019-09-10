/*
 * MSP Experiment Test 01
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
	/* Testing request payload 260 bytes (just a bunch of zeroes in this case)
	 * we expect to get the following call sequence:
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_complete()
	 */

	/* Format request frame */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (1)");

	/* Get the exp send frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (2)");
	test_assert(len == 9, "header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 260, "value of DL field");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of exp send frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame ID of EXP_SEND set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* Frame ack */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (3)");

	/* Data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (4)");
	test_assert(len == 265, "data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame ID of DATA set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 261) == msp_from_bigendian32(buf+261), "FCS calculation (2)");

	/* Tracsaction ack */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (5)");
	DEBUG_EXEC(if (code) printf("code: %d\n", code));

	test_assert(seq == 3, "should call 3 handlers");
	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_exprecv_data should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	test_assert(seq == 1, "msp_expsend_data in correct sequence");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode is not MSP_OP_REQ_PAYLOAD");
	test_assert(offset == 0, "offset should be 0 at start of request");
	test_assert(len == 260, "only send 260 bytes of data");

	for (i = 0; i < len; i++)
		buf[i] = 0;

	seq++;
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(seq == 0, "msp_expsend_start in correct sequence");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode is not MSP_OP_REQ_PAYLOAD");
	
	*len = 260;

	seq++;
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(seq == 2, "msp_expsend_complete in correct sequence");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode is not MSP_OP_REQ_PAYLOAD");

	seq++;
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
