/*
 * MSP Experiment Test 07
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
	int hk_content_faulty;
	int pl_content_faulty;
	int i;

	/* Testing to first send a housekeeping transaction and then a payload
	 * transaction. We expect the following sequence of function calls:
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_complete()
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_complete()
	 */

	/* Set 0 to the previous transaction-ID for both HK and PAYLOAD */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD, 0);
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_REQ_HK, 0);

	/* Format the first request frame */
	buf[0] = MSP_OP_REQ_HK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (1)");

	/* Get the first exp send frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (2)");
	test_assert(len == 9, "header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 13, "value of DL field for first send frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of exp send frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame ID of EXP_SEND should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* Acknowledge the send frame */
	buf[0] = MSP_OP_F_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (3)");

	/* Get the first data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (4)");
	test_assert(len == (13+5), "second data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of the first data frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame ID of the first data frame should set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 13+1) == msp_from_bigendian32(buf+13+1), "FCS calculation (2)");
	hk_content_faulty = 0;
	for (i = 0; i < 13; i++)
		hk_content_faulty |= (buf[i+1] != 0x13);
	test_assert(!hk_content_faulty, "Receive correct housekeeping data");

	/* Acknowledge the transaction */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (5)");

	/* Check that the sequence flag is set to 1 */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_HK) == 1, "sequence flag updated after housekeeping request");

	/*** REQUEST PAYLOAD ***/

	/* Format the second request frame */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (6)");

	/* Get the second exp send frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (7)");
	test_assert(len == 9, "second header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 100, "value of DL field for second send frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of second exp send frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame ID of the second EXP_SEND should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (3)");

	/* Acknowledge the second send frame */
	buf[0] = MSP_OP_F_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (8)");

	/* Get the data frame of the second transaction */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (9)");
	test_assert(len == (100+5), "second data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of the second data frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame ID of the second data frame should set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 100+1) == msp_from_bigendian32(buf+100+1), "FCS calculation (4)");
	pl_content_faulty = 0;
	for (i = 0; i < 100; i++)
		pl_content_faulty |= (buf[i+1] != 0x42);
	test_assert(!pl_content_faulty, "Receive correct payload data");


	/* Acknowledge the second transaction */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (10)");

	/* Check that the sequence flag is set to 1 */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 1, "sequence flag updated after payload request");


	test_assert(seq == 6, "should have called 6 handlers");
	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_exprecv_data should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	if (seq == 1) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode of handler at seq = 1");
		test_assert(len == 13, "length of housekeeping to be sent");
		test_assert(offset == 0, "offset should be 0 for housekeeping");
		for (i = 0; i < len; i++)
			buf[i] = 0x13;
	} else if (seq == 4) {
		test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode of handler at seq = 4");
		test_assert(len == 100, "length of payload to be sent");
		test_assert(offset == 0, "offset should be 0 for payload");
		for (i = 0; i < len; i++)
			buf[i] = 0x42;
	} else {
		test_assert(0, "msp_expsend_data should be unreachable at this point in time");
	}

	seq++;
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	if (seq == 0) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode in msp_expsend_start at seq = 0");
		*len = 13;
	} else if (seq == 3) {
		test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_start at seq = 3");
		*len = 100;
	} else {
		test_assert(0, "msp_expsend_start should be unreachable at this point in time");
	}

	seq++;
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
}
void msp_expsend_complete(unsigned char opcode)
{
	if (seq == 2) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode in msp_expsend_complete at seq = 2");
	} else if (seq == 5) {
		test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_complete at seq = 5");
	} else {
		test_assert(0, "msp_expsend_complete should be unreachable at this point in time");
	}

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
