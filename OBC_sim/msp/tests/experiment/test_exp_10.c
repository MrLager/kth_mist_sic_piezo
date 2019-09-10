/*
 * MSP Experiment Test 10
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
	int pl_content_faulty;
	unsigned long i;

	/* Testing a request where at first the experiment has no data to send, but
	 * then has data to send in the second transaction.
	 * msp_expsend_start()
	 * msp_expsend_complete()
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_complete()
	 * 
	 * i.e., we should not get asked to fill a buffer with 0 bytes of data. */

	/* initialize previous transaction-ID to 0 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD, 0);

	/* Format the first request frame */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (1)");

	/* Get the first exp send frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (2)");
	test_assert(len == 9, "first exp_send header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for first send frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of first exp send frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of the first EXP_SEND should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* Acknowledge the first transaction since there was no data to be sent */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (3)");

	/* Check that the sequence flag has been updated */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 1, "sequence flag updated after first transaction");

	/* Format the second request frame */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (4)");

	/* Get the second exp send frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (5)");
	test_assert(len == 9, "second exp_send header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 100, "value of DL field for second send frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of second send frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of the second EXP_SEND should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (2)");

	/* Acknowledge the send frame */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (6)");

	/* Get the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (7)");
	test_assert(len == 100+5, "data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of the data frame should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 100+1) == msp_from_bigendian32(buf+100+1), "FCS calculation (3)");
	pl_content_faulty = 0;
	for (i = 0; i < 100; i++)
		pl_content_faulty |= (buf[i+1] != 0x29);
	test_assert(!pl_content_faulty, "content of data frame matching what was put in.");

	/* Acknowledge the final transacion */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (8)");

	/* Check that the sequence flag has been updated */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 0, "sequence flag updated after second transaction");

	test_assert(seq == 5, "should have called 5 handlers");
	return;
}


void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	if (seq == 0) {
		test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_start for first transaction");
		*len = 0;
	} else if (seq == 2) {
		test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_start for second transaction");
		*len = 100;
	} else {
		test_assert(0, "msp_expsend_start should be unreachable at this point");
	}

	seq++;
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	test_assert(seq == 3, "msp_expsend_data should only be called as the fourth handler");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_data");
	test_assert(len == 100, "length of the data to send in msp_expsend_data");
	test_assert(offset == 0, "offset should be 0 since no data has previously been sent");
	for (i = 0; i < len; i++)
		buf[i] = 0x29;

	seq++;
}
void msp_expsend_complete(unsigned char opcode)
{
	if (seq == 1) {
		test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_complete for first transaction");
	} else if (seq == 4) {
		test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_complete for second transaction");
	} else {
		test_assert(0, "msp_expsend_complete should be unreachable at this point");
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
