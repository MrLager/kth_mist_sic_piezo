/*
 * MSP Experiment Test 12
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

	/* Test receiving a transaction with one byte of data and then receiving a
	 * transaction with one byte of data. Both will have transaction-ID 0.
	 * Expected call sequence:
	 *  - msp_exprecv_start()
	 *  - msp_exprecv_data()
	 *  - msp_exprecv_complete()
	 *  - msp_expsend_start()
	 *  - msp_expsend_data()
	 *  - msp_expsend_complete()
	 */

	/* Set previous transaction-ID's to 1 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_SEND_PUS, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD, 1);

	/* Setup OBC Send header */
	buf[0] = MSP_OP_SEND_PUS;
	msp_to_bigendian32(buf+1, 1); /* DL = 1 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (1)");

	/* Get the ack frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (2)");
	test_assert(len == 9, "ack frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of ack frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of ack frame should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* OBC Send data */
	buf[0] = MSP_OP_DATA_FRAME | 0x80;
	buf[1] = 0x11;
	fcs = msp_exp_frame_generate_fcs(buf, 1, 2);
	msp_to_bigendian32(buf+2, fcs);
	code = msp_recv_callback(buf, 6);
	test_assert(code == 0, "Unexpected error (3)");

	/* Get the transaction acknowledgement frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (4)");
	test_assert(len == 9, "transaction ack frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for transaction ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of transaction ack frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of transaction ack frame should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (2)");

	/* Transaction-ID should be updated here */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_SEND_PUS) == 0, "update seqflag for SEND_PUS");


	/* Setup OBC Request header */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (5)");

	/* Get the EXP_SEND frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (6)");
	test_assert(len == 9, "response frame length");
	test_assert(msp_from_bigendian32(buf+1) == 1, "value of DL field for response frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of response frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of response frame should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (3)");

	/* Acknowledge the response */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (7)");

	/* Receive DATA frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (8)");
	test_assert(len == 6, "A data frame with 1 byte of data should be 6 bytes");
	test_assert(buf[1] == 0x22, "received payload in data frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame from experiment");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of the data frame should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 2) == msp_from_bigendian32(buf+2), "FCS calculation (4)");

	/* Acknowledge the transaction */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (9)");

	/* Transaction-ID should be updated here */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 0, "update seqflag for REQ_PAYLOAD");


	test_assert(seq == 6, "6 handlers should have been called");
	return;
}


void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(seq == 3, "seq in msp_expsend_start");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_start");
	*len = 1;

	seq++;
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(seq == 4, "seq in msp_expsend_data");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_data");
	test_assert(len == 1, "len in msp_expsend_data");
	test_assert(offset == 0, "offset in msp_expsend_data");

	buf[0] = 0x22;

	seq++;
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(seq == 5, "seq in msp_expsend_complete");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_complete");

	seq++;
}
void msp_expsend_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_expsend_error should be unreachable");
}


void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(seq == 0, "seq in msp_exprecv_start");
	test_assert(opcode == MSP_OP_SEND_PUS, "opcode in msp_exprecv_start");
	test_assert(len == 1, "len in msp_exprecv_start");

	seq++;
}
void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(seq == 1, "seq in msp_exprecv_data");
	test_assert(opcode == MSP_OP_SEND_PUS, "opcode in msp_exprecv_data");
	test_assert(len == 1, "len in msp_exprecv_data");
	test_assert(offset == 0, "offset in msp_exprecv_data");

	test_assert(buf[0] == 0x11, "received data in msp_exprecv_data");

	seq++;
}
void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(seq == 2, "seq in msp_exprecv_complete");
	test_assert(opcode == MSP_OP_SEND_PUS, "opcode in msp_exprecv_complete");

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
