/*
 * MSP Experiment Test 08
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
	int i;

	/* Testing two housekeeping requests in a row. We expect the following
	 * sequence of function calls:
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_complete()
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_complete()
	 */

	/* Set previous transaction-ID to 0 */
	msp_exp_state_initialize(msp_seqflags_init());
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
	test_assert(len == 9, "first exp_send header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 13, "value of DL field for first send frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of first exp send frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID first of EXP_SEND should be set to 1");
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
	test_assert(len == (13+5), "first data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of the first data frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame ID of the first data frame should set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 13+1) == msp_from_bigendian32(buf+13+1), "FCS calculation (2)");
	hk_content_faulty = 0;
	for (i = 0; i < 13; i++)
		hk_content_faulty |= (buf[i+1] != 0x13);
	test_assert(!hk_content_faulty, "Receive correct housekeeping data (1)");

	/* Acknowledge the transaction */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (5)");

	/* Check that the sequence flag is set to 1 */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_HK) == 1, "sequence flag updated after first housekeeping request");

	/* Now request HK again */
	buf[0] = MSP_OP_REQ_HK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (6)");

	/* Get the second exp send frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (7)");
	test_assert(len == 9, "second exp_send header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 13, "value of DL field for second send frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of second exp send frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of second EXP_SEND should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (3)");

	/* Acknowledge the send frame */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (8)");

	/* Get the second data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (9)");
	test_assert(len == (13+5), "second data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of the second data frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame ID of the second data frame should set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 13+1) == msp_from_bigendian32(buf+13+1), "FCS calculation (4)");
	hk_content_faulty = 0;
	for (i = 0; i < 13; i++)
		hk_content_faulty |= (buf[i+1] != 0x23);
	test_assert(!hk_content_faulty, "Receive correct housekeeping data (2)");

	/* Acknowledge the second transaction */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (10)");

	/* Check that the sequence flag is set to 1 */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_HK) == 0, "sequence flag updated after second housekeeping request");


	test_assert(seq == 6, "should have called 6 handlers");
	return;
}


void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	if (seq == 0) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode of first transaction in msp_expsend_start");
		*len = 13;
	} else if (seq == 3) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode of second transaction in msp_expsend_start");
		*len = 13;
	} else {
		test_assert(0, "msp_expsend_start should be unreachable at this point");
	}

	seq++;
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	if (seq == 1) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode of first transaction in msp_expsend_data");
		test_assert(len == 13, "length of first transaction in msp_expsend_data");
		test_assert(offset == 0, "offset for the first transaction should be 0 in msp_expsend_data");
		for (i = 0; i < len; i++)
			buf[i] = 0x13;
	} else if (seq == 4) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode of second transaction in msp_expsend_data");
		test_assert(len == 13, "length of second transaction in msp_expsend_data");
		test_assert(offset == 0, "offset for the second transaction should be 0 in msp_expsend_data");
		for (i = 0; i < len; i++)
			buf[i] = 0x23;
	} else {
		test_assert(0, "msp_expsend_data should be unreachable at this point");
	}

	seq++;
}
void msp_expsend_complete(unsigned char opcode)
{
	if (seq == 2) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode of first transaction in msp_expsend_complete");
	} else if (seq == 5) {
		test_assert(opcode == MSP_OP_REQ_HK, "opcode of second transaction in msp_expsend_complete");
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
