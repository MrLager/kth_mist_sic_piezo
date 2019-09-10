/*
 * MSP Experiment Test 11
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

	/* Testing where a transaction acknowledge frame gets corrupted and the OBC
	 * asks for it again. The functions should be called in the following
	 * order:
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_error()
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_complete()
	 */

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
	test_assert(msp_from_bigendian32(buf+1) == 52, "value of DL field for first send frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of first exp send frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of the first EXP_SEND should be set to 1");
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
	test_assert(len == 52+5, "data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of first data frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of the first data frame should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 52+1) == msp_from_bigendian32(buf+52+1), "FCS calculation (2)");
	pl_content_faulty = 0;
	for (i = 0; i < 52; i++)
		pl_content_faulty |= (buf[i+1] != 0x22);
	test_assert(!pl_content_faulty, "content of data frame matching what was put in.");


	/* Now we send another request of nowhere, interrupting the previous
	 * transaction. The experiment must be able to recover from this. */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (5)");
	DEBUG_EXEC(if (code) printf("%d\n", code));

	/* verify that the transaction-ID has not been updated */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 0, "seqflag should not be updated on error (1)");

	/* Get the exp send frame (again) */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (6)");
	test_assert(len == 9, "second exp_send header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 52, "value of DL field for second send frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of second exp send frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of the second EXP_SEND should still be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (3)");

	/* verify that the transaction-ID has not been updated */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 0, "seqflag should not be updated on error (2)");

	/* Acknowledge the send frame (again) */
	buf[0] = MSP_OP_F_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (7)");

	/* Get the data frame (again) */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (8)");
	test_assert(len == 52+5, "second data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of second data frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of the second data frame should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 52+1) == msp_from_bigendian32(buf+52+1), "FCS calculation (2)");
	pl_content_faulty = 0;
	for (i = 0; i < 52; i++)
		pl_content_faulty |= (buf[i+1] != 0x22);
	test_assert(!pl_content_faulty, "content of second data frame matching what was put in.");

	/* Finally, acknowledge the transaction */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (9)");

	/* not it should actually have been updated */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 1, "seqflag should now be set to 1");

	test_assert(seq == 6, "6 handlers should be called");
	return;
}


void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(seq == 0 || seq == 3, "msp_expsend_data should be called first or fourth");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode should be MSP_OP_REQ_PAYLOAD in msp_expsend_start");
	*len = 52;

	seq++;
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	test_assert(seq == 1 || seq == 4, "msp_expsend_data should be called second or fifth");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode should be MSP_OP_REQ_PAYLOAD in msp_expsend_data");
	test_assert(len == 52, "length should be 52 bytes in msp_expsend_data");
	test_assert(offset == 0, "offset should be 0 in msp_expsend_data");
	for (i = 0; i < len; i++)
		buf[i] = 0x22;

	seq++;
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(seq == 5, "msp_expsend_complete should be called sixth");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_complete");

	seq++;
}
void msp_expsend_error(unsigned char opcode, int error)
{
	test_assert(seq == 2, "msp_expsend_error should be called third");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_expsend_error");
	DEBUG_EXEC(printf("transaction error: %d\n", error));

	seq++;
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
