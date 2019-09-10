/*
 * MSP Experiment Test 09
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
	unsigned long i;

	/* Testing where a transaction acknowledge frame gets corrupted and the OBC
	 * asks for it again. The functions should be called in the following
	 * order:
	 * msp_exprecv_start()
	 * msp_exprecv_data()
	 * msp_exprecv_complete()
	 * 
	 * i.e., we should not get a function call for when receiving a duplicate
	 * transaction */

	/* initialize previous transaction-ID to 0 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_SEND_PUS, 0);

	/* Format the send frame */
	buf[0] = MSP_OP_SEND_PUS | 0x80;
	msp_to_bigendian32(buf+1, 30);
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
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of ack frame should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* Format the data frame */
	buf[0] = MSP_OP_DATA_FRAME;
	for (i = 0; i < 30; i++)
		buf[i+1] = 0x73;
	fcs = msp_exp_frame_generate_fcs(buf, 1, 30+1);
	msp_to_bigendian32(buf+30+1, fcs);
	code = msp_recv_callback(buf, 30+5);
	test_assert(code == 0, "Unexpected error (3)");

	/* Get the transaction ack frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (4)");
	test_assert(len == 9, "transaction ack frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for transaction ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of ack frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of transaction ack frame should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (2)");

	/* Check that the sequence flag is set to 1 */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_SEND_PUS) == 1, "sequence flag updated after T_ACK");

	/* Now we pretend that we get an error when sending the T_ACK, so we ask
	 * the experiment for a frame again. We should get a NULL frame then. */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (5)");
	test_assert(len == 9, "NULL frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for NULL frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_NULL, "opcode of NULL frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame-ID of NULL frame should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (3)");


	/* Now the OBC should retransmit the PUS packet since it did not get a
	 * transaction acknowledge */
	buf[0] = MSP_OP_SEND_PUS | 0x80;
	msp_to_bigendian32(buf+1, 30);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (6)");

	/* Get the transaction acknowledge frame again */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (7)");
	test_assert(len == 9, "transaction ack frame length");
	test_assert(msp_from_bigendian32(buf+1) == 0, "value of DL field for the second transaction ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of ack frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame-ID of the second transaction ack frame should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (4)");

	/* Check that the sequence flag is unchanged after the retransmission */
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_SEND_PUS) == 1, "sequence flag unchanged after retransmission");

	test_assert(seq == 3, "only 3 handlers should be called");
	return;
}


void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(0, "msp_expsend_start should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_expsend_data should be unreachable");
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(0, "msp_expsend_complete should be unreachable");
}
void msp_expsend_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_expsend_error should be unreachable");
}


void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(seq == 0, "msp_exprecv_start should be the first handler to be called");
	test_assert(opcode == MSP_OP_SEND_PUS, "opcode in msp_exprecv_start");
	test_assert(len == 30, "length of data to be received in msp_exprecv_start");

	seq++;
}
void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	test_assert(seq == 1, "msp_exprecv_data should be the second handler to be called");
	test_assert(opcode == MSP_OP_SEND_PUS, "opcode in msp_exprecv_data");
	test_assert(len == 30, "length of received data in msp_exprecv_data");
	test_assert(offset == 0, "offset should be 0");
	for (i = 0; i < len; i++) {
		if (buf[i] != 0x73) {
			test_assert(0, "received data does not match what was sent");
			break;
		}
	}

	seq++;
}
void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(seq == 2, "msp_exprecv_complete should be the third handler to be called");
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
