/*
 * MSP Experiment 32-bit Test 01
 * Author: John Wikman
 *
 * Test sending the maximum amount of data (~4GB) to the OBC.
 */

#define DATA_SIZE 4294967295LU

#include "test_exp.h"


static unsigned long seq = 0;
static unsigned long expected_seq;

void test(void)
{
	static unsigned char buf[1000];
	unsigned long fcs;
	unsigned long len;
	unsigned long received_data, expected_size;
	unsigned char expected_frame_id;
	float progress;
	int code;

	/* Make sure that the size of a long is exactly 4 bytes in size. */
	test_precondition(sizeof(unsigned long) == 4, "Size of long must be 4 bytes in size");

	/* Testing receiving ~4GB from the experiment */
	/* Expected seq = 1 + ceil(DATA_SIZE / MSP_EXP_MTU) + 1 */
	/* I.e. One start, one complete, and one for each data frame */
	expected_seq = DATA_SIZE / MSP_EXP_MTU;
	if (expected_seq % MSP_EXP_MTU != 0)
		expected_seq += 1;
	expected_seq += 2;

	/* Set previous transaction-ID to 1 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD, 1);

	/* Format request header */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf+1, 0); /* DL = 0 */
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get response header */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "length of response header");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of response header");
	test_assert((buf[0] & 0x80) == 0x00, "frame-ID of response header");
	test_assert(msp_from_bigendian32(buf+1) == DATA_SIZE, "experiment should send DATA_SIZE bytes");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");

	/* Acknowledge the response header */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	progress = 0.0f;

	/* Now receive all the data frames */
	received_data = 0;
	expected_frame_id = 1;
	while (received_data < DATA_SIZE) {
		/* Print progress */
		if (((float) received_data / (float) DATA_SIZE) >= progress) {
			DEBUG_EXEC(fprintf(stderr, "progress: %.02f\n", 100*progress));
			progress += 0.1f;
		}

		/* Calculate the size of the data frame to send */
		expected_size = DATA_SIZE - received_data;
		if (expected_size > MSP_EXP_MTU)
			expected_size = MSP_EXP_MTU;

		/* Get data frame */
		code = msp_send_callback(buf, &len);
		test_assert(code == 0, "Unexpected error");
		test_assert(len == (expected_size + 5), "length of data frame");
		test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame");
		test_assert((buf[0] & 0x80) == (expected_frame_id << 7), "frame-ID of data frame");
		/* Ignore the contents of the data frame */
		test_assert(msp_exp_frame_generate_fcs(buf, 0, expected_size + 1) == msp_from_bigendian32(buf + expected_size + 1), "FCS calculation");

		received_data += expected_size;

		/* Format acknowledge frame */
		if (received_data < DATA_SIZE) {
			/* Frame Ack */
			buf[0] = MSP_OP_F_ACK | (expected_frame_id << 7);
			msp_to_bigendian32(buf+1, 0);
			fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
			msp_to_bigendian32(buf+5, fcs);
			code = msp_recv_callback(buf, 9);
			test_assert(code == 0, "Unexpected error");
		} else {
			/* Transaction Ack */
			buf[0] = MSP_OP_T_ACK; /* transaction-ID == 0 */
			msp_to_bigendian32(buf+1, 0);
			fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
			msp_to_bigendian32(buf+5, fcs);
			code = msp_recv_callback(buf, 9);
			test_assert(code == 0, "Unexpected error");
		}

		expected_frame_id ^= 1;

		test_assert(seq <= expected_seq, "Called too many handlers");
		test_checkpoint(); /* Make sure that we dont loop more than expected. */
	}


	DEBUG_EXEC(printf("seq: %lu\t expected seq: %lu\n", seq, expected_seq));
	test_assert(seq == expected_seq, "Expected seq count does not match");
	test_assert(received_data == DATA_SIZE, "received correct number of bytes");
	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_exprecv_data should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long expected_len;

	test_assert(seq > 0, "msp_expsend_data should not be called first");
	test_assert(seq < (expected_seq - 1), "msp_expsend_data should not be called last");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "");

	expected_len = MSP_EXP_MTU;
	if (seq == (expected_seq - 2)) {
		/* If this is the last data frame, we should have a bit smaller smaller
		 * len if DATA_SIZE is not a multiple of MTU. */
		if (DATA_SIZE % MSP_EXP_MTU != 0)
			expected_len = DATA_SIZE % MSP_EXP_MTU;
	}
	test_assert(len == expected_len, "");

	/* Check offset (all previous sent data should be MTU in size) */
	test_assert(offset == MSP_EXP_MTU*(seq-1), "");

	/* Ignore filling up the frame with data, just let whatever is in the buffer be there. */

	seq++;
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "");
	test_assert(seq == 0, "msp_expsend_data should be called first");

	*len = DATA_SIZE;

	seq++;
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(seq == (expected_seq - 1), "msp_expsend_complete should be called last");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "");

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
