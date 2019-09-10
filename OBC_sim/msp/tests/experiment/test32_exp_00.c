/*
 * MSP Experiment 32-bit Test 00
 * Author: John Wikman
 *
 * Test receiving the maximum amount of data from the OBC.
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
	unsigned long sent_data, current_size;
	unsigned char frame_id;
	float progress;
	int code;

	/* Make sure that the size of a long is exactly 4 bytes in size. */
	test_precondition(sizeof(unsigned long) == 4, "Size of long must be 4 bytes in size");

	/* Testing sending ~4GB to the experiment */
	/* Expected seq = 1 + ceil(DATA_SIZE / MSP_EXP_MTU) + 1 */
	/* I.e. One start, one complete, and one for each data frame */
	expected_seq = DATA_SIZE / MSP_EXP_MTU;
	if (expected_seq % MSP_EXP_MTU != 0)
		expected_seq += 1;
	expected_seq += 2;

	/* Set previous transaction-ID to 1 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_SEND_PUS, 1);

	/* Format send frame */
	buf[0] = MSP_OP_SEND_PUS;
	msp_to_bigendian32(buf+1, DATA_SIZE);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error");

	/* Get acknowledge header */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error");
	test_assert(len == 9, "length of ack frame");
	test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of ack frame");
	test_assert((buf[0] & 0x80) == 0x00, "frame-ID of ack frame");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");

	progress = 0.0f;

	/* Now send all the data frames */
	sent_data = 0;
	frame_id = 1;
	while (sent_data < DATA_SIZE) {
		/* Print progress */
		if (((float) sent_data / (float) DATA_SIZE) >= progress) {
			DEBUG_EXEC(fprintf(stderr, "progress: %.02f\n", 100*progress));
			progress += 0.1f;
		}

		/* Calculate the size of the data frame to send */
		current_size = DATA_SIZE - sent_data;
		if (current_size > MSP_EXP_MTU)
			current_size = MSP_EXP_MTU;

		/* Format send frame */
		buf[0] = MSP_OP_DATA_FRAME | (frame_id << 7);
		fcs = msp_exp_frame_generate_fcs(buf, 1, 1 + current_size);
		msp_to_bigendian32(buf + 1 + current_size, fcs);
		code = msp_recv_callback(buf, current_size + 5);
		test_assert(code == 0, "Unexpected error");

		sent_data += current_size;

		/* Get header frame */
		code = msp_send_callback(buf, &len);
		test_assert(len == 9, "length of header frame");
		test_assert(code == 0, "Unexpected error");
		test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation");
		if (sent_data < DATA_SIZE) {
			/* Decode as acknowledge Header */
			test_assert((buf[0] & 0x7F) == MSP_OP_F_ACK, "opcode of ack frame");
			test_assert((buf[0] & 0x80) == (frame_id << 7), "frame-ID of ack frame");
		} else {
			/* Decode as transaction acknowledgment */
			test_assert((buf[0] & 0x7F) == MSP_OP_T_ACK, "opcode of transaction ack");
			test_assert((buf[0] & 0x80) == 0x00, "frame-ID of transaction ack");
		}

		frame_id ^= 1;

		test_assert(seq <= expected_seq, "Called too many handlers");
		test_checkpoint(); /* Make sure that we dont loop more than expected. */
	}


	DEBUG_EXEC(printf("seq: %lu\t expected seq: %lu\n", seq, expected_seq));
	test_assert(seq == expected_seq, "Expected seq count does not match");
	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(seq > 0, "this should not be the first handler");
	test_assert(seq < (expected_seq - 1), "this should not be the last handler");
	test_assert(opcode == MSP_OP_SEND_PUS, "");

	if (seq == (expected_seq - 2)) {
		/* The last data frame */
		if (DATA_SIZE % MSP_EXP_MTU == 0)
			test_assert(len == MSP_EXP_MTU, "");
		else
			test_assert(len == (DATA_SIZE % MSP_EXP_MTU), "");

	} else {
		/* Not the last data frame, should have len == MTU */
		test_assert(len == MSP_EXP_MTU, "");
	}

	/* Check the offset */
	test_assert(offset == MSP_EXP_MTU*(seq-1), "");

	seq++;
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_expsend_data should be unreachable");
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(seq == 0, "msp_exprecv_start should be called first");
	test_assert(opcode == MSP_OP_SEND_PUS, "start has correct opcode");
	test_assert(len == DATA_SIZE, "SEND_PUS should send DATA_SIZE bytes of data (~4GB)");

	seq++;
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(0, "msp_expsend_start should be unreachable");
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(seq == (expected_seq - 1), "msp_exprecv_complete should be called last");
	test_assert(opcode == MSP_OP_SEND_PUS, "opcode in complete");

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
