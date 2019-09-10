/*
 * MSP Experiment Test 04
 * Author: John Wikman
 */

#include "test_exp.h"

static int seq = 0;

unsigned char hk_data[1389];

void test(void)
{
	static unsigned char buf[1000];
	unsigned long fcs;
	unsigned long len;
	int code;
	int i;

	/* Testing request housekeeping 1389 bytes ()
	 * we expect to get the following call sequence:
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_data()
	 * msp_expsend_data()
	 * msp_expsend_complete()
	 */

	/* Previous Transaction ID for EXP_SEND should have been 0. */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_REQ_HK, 0);

	/* populate HK data with predicable frame values */
	for (i = 0; i < 1389; i++) {
		if (i < 507)
			hk_data[i] = 0x11;
		else if (i < 507*2)
			hk_data[i] = 0x22;
		else
			hk_data[i] = 0x33;
	}

	/* Format request frame */
	buf[0] = MSP_OP_REQ_HK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (1)");

	/* Get the exp send frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (2)");
	test_assert(len == 9, "header frame length");
	test_assert(msp_from_bigendian32(buf+1) == 1389, "value of DL field");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of exp send frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame ID of EXP_SEND set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* Send the ACK frame */
	buf[0] = MSP_OP_F_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (3)");

	/* Get the first data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (4)");
	test_assert(len == (507+5), "data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame ID of first data frame should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 508) == msp_from_bigendian32(buf+508), "FCS calculation (2)");
	for (i = 1; i < 508; i++) {
		if (buf[i] != 0x11) {
			test_assert(0, "datatype in first data frame.");
			break;
		}
	}

	/* Send the ACK frame */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (5)");

	/* Get the second data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (6)");
	test_assert(len == (507+5), "data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame ID of second data frame should be set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 508) == msp_from_bigendian32(buf+508), "FCS calculation (3)");
	for (i = 1; i < 508; i++) {
		if (buf[i] != 0x22) {
			test_assert(0, "datatype in second data frame.");
			break;
		}
	}

	/* Send the ACK frame */
	buf[0] = MSP_OP_F_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (7)");

	/* Get the third data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (8)");
	test_assert(len == (375+5), "data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame ID of third data frame should be set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 376) == msp_from_bigendian32(buf+376), "FCS calculation (3)");
	for (i = 1; i < 376; i++) {
		if (buf[i] != 0x33) {
			test_assert(0, "datatype in third data frame.");
			break;
		}
	}

	/* Send the transaction acknowledge */
	buf[0] = MSP_OP_T_ACK | 0x80;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (9)");


	test_assert(seq == 5, "five handlers should be called in total");
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
		test_assert(offset == 0, "offset should be 0 for first data frame");
		test_assert(len == 507, "length for first data frame should be 507");
	} else if (seq == 2) {
		test_assert(offset == 507, "offset should be 507 for second data frame");
		test_assert(len == 507, "length for second data frame should be 507");
	} else if (seq == 3) {
		test_assert(offset == 1014, "offset should be 1014 for third data frame");
		test_assert(len == 375, "length for third data frame should be 375");
	} else {
		test_assert(0, "unexpected call to msp_expsend_data");
	}
	test_assert(opcode == MSP_OP_REQ_HK, "opcode should be MSP_OP_REQ_HK in msp_expsend_data");

	for (i = 0; i < len; i++)
		buf[i] = hk_data[offset+i];

	seq++;
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(seq == 0, "msp_expsend_start should be called first");
	test_assert(opcode == MSP_OP_REQ_HK, "opcode should be MSP_OP_REQ_HK in msp_expsend_start");

	*len = 1389;

	seq++;
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(seq == 4, "msp_expsend_complete should be called fifth");
	test_assert(opcode == MSP_OP_REQ_HK, "opcode should be MSP_OP_REQ_HK in msp_expsend_complete");

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
