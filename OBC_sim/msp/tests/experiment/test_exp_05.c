/*
 * MSP Experiment Test 05
 * Author: John Wikman
 */

#include "test_exp.h"

static int seq = 0;

unsigned char pl_data[30];

void test(void)
{
	static unsigned char buf[1000];
	unsigned long fcs;
	unsigned long len;
	int code;
	int i;

	/* populate pl_data */
	for (i = 0; i < 30; i++)
		pl_data[i] = i;

	/* Testing request payload where the experiment has to retransmit one frame.
	 * we expect to get the following call sequence:
	 * msp_expsend_start()
	 * msp_expsend_data()
	 * msp_expsend_data() [with the same parameters as before]
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
	test_assert(msp_from_bigendian32(buf+1) == 30, "value of DL field");
	test_assert((buf[0] & 0x7F) == MSP_OP_EXP_SEND, "opcode of exp send frame");
	test_assert((buf[0] & 0x80) == 0x00, "Frame ID of EXP_SEND set to 0");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 5) == msp_from_bigendian32(buf+5), "FCS calculation (1)");

	/* Acknowledge exp send */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (3)");

	/* Get the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (4)");
	test_assert(len == (30+5), "data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame ID of data frame set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 31) == msp_from_bigendian32(buf+31), "FCS calculation (2)");

	/*----------------------------------------------------------------*/
	/* HERE WE PRETEND THAT THERE WAS A BIT FLIP WHEN SENDING OVER I2C*/
	/*----------------------------------------------------------------*/

	/* Acknowledge exp send (same as before since FCS did not match up */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9); /* we should expect something due to retransmission */
	/* DEBUG_EXEC(printf("retransmit code: %d\n", code));
	 * test_assert(code == MSP_ERR_FAULTY_FRAME, "Should be notified of faulty frame");
	 */

	/* Get the data frame (again) */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "Unexpected error (6)");
	test_assert(len == (30+5), "data frame length");
	test_assert((buf[0] & 0x7F) == MSP_OP_DATA_FRAME, "opcode of data frame");
	test_assert((buf[0] & 0x80) == 0x80, "Frame ID of data frame set to 1");
	test_assert(msp_exp_frame_generate_fcs(buf, 0, 31) == msp_from_bigendian32(buf+31), "FCS calculation (3)");

	/* Now finally acknowledge transaction */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf+1, 0);
	fcs = msp_exp_frame_generate_fcs(buf, 1, 5);
	msp_to_bigendian32(buf+5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "Unexpected error (7)");

	test_assert(seq == 4, "four handlers should be called in total");
	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_exprecv_data should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	unsigned long i;

	test_assert(seq == 1 || seq == 2, "msp_expsend_data should be called second and third");
	test_assert(offset == 0, "offset should not change between retransmissions");
	test_assert(len == 30, "length of data to send in data frame");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode in msp_exprecv_data");

	for (i = 0; i < len; i++)
		buf[i] = pl_data[i];

	seq++;
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(seq == 0, "msp_expsend_start should be called first");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode should be MSP_OP_REQ_PAYLOAD in msp_expsend_start");

	*len = 30;

	seq++;
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
}
void msp_expsend_complete(unsigned char opcode)
{
	test_assert(seq == 3, "msp_expsend_complete should be called fourth");
	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "opcode should be MSP_OP_REQ_PAYLOAD in msp_expsend_complete");

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
