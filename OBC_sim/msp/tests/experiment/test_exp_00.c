/*
 * MSP Experiment Test 00
 * Author: John Wikman
 */

#include "test_exp.h"

void test(void)
{
	unsigned char data0[5] = {0xB1, 0x00, 0x00, 0x01, 0x04};
	unsigned char data1[10] = {0x11, 0x21, 0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1};
	unsigned char data2[8] = {0x11,0x12,0x13,0x14,0x15,0x16,0xAA,0xFC};
	unsigned long fcs0;
	unsigned long fcs1;
	unsigned long fcs2;

	/* Example from the specification */
	fcs0 = msp_exp_frame_generate_fcs(data0, 1, sizeof(data0));
	test_assert(fcs0 == 0xC877D162, "Specification example");
	DEBUG_EXEC(if (fcs0 != 0xC877D162) printf("fcs0: %lX\n", fcs0));

	/* Some random frame from OBC (verified using a third part CRC calculator)
	 * pseudo header = 0x11 << 1 | 0x00 = 0x22 | 0x00 = 0x22 */
	fcs1 = msp_exp_frame_generate_fcs(data1, 1, sizeof(data1));
	test_assert(fcs1 == 0xDAF5CD26, "Random frame from OBC");
	DEBUG_EXEC(if (fcs1 != 0xDAF5CD26) printf("fcs1: %lX\n", fcs1));

	/* Random frame from the experiment (third part verified CRC)
	 * pseudo header = 0x11 << 1 | 0x01 = 0x22 | 0x01 = 0x23 */
	fcs2 = msp_exp_frame_generate_fcs(data2, 0, sizeof(data2));
	test_assert(fcs2 == 0x9288C5D6, "Random frame from experiment");
	DEBUG_EXEC(if (fcs2 != 0x9288C5D6) printf("fcs2: %lX\n", fcs2));

	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_exprecv_data should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_expsend_data should be unreachable");
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	test_assert(0, "msp_expsend_start should be unreachable");
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
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
