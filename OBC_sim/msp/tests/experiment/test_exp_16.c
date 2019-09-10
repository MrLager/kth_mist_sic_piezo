/*
 * MSP Experiment Test 16
 * Author: John Wikman
 */

#include "test_exp.h"

#define CUSTOM_OP_SYS_A 0x50
#define CUSTOM_OP_SYS_B 0x59
#define CUSTOM_OP_SYS_C 0x5F

#define CUSTOM_OP_REQ_A 0x60
#define CUSTOM_OP_REQ_B 0x69
#define CUSTOM_OP_REQ_C 0x6F

#define CUSTOM_OP_SEND_A 0x70
#define CUSTOM_OP_SEND_B 0x79
#define CUSTOM_OP_SEND_C 0x7F

void test(void)
{
	/* Test two transactions each for three custom OBC send opcodes. */

	/* Set all sequence flags to 1 */
	msp_exp_state_initialize(msp_seqflags_init());
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_ACTIVE, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_SEND_TIME, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B, 1);
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C, 1);

	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_ACTIVE) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_SEND_TIME) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "");

	/* Update SYS_A */
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A, 0);

	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_ACTIVE) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_SEND_TIME) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 0, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "");

	/* Update SEND_TIME */
	msp_seqflags_set(&msp_exp_state.seqflags, MSP_OP_SEND_TIME, 0);

	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_ACTIVE) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_SEND_TIME) == 0, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 0, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "");

	/* Update REQ_C */
	msp_seqflags_set(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C, 0);

	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_ACTIVE) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_REQ_PAYLOAD) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, MSP_OP_SEND_TIME) == 0, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_A) == 0, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SYS_C) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_REQ_C) == 0, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_B) == 1, "");
	test_assert(msp_seqflags_get(&msp_exp_state.seqflags, CUSTOM_OP_SEND_C) == 1, "");

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
