/*
 * MSP OBC Test 12
 * Author: John Wikman
 */

#define TEST_MTU 507

#define CUSTOM_OP_SYS_A 0x50
#define CUSTOM_OP_SYS_B 0x57
#define CUSTOM_OP_SYS_C 0x5F

#include "test_obc.h"

struct msp_response simulate_loop(msp_link_t *link);

unsigned char test_buf[TEST_MTU + 5];
unsigned char test_storage[8192];
msp_link_t test_link;

static unsigned int seq = 0;

void test(void)
{
	struct msp_response r;

	test_link = msp_create_link(0x11, msp_seqflags_init(), test_buf, TEST_MTU);

	/* Set previous transaction_id's to 1 */
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_SYS_A, 1);
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_SYS_B, 1);
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_SYS_C, 1);

	/* Start a SYS_A transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_SYS_A, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SYS_A, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 0, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_A) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_C) == 1, "");

	/* Start another SYS_A transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_SYS_A, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SYS_A, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 0, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_C) == 1, "");

	/* Start a SYS_B transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_SYS_B, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SYS_B, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 0, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_B) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_C) == 1, "");

	/* Start a SYS_C transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_SYS_C, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SYS_C, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 0, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_B) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_C) == 0, "");

	/* Start another SYS_B transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_SYS_B, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SYS_B, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 0, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_C) == 0, "");

	/* Start another SYS_C transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_SYS_C, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SYS_C, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 0, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SYS_C) == 1, "");

	test_assert(seq == 12, "12 I2C transmissions should've occured");

	return;
}

struct msp_response simulate_loop(msp_link_t *link)
{
	struct msp_response r;
	unsigned long len, offset;

	switch (msp_next_action(link)) {
	case MSP_LINK_ACTION_TX_HEADER:
		r = msp_send_header_frame(link);
		break;
	case MSP_LINK_ACTION_RX_HEADER:
		r = msp_recv_header_frame(link);
		break;
	case MSP_LINK_ACTION_TX_DATA:
		len = msp_next_data_length(link);
		offset = msp_next_data_offset(link);
		r = msp_send_data_frame(link, test_storage + offset, len);
		break;
	case MSP_LINK_ACTION_RX_DATA:
		offset = msp_next_data_offset(link);
		r = msp_recv_data_frame(link, test_storage + offset, &len);
		break;
	default:
		break;
	}

	return r;
}


int msp_i2c_write(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	unsigned long fcs;
	unsigned char pseudo_header;

	pseudo_header = (slave_address << 1);
	fcs = msp_crc32(&pseudo_header, 1, 0);

	test_assert(slave_address == 0x11, "Value of slave_address in msp_i2c_write");
	/* Determine action for each value of seq */
	switch (seq) {
	case 0: /* 1st SYS_A */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_SYS_A, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 2: /* 2nd SYS_A */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (CUSTOM_OP_SYS_A | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 4: /* 1st SYS_B */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_SYS_B, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 6: /* 1st SYS_C */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_SYS_C, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 8: /* 2nd SYS_B */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (CUSTOM_OP_SYS_B | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 10: /* 2nd SYS_C */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (CUSTOM_OP_SYS_C | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	default:
		test_assert(0, "msp_i2c_write called out of sequence");
		break;
	}
	
	seq++;
	return 0;
}
int msp_i2c_read(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	unsigned long fcs;
	unsigned char pseudo_header;

	pseudo_header = (slave_address << 1) | 0x01;
	fcs = msp_crc32(&pseudo_header, 1, 0);

	test_assert(slave_address == 0x11, "Value of slave_address in msp_i2c_read");
	/* Determine action for each value of seq */
	switch (seq) {
	case 1: /* T_ACK 1st SYS_A */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 3: /* T_ACK 2nd SYS_A */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 5: /* T_ACK 1st SYS_B */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 7: /* T_ACK 1st SYS_C */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 9: /* T_ACK 2nd SYS_B */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 11: /* T_ACK 2nd SYS_C */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	default:
		test_assert(0, "msp_i2c_read called out of sequence");
		break;
	}

	seq++;
	return 0;
}


