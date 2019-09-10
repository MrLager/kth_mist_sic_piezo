/*
 * MSP OBC Test 14
 * Author: John Wikman
 */

#define TEST_MTU 507

#define CUSTOM_OP_SEND_A 0x70
#define CUSTOM_OP_SEND_B 0x7D
#define CUSTOM_OP_SEND_C 0x7F

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
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_SEND_A, 1);
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_SEND_B, 1);
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_SEND_C, 1);

	/* Start a SEND_A transaction */
	test_storage[0] = 0xAA;
	r = msp_start_transaction(&test_link, CUSTOM_OP_SEND_A, 1);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SEND_A, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_A) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_C) == 1, "");

	/* Start another SEND_A transaction */
	test_storage[0] = 0xAA;
	r = msp_start_transaction(&test_link, CUSTOM_OP_SEND_A, 1);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SEND_A, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_C) == 1, "");

	/* Start a SEND_B transaction */
	test_storage[0] = 0xAA;
	r = msp_start_transaction(&test_link, CUSTOM_OP_SEND_B, 1);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SEND_B, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_B) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_C) == 1, "");

	/* Start a SEND_C transaction */
	test_storage[0] = 0xAA;
	r = msp_start_transaction(&test_link, CUSTOM_OP_SEND_C, 1);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SEND_C, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_B) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_C) == 0, "");

	/* Start another SEND_B transaction */
	test_storage[0] = 0xAA;
	r = msp_start_transaction(&test_link, CUSTOM_OP_SEND_B, 1);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SEND_B, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_C) == 0, "");

	/* Start another SEND_C transaction */
	test_storage[0] = 0xAA;
	r = msp_start_transaction(&test_link, CUSTOM_OP_SEND_C, 1);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_SEND_C, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_SEND_C) == 1, "");


	test_assert(seq == 24, "24 I2C transmissions should've occured");

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
	case 0: /* 1st SEND_A - Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_SEND_A, "");
		test_assert(msp_from_bigendian32(data + 1) == 1, "DL = 1");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 2: /* 1st SEND_A - Data Frame */
		test_assert(size == 6, "Data frame size");
		test_assert(data[0] == (MSP_OP_DATA_FRAME | 0x80), "");
		test_assert(data[1] == 0xAA, "data in data frame");
		fcs = msp_crc32(data, 2, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 2), "");
		break;
	case 4: /* 2nd SEND_A - Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (CUSTOM_OP_SEND_A | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 1, "DL = 1");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 6: /* 2nd SEND_A - Data Frame */
		test_assert(size == 6, "Data frame size");
		test_assert(data[0] == MSP_OP_DATA_FRAME, "");
		test_assert(data[1] == 0xAA, "data in data frame");
		fcs = msp_crc32(data, 2, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 2), "");
		break;
	case 8: /* 1st SEND_B - Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_SEND_B, "");
		test_assert(msp_from_bigendian32(data + 1) == 1, "DL = 1");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 10: /* 1st SEND_B - Data Frame */
		test_assert(size == 6, "Data frame size");
		test_assert(data[0] == (MSP_OP_DATA_FRAME | 0x80), "");
		test_assert(data[1] == 0xAA, "data in data frame");
		fcs = msp_crc32(data, 2, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 2), "");
		break;
	case 12: /* 1st SEND_C - Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_SEND_C, "");
		test_assert(msp_from_bigendian32(data + 1) == 1, "DL = 1");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 14: /* 1st SEND_C - Data Frame */
		test_assert(size == 6, "Data frame size");
		test_assert(data[0] == (MSP_OP_DATA_FRAME | 0x80), "");
		test_assert(data[1] == 0xAA, "data in data frame");
		fcs = msp_crc32(data, 2, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 2), "");
		break;
	case 16: /* 2nd SEND_B - Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (CUSTOM_OP_SEND_B | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 1, "DL = 1");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 18: /* 2nd SEND_B - Data Frame */
		test_assert(size == 6, "Data frame size");
		test_assert(data[0] == MSP_OP_DATA_FRAME, "");
		test_assert(data[1] == 0xAA, "data in data frame");
		fcs = msp_crc32(data, 2, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 2), "");
		break;
	case 20: /* 2nd SEND_C - Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (CUSTOM_OP_SEND_C | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 1, "DL = 1");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 22: /* 2nd SEND_C - Data Frame */
		test_assert(size == 6, "Data frame size");
		test_assert(data[0] == MSP_OP_DATA_FRAME, "");
		test_assert(data[1] == 0xAA, "data in data frame");
		fcs = msp_crc32(data, 2, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 2), "");
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
	case 1: /* 1st SEND_A - F_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_F_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 3: /* 1st SEND_A - T_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 5: /* 2nd SEND_A - F_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_F_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 7: /* 2nd SEND_A - T_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 9: /* 1st SEND_B - F_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_F_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 11: /* 1st SEND_B - T_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 13: /* 1st SEND_C - F_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_F_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 15: /* 1st SEND_C - T_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 17: /* 2nd SEND_B - F_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_F_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 19: /* 2nd SEND_B - T_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 21: /* 2nd SEND_C - F_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_F_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 23: /* 2nd SEND_C - T_ACK */
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


