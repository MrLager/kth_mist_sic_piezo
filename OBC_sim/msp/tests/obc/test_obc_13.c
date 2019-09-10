/*
 * MSP OBC Test 13
 * Author: John Wikman
 */

#define TEST_MTU 507

#define CUSTOM_OP_REQ_A 0x60
#define CUSTOM_OP_REQ_B 0x64
#define CUSTOM_OP_REQ_C 0x6F

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
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_REQ_A, 1);
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_REQ_B, 1);
	msp_seqflags_set(&test_link.flags, CUSTOM_OP_REQ_C, 1);

	/* Start a REQ_A transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_REQ_A, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive response */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_REQ_A, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 1, "");
	test_assert(test_storage[0] == 0xA1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_A) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_C) == 1, "");

	/* Start another REQ_A transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_REQ_A, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive response */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_REQ_A, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 1, "");
	test_assert(test_storage[0] == 0xA1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_C) == 1, "");

	/* Start a REQ_A transaction which will have a duplicate response */
	r = msp_start_transaction(&test_link, CUSTOM_OP_REQ_A, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive (duplicate) response */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_REQ_A, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 0, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_C) == 1, "");

	/* Start a REQ_B transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_REQ_B, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive response */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_REQ_B, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 1, "");
	test_assert(test_storage[0] == 0xA1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_B) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_C) == 1, "");

	/* Start a REQ_C transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_REQ_C, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive response */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_REQ_C, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 1, "");
	test_assert(test_storage[0] == 0xA1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_B) == 0, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_C) == 0, "");

	/* Start another REQ_B transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_REQ_B, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive response */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_REQ_B, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 1, "");
	test_assert(test_storage[0] == 0xA1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_C) == 0, "");

	/* Start another REQ_C transaction */
	r = msp_start_transaction(&test_link, CUSTOM_OP_REQ_C, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive response */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == CUSTOM_OP_REQ_C, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 1, "");
	test_assert(test_storage[0] == 0xA1, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");
	/* Verify the sequence flags for A, B, and C */
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_A) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_B) == 1, "");
	test_assert(msp_seqflags_get(&test_link.flags, CUSTOM_OP_REQ_C) == 1, "");

	test_assert(seq == 33, "33 I2C transmissions should've occured (5 for each transaction and 3 for the duplicate)");

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
	case 0: /* 1st REQ_A - Request Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_REQ_A, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 2: /* 1st REQ_A - F_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_F_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 4: /* 1st REQ_A - T_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_T_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 5: /* 2nd REQ_A - Request Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_REQ_A, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 7: /* 2nd REQ_A - F_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_F_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 9: /* 2nd REQ_A - T_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_T_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 10: /* Duplicate REQ_A - Request Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_REQ_A, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 12: /* Duplicate REQ_A - T_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_T_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 13: /* 1st REQ_B - Request Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_REQ_B, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 15: /* 1st REQ_B - F_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_F_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 17: /* 1st REQ_B - T_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_T_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 18: /* 1st REQ_C - Request Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_REQ_C, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 20: /* 1st REQ_C - F_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_F_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 22: /* 1st REQ_C - T_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_T_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 23: /* 2nd REQ_B - Request Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_REQ_B, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 25: /* 2nd REQ_B - F_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_F_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 27: /* 2nd REQ_B - T_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_T_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 28: /* 2nd REQ_C - Request Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == CUSTOM_OP_REQ_C, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 30: /* 2nd REQ_C - F_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_F_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 32: /* 2nd REQ_C - T_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_T_ACK | 0x80), "");
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
	case 1: /* 1st REQ_A - Response Header */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND;
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 3: /* 1st REQ_A - Data Frame */
		test_assert(size == 9, "max(data frame size, header size)");

		data[0] = MSP_OP_DATA_FRAME | 0x80;
		data[1] = 0xA1;
		fcs = msp_crc32(data, 2, fcs);
		msp_to_bigendian32(data + 2, fcs);
		break;
	case 6: /* 2nd REQ_A - Response Header */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND | 0x80;
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 8: /* 2nd REQ_A - Data Frame */
		test_assert(size == 9, "max(data frame size, header size)");

		data[0] = MSP_OP_DATA_FRAME;
		data[1] = 0xA1;
		fcs = msp_crc32(data, 2, fcs);
		msp_to_bigendian32(data + 2, fcs);
		break;
	case 11: /* Duplicate REQ_A - Response Header */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND | 0x80;
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 14: /* 1st REQ_B - Response Header */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND;
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 16: /* 1st REQ_B - Data Frame */
		test_assert(size == 9, "max(data frame size, header size)");

		data[0] = MSP_OP_DATA_FRAME | 0x80;
		data[1] = 0xA1;
		fcs = msp_crc32(data, 2, fcs);
		msp_to_bigendian32(data + 2, fcs);
		break;
	case 19: /* 1st REQ_C - Response Header */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND;
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 21: /* 1st REQ_C - Data Frame */
		test_assert(size == 9, "max(data frame size, header size)");

		data[0] = MSP_OP_DATA_FRAME | 0x80;
		data[1] = 0xA1;
		fcs = msp_crc32(data, 2, fcs);
		msp_to_bigendian32(data + 2, fcs);
		break;
	case 24: /* 2nd REQ_B - Response Header */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND | 0x80;
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 26: /* 2nd REQ_B - Data Frame */
		test_assert(size == 9, "max(data frame size, header size)");

		data[0] = MSP_OP_DATA_FRAME;
		data[1] = 0xA1;
		fcs = msp_crc32(data, 2, fcs);
		msp_to_bigendian32(data + 2, fcs);
		break;
	case 29: /* 2nd REQ_C - Response Header */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND | 0x80;
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 31: /* 2nd REQ_C - Data Frame */
		test_assert(size == 9, "max(data frame size, header size)");

		data[0] = MSP_OP_DATA_FRAME;
		data[1] = 0xA1;
		fcs = msp_crc32(data, 2, fcs);
		msp_to_bigendian32(data + 2, fcs);
		break;
	default:
		test_assert(0, "msp_i2c_read called out of sequence");
		break;
	}

	seq++;
	return 0;
}


