/*
 * MSP OBC Test 15
 * Author: John Wikman
 *
 * Test correct OBC handling when receiving EXP_BUSY frames.
 */

#define TEST_MTU 507

#include "test_obc.h"

struct msp_response simulate_loop(msp_link_t *link);

unsigned char test_buf[TEST_MTU + 5];
unsigned char test_storage[8192];
msp_link_t test_link;

static unsigned int seq = 0;

void test(void)
{
	struct msp_response r;
	unsigned long i, is_ok;

	test_link = msp_create_link(0x11, msp_seqflags_init(), test_buf, TEST_MTU);

	/* Set previous transaction_id's to 1 */
	msp_seqflags_set(&test_link.flags, MSP_OP_SEND_TIME, 1);
	msp_seqflags_set(&test_link.flags, MSP_OP_REQ_HK, 1);

	/* SEND TIME */
	/* Try receiving a BUSY after sending initial header and the data frame. */
	test_storage[0] = 0x19;
	test_storage[1] = 0x93;
	test_storage[2] = 0x09;
	test_storage[3] = 0x16;
	r = msp_start_transaction(&test_link, MSP_OP_SEND_TIME, 4);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive F_ACK (but should be busy) */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_BUSY, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_BUSY) print_response(r));
	/* Receive F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send Data Frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive T_ACK (but should be busy) */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_BUSY, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_BUSY) print_response(r));
	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == MSP_OP_SEND_TIME, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 4, "");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");

	/* REQUEST HOUSEKEEPING */
	/* Try receiving a BUSY instead of response header and data frame. */
	r = msp_start_transaction(&test_link, MSP_OP_REQ_HK, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive response header (but should be busy) */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_BUSY, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_BUSY) print_response(r));
	/* Receive response header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Receive data frame (but should be busy) */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_BUSY, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_BUSY) print_response(r));
	/* Receive data frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));
	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == MSP_OP_REQ_HK, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 10, "");
	is_ok = 1;
	for (i = 0; i < 10; i++) {
		if (test_storage[i] != 0xF4)
			is_ok = 0;
	}
	test_assert(is_ok, "integrity of requested data");
	test_assert(!msp_is_active(&test_link), "Link should be inactive after a successful transaction");


	test_assert(seq == 13, "13 I2C transmissions should've occured");

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
	case 0: /* SEND_TIME - Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_SEND_TIME, "");
		test_assert(msp_from_bigendian32(data + 1) == 4, "DL = 4");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 3: /* SEND_TIME - Data Frame */
		test_assert(size == 9, "Data frame size");
		test_assert(data[0] == (MSP_OP_DATA_FRAME | 0x80), "");
		test_assert(data[1] == 0x19, "data in data frame");
		test_assert(data[2] == 0x93, "data in data frame");
		test_assert(data[3] == 0x09, "data in data frame");
		test_assert(data[4] == 0x16, "data in data frame");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 6: /* REQ_HK - Request Header */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_REQ_HK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 9: /* REQ_HK - F_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_F_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 12: /* REQ_HK - T_ACK */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_T_ACK, "");
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
	unsigned long fcs, i;
	unsigned char pseudo_header;

	pseudo_header = (slave_address << 1) | 0x01;
	fcs = msp_crc32(&pseudo_header, 1, 0);

	test_assert(slave_address == 0x11, "Value of slave_address in msp_i2c_read");
	/* Determine action for each value of seq */
	switch (seq) {
	case 1: /* SEND_TIME - Busy */
		data[0] = MSP_OP_EXP_BUSY;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 2: /* SEND_TIME - F_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_F_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 4: /* SEND_TIME - Busy */
		data[0] = MSP_OP_EXP_BUSY;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 5: /* SEND_TIME - T_ACK */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 7: /* REQ_HK - Busy */
		data[0] = MSP_OP_EXP_BUSY;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 8: /* REQ_HK - Response Header */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND;
		msp_to_bigendian32(data + 1, 10); /* DL = 10 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 10: /* REQ_HK - Busy */
		data[0] = MSP_OP_EXP_BUSY;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 11: /* REQ_HK - Data Frame */
		test_assert(size == 15, "Data frame size");

		data[0] = MSP_OP_DATA_FRAME | 0x80;
		for (i = 0; i < 10; i++) /* Fill up data frame */
			data[i + 1] = 0xF4;
		fcs = msp_crc32(data, 11, fcs);
		msp_to_bigendian32(data + 11, fcs);
		break;
	default:
		test_assert(0, "msp_i2c_read called out of sequence");
		break;
	}

	seq++;
	return 0;
}


