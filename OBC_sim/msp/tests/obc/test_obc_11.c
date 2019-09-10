/*
 * MSP OBC Test 11
 * Author: John Wikman
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

	test_link = msp_create_link(0x11, msp_seqflags_init(), test_buf, TEST_MTU);

	/* Set previous transaction_id's to 1 */
	msp_seqflags_set(&test_link.flags, MSP_OP_SEND_PUS, 1);
	msp_seqflags_set(&test_link.flags, MSP_OP_REQ_PAYLOAD, 1);



	/* Send a 1 byte PUS PACKET */
	r = msp_start_transaction(&test_link, MSP_OP_SEND_PUS, 1);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 0, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	test_storage[0] = 0xA1;

	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Receive F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Send data frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == MSP_OP_SEND_PUS, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 1, "");

	/* Post transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");
	test_assert(msp_seqflags_get(&test_link.flags, MSP_OP_SEND_PUS) == 0, "");
	test_assert(seq == 4, "4 frames should've been sent/received after 1st transaction");



	/* Request payload (should be 1 byte in size) */
	r = msp_start_transaction(&test_link, MSP_OP_REQ_PAYLOAD, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 4, "No I2C transmission should be made on transaction start");
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

	/* Receive data frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == MSP_OP_REQ_PAYLOAD, "");
	test_assert(r.transaction_id == 0, "");
	test_assert(r.len == 1, "");

	/* Post transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");
	test_assert(msp_seqflags_get(&test_link.flags, MSP_OP_REQ_PAYLOAD) == 0, "");
	test_assert(seq == 9, "9 frames should've been sent/received in total after 2nd transaction");
	/* Check that the received data is correct */
	test_assert(test_storage[0] == 0xB2, "Integrity of received data (2nd transaction)");



	/* Request payload again (but this time the first ack will get corrupted) */
	r = msp_start_transaction(&test_link, MSP_OP_REQ_PAYLOAD, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 9, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Send request header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Receive response frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Send F_ACK */
	/* THIS WILL GET CORRUPTED DURING TRANSMISSION */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Receive response frame again */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_ERROR, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_ERROR) print_response(r));

	/* Send F_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Receive data frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Send T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == MSP_OP_REQ_PAYLOAD, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 1, "");

	/* Post transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");
	test_assert(msp_seqflags_get(&test_link.flags, MSP_OP_REQ_PAYLOAD) == 1, "");
	test_assert(seq == 16, "16 frames should've been sent/received in total after all transactions");
	/* Check that the received data is correct */
	test_assert(test_storage[0] == 0xC3, "Integrity of received data (3rd transaction)");

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
	case 0: /* Send header (1st transaction) */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_SEND_PUS, "");
		test_assert(msp_from_bigendian32(data + 1) == 1, "DL = 1");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 2: /* Send data frame */
		test_assert(size == (1 + 5), "Data frame size for 1 byte of data");
		test_assert(data[0] == (MSP_OP_DATA_FRAME | 0x80), "");
		test_assert(data[1] == 0xA1, "value of data field");
		fcs = msp_crc32(data, 1 + 1, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 1 + 1), "");
		break;
	case 4: /* Request header (2nd transaction) */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_REQ_PAYLOAD, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 6: /* Ack response frame */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_F_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 8: /* Ack transaction */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_T_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 9: /* Request header (3rd transaction) */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == MSP_OP_REQ_PAYLOAD, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 11: /* Ack response frame */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_F_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");

		/* Pretend like this got corrupted when being sent */
		break;
	case 13: /* Ack response frame (again) */
		test_assert(size == 9, "Header frame size");
		test_assert(data[0] == (MSP_OP_F_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 15: /* Finally ack the 3rd transaction */
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
	case 1: /* Ack send header (1st transaction) */
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_F_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 3:
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_T_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 5:
		test_assert(size == 9, "Header frame size");

		data[0] = MSP_OP_EXP_SEND; /* Transaction-ID == 0 */
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 7:
		test_assert(size == 9, "Data frame size is 6, but the minimum number of bytes read should always be at least 9");

		data[0] = MSP_OP_DATA_FRAME | 0x80;
		data[1] = 0xB2;
		fcs = msp_crc32(data, 1 + 1, fcs);
		msp_to_bigendian32(data + 1 + 1, fcs);
		data[6] = 0xFF; /* padding with 0xFF */
		data[7] = 0xFF;
		data[8] = 0xFF;
		break;
	default:
		test_assert(0, "msp_i2c_read called out of sequence");
		break;
	case 10: /* 3rd transaction response */
		test_assert(size == 9, "Header frame size");

		data[0] = (MSP_OP_EXP_SEND | 0x80); /* Transaction-ID == 1 */
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 12: /* 3rd transaction response again since ack got corrupted */
		test_assert(size == 9, "Header frame size");

		data[0] = (MSP_OP_EXP_SEND | 0x80); /* Transaction-ID == 1 */
		msp_to_bigendian32(data + 1, 1); /* DL = 1 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 14: /* Now send the data frame of the 3rd transaction */
		test_assert(size == 9, "Data frame size is 6, but the minimum number of bytes read should always be at least 9");

		data[0] = MSP_OP_DATA_FRAME;
		data[1] = 0xC3;
		fcs = msp_crc32(data, 1 + 1, fcs);
		msp_to_bigendian32(data + 1 + 1, fcs);
		data[6] = 0xFF; /* padding with 0xFF */
		data[7] = 0xFF;
		data[8] = 0xFF;
		break;
	}

	seq++;
	return 0;
}


