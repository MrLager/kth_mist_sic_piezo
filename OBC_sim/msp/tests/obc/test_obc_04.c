/*
 * MSP OBC Test 04
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
	unsigned long i;
	int is_correct;

	test_link = msp_create_link(0x11, msp_seqflags_init(), test_buf, TEST_MTU);

	/* Set previous transaction_id to 0 */
	msp_seqflags_set(&test_link.flags, MSP_OP_REQ_HK, 0);

	r = msp_start_transaction(&test_link, MSP_OP_REQ_HK, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 0, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "Transaction start should be OK");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Send Request Header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Receive Response */
	r = simulate_loop(&test_link);
	test_assert(test_link.transaction_id == 1, "Transaction-ID be set to 1");
	test_assert(test_link.frame_id == 1, "Frame-ID should be set to 1");
	test_assert(test_link.total_length == 1389, "total_length of the transaction should be set in the link");
	test_assert(test_link.processed_length == 0, "processed_length should be set in the link");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* F_ACK */
	r = simulate_loop(&test_link);
	test_assert(test_link.transaction_id == 1, "");
	test_assert(test_link.frame_id == 1, "");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* 1st data frame */
	r = simulate_loop(&test_link);
	test_assert(test_link.transaction_id == 1, "");
	test_assert(test_link.frame_id == 0, "");
	test_assert(test_link.processed_length == 507, "");
	test_assert(test_link.total_length == 1389, "");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* F_ACK */
	r = simulate_loop(&test_link);
	test_assert(test_link.transaction_id == 1, "");
	test_assert(test_link.frame_id == 0, "");
	test_assert(test_link.processed_length == 507, "");
	test_assert(test_link.total_length == 1389, "");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* 2nd data frame */
	r = simulate_loop(&test_link);
	test_assert(test_link.transaction_id == 1, "");
	test_assert(test_link.frame_id == 1, "");
	test_assert(test_link.processed_length == 1014, "");
	test_assert(test_link.total_length == 1389, "");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* F_ACK */
	r = simulate_loop(&test_link);
	test_assert(test_link.transaction_id == 1, "");
	test_assert(test_link.frame_id == 1, "");
	test_assert(test_link.processed_length == 1014, "");
	test_assert(test_link.total_length == 1389, "");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* 3rd (final) data frame */
	r = simulate_loop(&test_link);
	test_assert(test_link.transaction_id == 1, "");
	test_assert(test_link.processed_length == 1389, "");
	test_assert(test_link.total_length == 1389, "");
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.len == 1389, "");
	test_assert(r.opcode == MSP_OP_REQ_HK, "");

	/* Post transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");
	test_assert(msp_seqflags_get(&test_link.flags, MSP_OP_REQ_HK) == 1, "");

	/* Make sure that nine frames were sent */
	test_assert(seq == 9, "9 frames shouldve been sent");

	/* Check that the received data is correct */
	is_correct = 1;
	for (i = 0; i < 1389; i++) {
		if (i < 507) {
			if (test_storage[i] != 0x11)
				is_correct = 0;
		} else if (i < 1014) {
			if (test_storage[i] != 0x22)
				is_correct = 0;
		} else {
			if (test_storage[i] != 0x33)
				is_correct = 0;
		}
	}
	test_assert(is_correct, "Integrity of received data");

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
	case 0: /* Request header */
		test_assert(data[0] == MSP_OP_REQ_HK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 2: /* Ack response frame */
		test_assert(data[0] == (MSP_OP_F_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 4: /* Ack first data frame */
		test_assert(data[0] == MSP_OP_F_ACK, "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 6: /* Ack second data frame */
		test_assert(data[0] == (MSP_OP_F_ACK | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 8: /* Ack the transaction */
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
	unsigned long i;

	unsigned long fcs;
	unsigned char pseudo_header;

	pseudo_header = (slave_address << 1) | 0x01;
	fcs = msp_crc32(&pseudo_header, 1, 0);

	test_assert(slave_address == 0x11, "Value of slave_address in msp_i2c_read");
	/* Determine action for each value of seq */
	switch (seq) {
	case 1:
		test_assert(size == 9, "len should be 9 at seq: 1");

		data[0] = MSP_OP_EXP_SEND | 0x80; /* Transaction-ID == 1 */
		msp_to_bigendian32(data + 1, 1389); /* DL = 1389 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 3:
		test_assert(size == (507 + 5), "length of data frame");

		data[0] = MSP_OP_DATA_FRAME;
		for (i = 0; i < 507; i++)
			data[i + 1] = 0x11;
		fcs = msp_crc32(data, 507 + 1, fcs);
		msp_to_bigendian32(data + 507 + 1, fcs);
		break;
	case 5:
		test_assert(size == (507 + 5), "length of data frame");

		data[0] = MSP_OP_DATA_FRAME | 0x80;
		for (i = 0; i < 507; i++)
			data[i + 1] = 0x22;
		fcs = msp_crc32(data, 507 + 1, fcs);
		msp_to_bigendian32(data + 507 + 1, fcs);
		break;
	case 7:
		test_assert(size == (375 + 5), "length of data frame");

		data[0] = MSP_OP_DATA_FRAME;
		for (i = 0; i < 375; i++)
			data[i + 1] = 0x33;
		fcs = msp_crc32(data, 375 + 1, fcs);
		msp_to_bigendian32(data + 375 + 1, fcs);
		break;
	default:
		test_assert(0, "msp_i2c_read called out of sequence");
		break;
	}

	seq++;
	return 0;
}


