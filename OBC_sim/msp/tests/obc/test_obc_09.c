/*
 * MSP OBC Test 09
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
	unsigned long i;
	struct msp_response r;

	test_link = msp_create_link(0x11, msp_seqflags_init(), test_buf, TEST_MTU);

	/* Set previous transaction_id to 0 */
	msp_seqflags_set(&test_link.flags, MSP_OP_SEND_PUS, 0);



	/* Start OBC send transaction */
	r = msp_start_transaction(&test_link, MSP_OP_SEND_PUS, 30);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 0, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "Transaction start should be OK");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Setup the data */
	for (i = 0; i < 30; i++)
		test_storage[i] = 0xC0;

	/* Send initial header */
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

	/* Receive corrupted T_ACK (FCS mismatch should occur) */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_ERROR, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_ERROR) print_response(r));

	/* Receive NULL frame (since experiment is in the ready state) */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_ABORTED, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_ABORTED) print_response(r));
	test_assert(r.opcode == MSP_OP_SEND_PUS, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 30, "");

	/* Post transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");
	test_assert(seq == 5, "5 frames should've been sent after 1st attempt");



	/* Restart the transaction */
	r = msp_start_transaction(&test_link, MSP_OP_SEND_PUS, 30);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 5, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "Transaction start should be OK");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Send initial header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Receive T_ACK */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));
	test_assert(r.opcode == MSP_OP_SEND_PUS, "");
	test_assert(r.transaction_id == 1, "");
	test_assert(r.len == 30, "");

	/* Post re-transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");
	test_assert(seq == 7, "7 frames should've been sent after both attempts");

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
	unsigned long i;
	int data_ok;

	unsigned long fcs;
	unsigned char pseudo_header;

	pseudo_header = (slave_address << 1);
	fcs = msp_crc32(&pseudo_header, 1, 0);

	test_assert(slave_address == 0x11, "Value of slave_address in msp_i2c_write");
	/* Determine action for each value of seq */
	switch (seq) {
	case 0:
		test_assert(size == 9, "Header frame length");

		test_assert(data[0] == (MSP_OP_SEND_PUS | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 30, "DL = 30");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 2:
		test_assert(size == (30 + 5), "Data frame length");
		
		test_assert(data[0] == MSP_OP_DATA_FRAME, "");
		data_ok = 1;
		for (i = 0; i < 30; i++) {
			if (data[i + 1] != 0xC0)
				data_ok = 0;
		}
		test_assert(data_ok == 1, "");
		fcs = msp_crc32(data, 30 + 1, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 30 + 1), "");
		break;
	case 5:
		test_assert(size == 9, "Header frame length");
		test_assert(data[0] == (MSP_OP_SEND_PUS | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 30, "DL = 30");
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
	case 1:
		test_assert(size == 9, "Header frame length");

		data[0] = (MSP_OP_F_ACK | 0x80);
		msp_to_bigendian32(data + 1, 0);
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 3:
		test_assert(size == 9, "Header frame length");

		data[0] = (MSP_OP_T_ACK | 0x80);
		msp_to_bigendian32(data + 1, 0);
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);

		/* Simulate a bit-flip */
		data[3] ^= 0x20;
		break;
	case 4:
		test_assert(size == 9, "Header frame length");

		data[0] = MSP_OP_NULL;
		msp_to_bigendian32(data + 1, 0);
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 6:
		/* T_ACK for duplicate transaction by OBC */
		test_assert(size == 9, "Header frame length");

		data[0] = (MSP_OP_T_ACK | 0x80);
		msp_to_bigendian32(data + 1, 0);
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


