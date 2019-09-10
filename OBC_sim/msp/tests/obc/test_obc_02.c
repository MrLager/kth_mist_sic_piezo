/*
 * MSP OBC Test 02
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

	/* Set previous transaction_id to 1 */
	msp_seqflags_set(&test_link.flags, MSP_OP_SEND_TIME, 1);

	r = msp_start_transaction(&test_link, MSP_OP_SEND_TIME, 4);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(test_link.transaction_id == 0, "Transaction-ID should be 0");
	test_assert(test_link.frame_id == 0, "Frame-ID should be 0");
	test_assert(seq == 0, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "Transaction start should be OK");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Send initial header */
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_TX_HEADER, "Send header as first action");
	r = simulate_loop(&test_link);
	test_assert(test_link.frame_id == 0, "Frame-ID should still be 0");
	test_assert(r.status == MSP_RESPONSE_OK, "Sending initial header should be OK (1st action)");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Read an F_ACK */
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_RX_HEADER, "Recieve header as second action");
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "Receiving header should be OK (2nd action)");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	test_storage[0] = 0x19;
	test_storage[1] = 0x93;
	test_storage[2] = 0x09;
	test_storage[3] = 0x16;

	/* Send time */
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_TX_DATA, "Send data as third action");
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "Sending data should be OK (3rd action)");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	/* Read a T_ACK */
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_RX_HEADER, "Recieve header as fourth action");
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "Transaction should be successful (4th action)");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_TRANSACTION_SUCCESSFUL) print_response(r));

	/* Post transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");

	/* Make sure that four frames were sent */
	test_assert(seq == 4, "4 frames shouldve been sent");

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
	case 0:
		test_assert(data[0] == MSP_OP_SEND_TIME, "");
		test_assert(msp_from_bigendian32(data + 1) == 4, "DL = 4");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 2:
		test_assert(data[0] == (MSP_OP_DATA_FRAME | 0x80), "");
		test_assert(data[1] == 0x19, "");
		test_assert(data[2] == 0x93, "");
		test_assert(data[3] == 0x09, "");
		test_assert(data[4] == 0x16, "");
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
		test_assert(size == 9, "len should be 9 at seq: 1");

		data[0] = MSP_OP_F_ACK;
		msp_to_bigendian32(data + 1, 0);
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 3:
		test_assert(size == 9, "len should be 9 at seq: 3");

		data[0] = MSP_OP_T_ACK;
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


