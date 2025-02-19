/*
 * MSP OBC Test 00
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
	test_assert(!msp_is_active(&test_link), "Link should not be active initially");

	/* Set previous transaction_id to 0 */
	msp_seqflags_set(&test_link.flags, MSP_OP_SEND_PUS, 0);

	r = msp_start_transaction(&test_link, MSP_OP_SEND_PUS, 100);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(test_link.transaction_id == 1, "Transaction-ID should be 1");
	test_assert(test_link.frame_id == 1, "Frame-ID should be 1");
	test_assert(seq == 0, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "Transaction start should be OK");

	/* Send header */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_OK, "");

	/* Receive NULL frame */
	r = simulate_loop(&test_link);
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_ABORTED, "");

	/* Post transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");

	/* Make sure that only 2 frames were sent */
	test_assert(seq == 2, "2 frames shouldve been sent");

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
		test_assert(data[0] == (MSP_OP_SEND_PUS | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 100, "DL = 100");
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

		data[0] = MSP_OP_NULL;
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


