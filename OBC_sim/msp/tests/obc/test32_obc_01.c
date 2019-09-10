/*
 * MSP OBC 32-bit Test 01
 * Author: John Wikman
 *
 * Test receiving the maximum amount of data (~4GB) from a request.
 */

#define TEST_MTU 507
#define DATA_SIZE 4294967295LU

/* Expected number of frames: 3 + 2*ceil(DATA_SIZE / TEST_MTU) =
 * = 3 + 2*ceil(4294967295 / 507) = 3 + 2*8471336 = 16942675
 * 1 Request + 1 Response + 1 ACK + ceil(DATA_SIZE / TEST_MTU)*(1 Data + 1 ACK) */
#define EXPECTED_SEQ 16942675LU


#include "test_obc.h"


struct msp_response simulate_loop(msp_link_t *link);

unsigned char test_buf[TEST_MTU + 5];
unsigned char test_storage[8192];
msp_link_t test_link;

static unsigned long seq = 0;

void test(void)
{
	struct msp_response r;
	float progress;
	unsigned long next_offset;

	/* Make sure that the size of a long is exactly 4 bytes in size. */
	test_precondition(sizeof(unsigned long) == 4, "Size of long must be 4 bytes in size");

	test_link = msp_create_link(0x11, msp_seqflags_init(), test_buf, TEST_MTU);
	test_assert(!msp_is_active(&test_link), "Link should not be active initially");

	/* Set previous transaction_id to 1 */
	msp_seqflags_set(&test_link.flags, MSP_OP_REQ_PAYLOAD, 1);

	r = msp_start_transaction(&test_link, MSP_OP_REQ_PAYLOAD, 0);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 0, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "Transaction start should be OK");

	progress = 0.0f;
	while (msp_is_active(&test_link)) {
		next_offset = msp_next_data_offset(&test_link);
		if (((float) next_offset / (float) DATA_SIZE) >= progress) {
			DEBUG_EXEC(fprintf(stderr, "progress: %.02f\n", 100*progress));
			progress += 0.1f;
		}
		r = simulate_loop(&test_link);

		test_assert(seq <= EXPECTED_SEQ, "Processed more frames than expected.");
		test_checkpoint(); /* Do not continue if any assertions failed */
	}
	/* Transaction should be successful */
	test_assert(r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL, "");

	/* Post transaction check */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");

	DEBUG_EXEC(fprintf(stderr, "seq: %lu, expected_seq: %lu\n", seq, EXPECTED_SEQ));
	test_assert(seq == EXPECTED_SEQ, "Invalid number of frames sent");

	return;
}

struct msp_response simulate_loop(msp_link_t *link)
{
	struct msp_response r;
	unsigned long len;

	switch (msp_next_action(link)) {
	case MSP_LINK_ACTION_TX_HEADER:
		r = msp_send_header_frame(link);
		break;
	case MSP_LINK_ACTION_RX_HEADER:
		r = msp_recv_header_frame(link);
		break;
	case MSP_LINK_ACTION_TX_DATA:
		test_assert(0, "unexpected action");
		break;
	case MSP_LINK_ACTION_RX_DATA:
		r = msp_recv_data_frame(link, test_storage, &len);
		break;
	default:
		break;
	}

	return r;
}


static unsigned long prev_id = 0;
static unsigned long current_id = 0;
static unsigned long transaction_id = 0;
static unsigned long sent_data = 0;

int msp_i2c_write(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	unsigned long fcs;
	unsigned char pseudo_header;
	pseudo_header = (slave_address << 1);
	fcs = msp_crc32(&pseudo_header, 1, 0);
	test_assert(slave_address == 0x11, "Value of slave_address in msp_i2c_write");
	test_assert(size == 9, "only header frames should be sent");

	if (seq == 0) {
		/* Request header */
		test_assert(data[0] == (MSP_OP_REQ_PAYLOAD), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
	} else if (sent_data != DATA_SIZE) {
		/* F-ACK */
		test_assert(data[0] == (MSP_OP_F_ACK | (prev_id << 7)), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
	} else {
		/* T-ACK */
		test_assert(data[0] == (MSP_OP_T_ACK | (transaction_id << 7)), "");
		test_assert(msp_from_bigendian32(data + 1) == 0, "DL = 0");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");

		test_assert(sent_data == DATA_SIZE, "make sure that all data has been sent");
	}
	
	seq++;
	return 0;
}
int msp_i2c_read(unsigned long slave_address, unsigned char *data, unsigned long size)
{
	unsigned long fcs, remaining_data;
	unsigned char pseudo_header;
	pseudo_header = (slave_address << 1) | 0x01;
	fcs = msp_crc32(&pseudo_header, 1, 0);
	test_assert(slave_address == 0x11, "Value of slave_address in msp_i2c_read");
	test_assert(sent_data < DATA_SIZE, "");

	if (seq == 1) {
		test_assert(size == 9, "size of header frame should be 9 bytes");
		current_id = 0;
		transaction_id = 0;
		sent_data = 0;
		prev_id = 0;

		data[0] = MSP_OP_EXP_SEND | (transaction_id << 7);
		msp_to_bigendian32(data + 1, DATA_SIZE);
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
	} else {
		remaining_data = DATA_SIZE - sent_data;
		test_assert(((size-5) == TEST_MTU) || ((size-5) == remaining_data), "size of requested data frame");

		data[0] = MSP_OP_DATA_FRAME | (current_id << 7);
		/* Ignore what is in the data field */
		fcs = msp_crc32(data, size - 4, fcs);
		msp_to_bigendian32(data + size - 4, fcs);

		sent_data += (size - 5);
	}

	prev_id = current_id;
	current_id ^= 1;

	seq++;
	return 0;
}

