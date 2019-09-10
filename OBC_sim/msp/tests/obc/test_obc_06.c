/*
 * MSP OBC Test 06
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

	/* Initialize previous transaction_id to 0 */
	msp_seqflags_set(&test_link.flags, MSP_OP_SEND_TIME, 0);



	/* Start 1st transaction */
	r = msp_start_transaction(&test_link, MSP_OP_SEND_TIME, 4);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 0, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "Transaction start should be OK");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	test_storage[0] = 0x11;
	test_storage[1] = 0x22;
	test_storage[2] = 0x33;
	test_storage[3] = 0x44;

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

	/* Post transaction check 1 */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");
	test_assert(msp_seqflags_get(&test_link.flags, MSP_OP_SEND_TIME) == 1, "");
	test_assert(seq == 4, "4 frames should've been sent after the 1st transaction");



	/* Start 2nd transaction */
	r = msp_start_transaction(&test_link, MSP_OP_SEND_TIME, 4);
	test_assert(msp_is_active(&test_link), "Link should be active after starting transaction");
	test_assert(seq == 4, "No I2C transmission should be made on transaction start");
	test_assert(r.status == MSP_RESPONSE_OK, "Transaction start should be OK");
	DEBUG_EXEC(if (r.status != MSP_RESPONSE_OK) print_response(r));

	test_storage[0] = 0xAA;
	test_storage[1] = 0xBB;
	test_storage[2] = 0xCC;
	test_storage[3] = 0xDD;

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

	/* Post transaction check 2 */
	test_assert(!msp_is_active(&test_link), "");
	test_assert(msp_next_action(&test_link) == MSP_LINK_ACTION_DO_NOTHING, "");
	test_assert(msp_seqflags_get(&test_link.flags, MSP_OP_SEND_TIME) == 0, "");
	test_assert(seq == 8, "8 frames should've been sent in total after both transactions");

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
	case 0: /* 1st send header */
		test_assert(data[0] == (MSP_OP_SEND_TIME | 0x80), "");
		test_assert(msp_from_bigendian32(data + 1) == 4, "DL = 4");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 2: /* 1st data frame */
		test_assert(data[0] == MSP_OP_DATA_FRAME, "");
		test_assert(data[1] == 0x11, "");
		test_assert(data[2] == 0x22, "");
		test_assert(data[3] == 0x33, "");
		test_assert(data[4] == 0x44, "");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 4: /* 2nd send header */
		test_assert(data[0] == MSP_OP_SEND_TIME, "");
		test_assert(msp_from_bigendian32(data + 1) == 4, "DL = 4");
		fcs = msp_crc32(data, 5, fcs);
		test_assert(fcs == msp_from_bigendian32(data + 5), "");
		break;
	case 6: /* 2nd data frame */
		test_assert(data[0] == (MSP_OP_DATA_FRAME | 0x80), "");
		test_assert(data[1] == 0xAA, "");
		test_assert(data[2] == 0xBB, "");
		test_assert(data[3] == 0xCC, "");
		test_assert(data[4] == 0xDD, "");
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
	case 1: /* 1st F_ACK */
		test_assert(size == 9, "");

		data[0] = MSP_OP_F_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 3: /* 1st T_ACK */
		test_assert(size == 9, "");

		data[0] = MSP_OP_T_ACK | 0x80;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 5: /* 2nd F_ACK */
		test_assert(size == 9, "");

		data[0] = MSP_OP_F_ACK;
		msp_to_bigendian32(data + 1, 0); /* DL = 0 */
		fcs = msp_crc32(data, 5, fcs);
		msp_to_bigendian32(data + 5, fcs);
		break;
	case 7: /* 2nd T_ACK */
		test_assert(size == 9, "");

		data[0] = MSP_OP_T_ACK;
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


