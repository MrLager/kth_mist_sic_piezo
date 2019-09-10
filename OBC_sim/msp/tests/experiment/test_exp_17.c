/*
 * MSP Experiment Test 17
 * Author: John Wikman
 */

#include "test_exp.h"

#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#if !_POSIX_BARRIERS
#error "Test needs POSIX barriers to run"
#endif

static volatile unsigned int seq = 0;

static pthread_barrier_t sync1, sync2, sync3, sync4;

/* Just in case we have a deadlock or something */
void test_timeout(int signal)
{
	fprintf(stderr, "\033[1;31m[Exp Test %02d] TIMED OUT\033[0m\n", TESTNO);
	exit(0);
}

void *concurrent_test(void *args)
{
	unsigned char buf[MSP_EXP_MAX_FRAME_SIZE];
	unsigned char send_pseudo_header, recv_pseudo_header;
	unsigned long fcs, len;
	int code;

	/* Set up both possible pseudo headers */
	recv_pseudo_header = MSP_EXP_ADDR << 1; /* Write bit set to 0 */
	send_pseudo_header = (MSP_EXP_ADDR << 1) | 0x01; /* Write bit set to 1 */

	/* Wait for main thread to call recv_callback */
	pthread_barrier_wait(&sync1);

	/* At this point, the experiment should have received a request frame from
	 * the OBC, so try to read a response. However, since we are still in the
	 * handler function in the main thread, this should be a busy frame. */
	code = msp_send_callback(buf, &len);
	test_assert(code == MSP_EXP_ERR_IS_BUSY, "should be busy");
	test_assert(buf[0] == MSP_OP_EXP_BUSY, "");
	test_assert(msp_from_bigendian32(buf + 1) == 0, "DL = 0");
	fcs = msp_crc32(&send_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	test_assert(fcs == msp_from_bigendian32(buf + 5), "FCS check");

	/* So maybe the OBC would try to resend the header frame in this situation... */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf + 1, 0);
	fcs = msp_crc32(&recv_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	msp_to_bigendian32(buf + 5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == MSP_EXP_ERR_IS_BUSY, "should be busy");

	/* Ok, so lets wait for the experiment to finish... */
	pthread_barrier_wait(&sync2);

	/* Wait until experiment calls send_callback */
	pthread_barrier_wait(&sync3);

	/* OBC has not received a frame yet, so maybe it resends the F_ACK */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf + 1, 0);
	fcs = msp_crc32(&recv_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	msp_to_bigendian32(buf + 5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == MSP_EXP_ERR_IS_BUSY, "should be busy");

	/* Now lets try to read the data frame again... */
	code = msp_send_callback(buf, &len);
	test_assert(code == MSP_EXP_ERR_IS_BUSY, "should be busy");
	test_assert(buf[0] == MSP_OP_EXP_BUSY, "");
	test_assert(msp_from_bigendian32(buf + 1) == 0, "DL = 0");
	fcs = msp_crc32(&send_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	test_assert(fcs == msp_from_bigendian32(buf + 5), "FCS check");

	/* Finally, let the experiment send the data frame and kill the thread. */
	pthread_barrier_wait(&sync4);

	return NULL;
}

void test(void)
{
	/* Test calling msp_recv_callback from two places at the same time. Same for msp_send_callback. */
	unsigned char buf[MSP_EXP_MAX_FRAME_SIZE];
	unsigned char send_pseudo_header, recv_pseudo_header;
	unsigned long fcs, len;

	pthread_t thread;
	int code;

	/* Allow test to run for at most 10 seconds */
	signal(SIGALRM, test_timeout);
	alarm(10);

	code = 0;
	code |= pthread_barrier_init(&sync1, NULL, 2); /* each barrier should wait for 2 threads */
	code |= pthread_barrier_init(&sync2, NULL, 2);
	code |= pthread_barrier_init(&sync3, NULL, 2);
	code |= pthread_barrier_init(&sync4, NULL, 2);
	if (code) {
		test_assert(0, "Error setting up pthread barriers");
		return;
	}
	code = pthread_create(&thread, NULL, concurrent_test, NULL);
	if (code) {
		test_assert(0, "Error setting up concurrent test thread");
		return;
	}
	/* Set up both possible pseudo headers */
	recv_pseudo_header = MSP_EXP_ADDR << 1;
	send_pseudo_header = (MSP_EXP_ADDR << 1) | 0x01;

	/* OBC Requests some data */
	buf[0] = MSP_OP_REQ_PAYLOAD;
	msp_to_bigendian32(buf + 1, 0);
	fcs = msp_crc32(&recv_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	msp_to_bigendian32(buf + 5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "No error should occur after other concurrent tasks have called the receive callback.");

	/* Send header */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "No error should be reported here");
	test_assert(buf[0] == MSP_OP_EXP_SEND, "");
	test_assert(msp_from_bigendian32(buf + 1) == 4, "DL = 4");
	fcs = msp_crc32(&send_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	test_assert(msp_from_bigendian32(buf + 5) == fcs, "check FCS");

	/* Receive F_ACK */
	buf[0] = MSP_OP_F_ACK;
	msp_to_bigendian32(buf + 1, 0);
	fcs = msp_crc32(&recv_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	msp_to_bigendian32(buf + 5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "No error should be reported here");

	/* Send the data frame */
	code = msp_send_callback(buf, &len);
	test_assert(code == 0, "No error should occur after other concurrent tasks have called the send callback.");
	test_assert(buf[0] == (MSP_OP_DATA_FRAME | 0x80), "");
	test_assert(buf[1] == 0x11, "");
	test_assert(buf[2] == 0x33, "");
	test_assert(buf[3] == 0x55, "");
	test_assert(buf[4] == 0x22, "");
	fcs = msp_crc32(&send_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	test_assert(msp_from_bigendian32(buf + 5) == fcs, "check FCS");

	/* Receive T_ACK */
	buf[0] = MSP_OP_T_ACK;
	msp_to_bigendian32(buf + 1, 0);
	fcs = msp_crc32(&recv_pseudo_header, 1, 0);
	fcs = msp_crc32(buf, 5, fcs);
	msp_to_bigendian32(buf + 5, fcs);
	code = msp_recv_callback(buf, 9);
	test_assert(code == 0, "No error should be reported here");

	pthread_join(thread, NULL);

	test_assert(seq == 3, "Only three handlers should have been called.");

	return;
}


void msp_exprecv_data(unsigned char opcode, const unsigned char *buf, unsigned long len, unsigned long offset)
{
	test_assert(0, "msp_exprecv_data should be unreachable");
}
void msp_expsend_data(unsigned char opcode, unsigned char *buf, unsigned long len, unsigned long offset)
{
	seq++;

	/* Wait for the other thread to be ready to call the callback functions */
	pthread_barrier_wait(&sync3);

	/* Wait until the callback function have been called from the other thread */
	pthread_barrier_wait(&sync4);

	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "");
	test_assert(len == 4, "");
	test_assert(offset == 0, "");

	buf[0] = 0x11;
	buf[1] = 0x33;
	buf[2] = 0x55;
	buf[3] = 0x22;
}

void msp_exprecv_start(unsigned char opcode, unsigned long len)
{
	test_assert(0, "msp_exprecv_start should be unreachable");
}
void msp_expsend_start(unsigned char opcode, unsigned long *len)
{
	seq++;

	/* Wait for other thread to be ready to call the callback functions */
	pthread_barrier_wait(&sync1);

	/* Wait until the callback functions have been called */
	pthread_barrier_wait(&sync2);

	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "");

	*len = 4;
}

void msp_exprecv_complete(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_complete should be unreachable");
}
void msp_expsend_complete(unsigned char opcode)
{
	seq++;

	test_assert(opcode == MSP_OP_REQ_PAYLOAD, "");
}

void msp_exprecv_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_exprecv_error should be unreachable");
}
void msp_expsend_error(unsigned char opcode, int error)
{
	test_assert(0, "msp_expsend_error should be unreachable");
}

void msp_exprecv_syscommand(unsigned char opcode)
{
	test_assert(0, "msp_exprecv_syscommand should be unreachable");
}
