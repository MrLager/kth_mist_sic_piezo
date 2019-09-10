/*
 * MSP OBC Test Main
 * Author: John Wikman
 */

#ifndef TESTNO
#error TESTNO not defined. Needs a test number.
#endif

#ifndef TESTNAME
#error TESTNAME not defined. A test needs a name.
#endif

#include <stdlib.h>
#include <stdio.h>

#include "test_obc.h"

static int test_failed = 0;
void _test_assert(int assertion, const char *assert_string, const char *description, const char *filename, int line)
{
	/* If the assertion is true, then we report no error. */
	if (assertion)
		return;

	if (!test_failed) {
		fprintf(stderr, "\033[1;31m[OBC Test %02d] FAILED\033[0m (%s)\n", TESTNO, TESTNAME);
		test_failed = 1;
	}
	fprintf(stderr, " - (%s:%d)\n", filename, line);
	if (description[0]) {
		/* The first case for when we have test_assert(0, "...") */
		if (assert_string[0] == '0' && assert_string[1] == 0)
			fprintf(stderr, "       %s\n", description);
		else
			fprintf(stderr, "       %s: %s\n", description, assert_string);
	} else {
		fprintf(stderr, "       %s\n", assert_string);
	}
}

void test_precondition(int assertion, const char *description)
{
	if (assertion)
		return;

	fprintf(stderr, "\033[1;31m[OBC Test %02d] PRECONDITION FAIL\033[0m (%s): %s\n", TESTNO, TESTNAME, description);
	exit(1);
}

/*
 * "Flushes" the test. Exits the test if one of the assertions has failed.
 * Should only be used if it is not safe to continue with the test after a
 * certain point.
 */
void test_checkpoint(void)
{
	if (test_failed)
		exit(1);
}

void print_response(struct msp_response r) {
	switch (r.status) {
	case MSP_RESPONSE_OK:
		printf("MSP_RESPONSE_OK\n");
		break;
	case MSP_RESPONSE_BUSY:
		printf("MSP_RESPONSE_BUSY\n");
		break;
	case MSP_RESPONSE_TRANSACTION_SUCCESSFUL:
		printf("MSP_RESPONSE_TRANSACTION_SUCCESSFUL\n");
		break;
	case MSP_RESPONSE_TRANSACTION_ABORTED:
		printf("MSP_RESPONSE_TRANSACTION_ABORTED\n");
		break;
	case MSP_RESPONSE_ERROR:
		printf("MSP_RESPONSE_ERROR: %d\n", r.error_code);
		break;
	default:
		break;
	}
}

int main()
{
	test();

	if (test_failed) {
		return 1;
	} else {
		fprintf(stderr, "\033[0;32m[OBC Test %02d] Passed\033[0m (%s)\n", TESTNO, TESTNAME);
		return 0;
	}
}
