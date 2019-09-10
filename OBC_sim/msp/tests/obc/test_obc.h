/*
 * test_obc.h
 * Author: John Wikman
 *
 * Include all the common features that an OBC test needs to run.
 */

#ifndef TEST_OBC_H
#define TEST_OBC_H

#include <stdio.h>

#include <msp_obc.h>

void test(void);
void _test_assert(int assertion, const char *assert_string, const char *description, const char *filename, int line);
void test_precondition(int assertion, const char *description);
void test_checkpoint(void);

#define test_assert(assertion, msg) _test_assert(assertion, #assertion, msg, __FILE__, __LINE__)

#ifdef VERBOSE
#define DEBUG_EXEC(x) x
#else
#define DEBUG_EXEC(X)
#endif

void print_response(struct msp_response r);

#endif
