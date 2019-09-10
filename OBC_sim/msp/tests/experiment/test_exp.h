/*
 * test_exp.h
 * Author: John Wikman
 *
 * Include all the headers that an experiment needs to run.
 */

#ifndef TEST_EXP_H
#define TEST_EXP_H

#include <stdio.h>

#include <msp_exp.h>

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

#endif
