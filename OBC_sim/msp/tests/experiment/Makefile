# Makefile for testing the experiment side of the MSP implementation
CC=gcc
CFLAGS=-ansi -Wall -pedantic
CFLAGS+=-DVERBOSE
#CFLAGS+=-DMSP_EXP_DEBUG

MSPDIR=../../target
MSPEXP-OBJ-FILES=${MSPDIR}/msp_crc.o
MSPEXP-OBJ-FILES+=${MSPDIR}/msp_endian.o
MSPEXP-OBJ-FILES+=${MSPDIR}/msp_exp_callback.o
MSPEXP-OBJ-FILES+=${MSPDIR}/msp_exp_state.o
MSPEXP-OBJ-FILES+=${MSPDIR}/msp_exp_frame.o
MSPEXP-OBJ-FILES+=${MSPDIR}/msp_seqflags.o
MSPEXP-C-FILES=${MSPDIR}/msp_crc.c
MSPEXP-C-FILES+=${MSPDIR}/msp_endian.c
MSPEXP-C-FILES+=${MSPDIR}/msp_exp_callback.c
MSPEXP-C-FILES+=${MSPDIR}/msp_exp_state.c
MSPEXP-C-FILES+=${MSPDIR}/msp_exp_frame.c
MSPEXP-C-FILES+=${MSPDIR}/msp_seqflags.c

TESTFLAGS=$(CFLAGS) -I$(MSPDIR)
C-TESTFLAGS=-I$(MSPDIR)

TESTS= test00 test01 test02 test03 test04 test05 test06 test07 test08 test09
TESTS+=test10 test11 test12 test13 test14 test15 test16 test17
OUTFILES= test00.out test01.out test02.out test03.out test04.out test05.out test06.out test07.out test08.out test09.out
OUTFILES+=test10.out test11.out test12.out test13.out test14.out test15.out test16.out test17.out
TESTS32= test32_00 test32_01
OUTFILES32= test32_00.out test32_01.out

.PHONY: all build configure clean run test $(TESTS)

all:
	@echo "'make all' has no function. You need to specify a specific test."


build: configure $(TESTS)
build32: configure $(TESTS32)

configure:
	@python ../../conf.py experiment --addr=0x11 --mtu=507

test%.out: FORCE
	@./$@
FORCE:

run: $(OUTFILES)
test: build run

run32: $(OUTFILES32)
test32: build32 run32

test00: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=0 -DTESTNAME='"FCS Calculation"' -o test00.out test_exp_00.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test01: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=1 -DTESTNAME='"Specification Example 1"' -o test01.out test_exp_01.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test02: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=2 -DTESTNAME='"Specification Example 2"' -o test02.out test_exp_02.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test03: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=3 -DTESTNAME='"Specification Example 3"' -o test03.out test_exp_03.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test04: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=4 -DTESTNAME='"Specification Example 4"' -o test04.out test_exp_04.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test05: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=5 -DTESTNAME='"Specification Example 5"' -o test05.out test_exp_05.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test06: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=6 -DTESTNAME='"Specification Example 6"' -o test06.out test_exp_06.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test07: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=7 -DTESTNAME='"Specification Example 7"' -o test07.out test_exp_07.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test08: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=8 -DTESTNAME='"Specification Example 8"' -o test08.out test_exp_08.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test09: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=9 -DTESTNAME='"Specification Example 9"' -o test09.out test_exp_09.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test10: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=10 -DTESTNAME='"Specification Example 10"' -o test10.out test_exp_10.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test11: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=11 -DTESTNAME='"Specification Example 11"' -o test11.out test_exp_11.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test12: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=12 -DTESTNAME='"Transactions with 1 byte of data"' -o test12.out test_exp_12.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test13: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=13 -DTESTNAME='"Transactions with custom system command opcodes"' -o test13.out test_exp_13.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test14: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=14 -DTESTNAME='"Transactions with custom request opcodes"' -o test14.out test_exp_14.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test15: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=15 -DTESTNAME='"Transactions with custom send opcodes"' -o test15.out test_exp_15.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test16: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=16 -DTESTNAME='"Sequence flags for custom opcodes"' -o test16.out test_exp_16.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test17:
	$(CC) $(C-TESTFLAGS) -std=gnu99 -Wall -pedantic -pthread -DTESTNO=17 -DTESTNAME='"Test busy behavior"' -o test17.out test_exp_17.c test_exp_main.c $(MSPEXP-C-FILES)


# 32-bit test cases below this point
test32_00: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=0 -DTESTNAME='"(32-bit test) Receiving 4GB of data from OBC"' -o test32_00.out test32_exp_00.c test_exp_main.c $(MSPEXP-OBJ-FILES)

test32_01: $(MSPEXP-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=1 -DTESTNAME='"(32-bit test) Sending 4GB of data from OBC"' -o test32_01.out test32_exp_01.c test_exp_main.c $(MSPEXP-OBJ-FILES)

clean:
	rm -f $(MSPEXP-OBJ-FILES)
	rm -f $(OUTFILES)
	rm -f $(OUTFILES32)
