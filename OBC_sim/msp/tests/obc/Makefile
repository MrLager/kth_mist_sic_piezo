# Makefile for testing the OBC side of the MSP implementation

CC=gcc
CFLAGS=-ansi -Wall -pedantic
CFLAGS+=-DVERBOSE
#CFLAGS+=-DMSP_OBC_DEBUG

MSPDIR=../../target
MSPOBC-OBJ-FILES=${MSPDIR}/msp_crc.o
MSPOBC-OBJ-FILES+=${MSPDIR}/msp_endian.o
MSPOBC-OBJ-FILES+=${MSPDIR}/msp_seqflags.o
MSPOBC-OBJ-FILES+=${MSPDIR}/msp_obc_link.o
MSPOBC-OBJ-FILES+=${MSPDIR}/msp_obc_frame.o
MSPOBC-C-FILES=${MSPDIR}/msp_crc.c
MSPOBC-C-FILES+=${MSPDIR}/msp_endian.c
MSPOBC-C-FILES+=${MSPDIR}/msp_seqflags.c
MSPOBC-C-FILES+=${MSPDIR}/msp_obc_link.c
MSPOBC-C-FILES+=${MSPDIR}/msp_obc_frame.c

TESTFLAGS=$(CFLAGS) -I$(MSPDIR)
C-TESTFLAGS=-I$(MSPDIR) -DVERBOSE

TESTS= test00 test01 test02 test03 test04 test05 test06 test07 test08 test09
TESTS+=test10 test11 test12 test13 test14 test15
OUTFILES= test00.out test01.out test02.out test03.out test04.out test05.out test06.out test07.out test08.out test09.out
OUTFILES+=test10.out test11.out test12.out test13.out test14.out test15.out
TESTS32= test32_00 test32_01
OUTFILES32= test32_00.out test32_01.out

.PHONY: all build configure clean run test $(TESTS)

all:
	@echo "'make all' has no function. You need to specify a specific test."


build: configure $(TESTS)
build32: configure $(TESTS32)

configure:
	@python ../../conf.py obc --driver=due

test%.out: FORCE
	@./$@
FORCE:

run: $(OUTFILES)
test: build run

run32: $(OUTFILES32)
test32: build32 run32

test00: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=0 -DTESTNAME='"Basic Functionality"' -o test00.out test_obc_00.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test01: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=1 -DTESTNAME='"Specification Example 1"' -o test01.out test_obc_01.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test02: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=2 -DTESTNAME='"Specification Example 2"' -o test02.out test_obc_02.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test03: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=3 -DTESTNAME='"Specification Example 3"' -o test03.out test_obc_03.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test04: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=4 -DTESTNAME='"Specification Example 4"' -o test04.out test_obc_04.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test05: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=5 -DTESTNAME='"Specification Example 5"' -o test05.out test_obc_05.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test06: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=6 -DTESTNAME='"Specification Example 6"' -o test06.out test_obc_06.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test07: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=7 -DTESTNAME='"Specification Example 7"' -o test07.out test_obc_07.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test08: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=8 -DTESTNAME='"Specification Example 8"' -o test08.out test_obc_08.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test09: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=9 -DTESTNAME='"Specification Example 9"' -o test09.out test_obc_09.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test10: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=10 -DTESTNAME='"Specification Example 10"' -o test10.out test_obc_10.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test11: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=11 -DTESTNAME='"Transactions with 1 byte of data"' -o test11.out test_obc_11.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test12: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=12 -DTESTNAME='"Transactions with custom system command opcodes"' -o test12.out test_obc_12.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test13: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=13 -DTESTNAME='"Transactions with custom request opcodes"' -o test13.out test_obc_13.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test14: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=14 -DTESTNAME='"Transactions with custom send opcodes"' -o test14.out test_obc_14.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test15: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=15 -DTESTNAME='"Receiving busy frames from experiment"' -o test15.out test_obc_15.c test_obc_main.c $(MSPOBC-OBJ-FILES)


# 32-bit test cases below this point
test32_00: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=0 -DTESTNAME='"(32-bit test) Sending 4GB of data"' -o test32_00.out test32_obc_00.c test_obc_main.c $(MSPOBC-OBJ-FILES)

test32_01: $(MSPOBC-OBJ-FILES)
	$(CC) $(TESTFLAGS) -DTESTNO=1 -DTESTNAME='"(32-bit test) Receiving 4GB of data"' -o test32_01.out test32_obc_01.c test_obc_main.c $(MSPOBC-OBJ-FILES)


clean:
	rm -f $(MSPOBC-OBJ-FILES)
	rm -f $(OUTFILES)
	rm -f $(OUTFILES32)

