# Simple Makefile for running the MSP test cases

.PHONY: all test clean

all:
	@echo "'make all' has no effect. Use 'make test' or 'make clean' instead."

build:
	@cd experiment && make build
	@cd obc && make build

testexp:
	@cd experiment && make run -k
testobc:
	@cd obc && make run -k

# -k allows us to keep running tests after one has failed. The testwrapper is
# there to make sure that the obc and experiment tests are run independently.
test: build
	@make testwrapper -k --no-print-directory

testwrapper: testexp testobc


# 32-bit variant of the same above
build32:
	@cd experiment && make build32
	@cd obc && make build32

test32exp:
	@cd experiment && make run32 -k
test32obc:
	@cd obc && make run32 -k

test32: build32
	@make test32wrapper -k --no-print-directory

test32wrapper: test32exp test32obc



clean:
	@cd experiment && make clean
	@cd obc && make clean
