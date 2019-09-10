# Makefile for managing the MSP examples

.PHONY: setup clean

MSP-MTU=15
MSP-ADDR=0x11

setup:
	@echo "Must specify the target platform. Use 'make setup_uno' or 'make setup_due' instead."

setup_uno:
	make clean
	python ../conf.py obc --driver=uno --lowmem
	cp ../target/* obc/
	python ../conf.py experiment --driver=uno --mtu=$(MSP-MTU) --addr=$(MSP-ADDR) --lowmem
	cp ../target/* experiment/

setup_due:
	make clean
	python ../conf.py obc --driver=due
	cp ../target/* obc/
	python ../conf.py experiment --driver=due --mtu=$(MSP-MTU) --addr=$(MSP-ADDR)
	cp ../target/* experiment/

clean:
	rm -f experiment/msp*
	rm -f obc/msp*
