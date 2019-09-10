/*
 * obcsim_transactions.cpp
 */

#include "Arduino.h"
#include "obcsim_configuration.hpp"
#include "obcsim_transactions.hpp"
#include "msp_obc.h"


// Increase buffer size if MTU is larger than 4096
static unsigned char request_buffer[REQUEST_BUFFER_SIZE + 1];
static bool request_buffer_overflow = false;

static unsigned char *send_buffer;

static struct msp_response process(msp_link_t *lnk);
static void print_response(struct msp_response r, const char *msg);

static void print_data(unsigned char *data, unsigned long len, PrintStyle pstyle);
static void print_bytes(unsigned char *data, unsigned long len);
static void print_bits(unsigned char *data, unsigned long len);
static void print_string(unsigned char *data, unsigned long len);

void invoke_syscommand(msp_link_t *lnk, unsigned char opcode)
{
	struct msp_response r;

	r = msp_start_transaction(lnk, opcode, 0);
	while (msp_is_active(lnk)) {
		r = process(lnk);
		if (msp_error_count(lnk) >= MSP_ERROR_THRESHOLD) {
			Serial.println(F("(error threshold reached)"));
			r = msp_abort_transaction(lnk);
		}
	}
	print_response(r, "");
}

void invoke_send(msp_link_t *lnk, unsigned char opcode, unsigned char *data, unsigned long len, PrintStyle pstyle)
{
	struct msp_response r;

	send_buffer = data;

	r = msp_start_transaction(lnk, opcode, len);
	while (msp_is_active(lnk)) {
		r = process(lnk);
		if (msp_error_count(lnk) >= MSP_ERROR_THRESHOLD) {
			Serial.println(F("(error threshold reached)"));
			r = msp_abort_transaction(lnk);
		}
	}
	print_response(r, "");
	if (r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL) {
		if (pstyle != NONE)
			Serial.print(F("\nSent "));
		print_data(data, len, pstyle);
	}
}

void invoke_send_repeat(msp_link_t *lnk, unsigned char opcode, unsigned char value, unsigned long times, PrintStyle pstyle)
{
	struct msp_response r;
	unsigned long i, len;

	// Re-use the request buffer to store the data to be repeated
	for (i = 0; i < EXP_MTU; ++i)
		request_buffer[i] = value;

	r = msp_start_transaction(lnk, opcode, times);
	while (msp_is_active(lnk)) {
		len = msp_next_data_length(lnk);

		switch (msp_next_action(lnk)) {
		case MSP_LINK_ACTION_TX_HEADER:
			r = msp_send_header_frame(lnk);
			break;
		case MSP_LINK_ACTION_RX_HEADER:
			r = msp_recv_header_frame(lnk);
			break;
		case MSP_LINK_ACTION_TX_DATA:
			r = msp_send_data_frame(lnk, request_buffer, len);
			break;
		default:
			// this should never be reached, so then we abort if this is the case
			r = msp_abort_transaction(lnk);
			break;
		}

		if (msp_error_count(lnk) >= MSP_ERROR_THRESHOLD) {
			Serial.println(F("(error threshold reached)"));
			r = msp_abort_transaction(lnk);
		}
	}
	print_response(r, "");
	if (r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL) {
		if (pstyle != NONE) {
			Serial.print(F("\nSent a sequence of "));
			Serial.print(times, DEC);
			Serial.print(F(" bytes with value 0x"));
			Serial.print(value, HEX);
			Serial.println(F("."));
		}
	}
}

void invoke_request(msp_link_t *lnk, unsigned char opcode, PrintStyle pstyle)
{
	struct msp_response r;

	request_buffer_overflow = false;
	r = msp_start_transaction(lnk, opcode, 0);
	while (msp_is_active(lnk)) {
		r = process(lnk);
		if (msp_error_count(lnk) >= MSP_ERROR_THRESHOLD || request_buffer_overflow) {
			Serial.println(F("(error threshold reached)"));
			r = msp_abort_transaction(lnk);
		}
	}
	print_response(r, "");
	if (request_buffer_overflow) {
		Serial.print(F("Buffer Overflow (Received more than "));
		Serial.print(REQUEST_BUFFER_SIZE, DEC);
		Serial.println(F(" bytes of data.)"));
	} else if (r.status == MSP_RESPONSE_TRANSACTION_SUCCESSFUL) {
		if (pstyle != NONE)
			Serial.print(F("\nReceived "));
		print_data(request_buffer, r.len, pstyle);
	}
}


/* Takes an action in an MSP transaction */
static struct msp_response process(msp_link_t *lnk)
{
	struct msp_response r;
	unsigned long len, offset;
	
	len = msp_next_data_length(lnk);
	offset = msp_next_data_offset(lnk);

	switch (msp_next_action(lnk)) {
	case MSP_LINK_ACTION_TX_HEADER:
		r = msp_send_header_frame(lnk);
		break;
	case MSP_LINK_ACTION_RX_HEADER:
		r = msp_recv_header_frame(lnk);
		break;
	case MSP_LINK_ACTION_TX_DATA:
		r = msp_send_data_frame(lnk, send_buffer + offset, len);
		break;
	case MSP_LINK_ACTION_RX_DATA:
		/* Make sure that we don't receive too much data. In that case we mark
		 * buffer as overflowed. */
		if ((len + offset) >= REQUEST_BUFFER_SIZE) {
			offset = 0;
			request_buffer_overflow = true;
		}

		r = msp_recv_data_frame(lnk, request_buffer + offset, &len);
		break;
	default:
		break;
	}

	return r;
}

/* A print function for printing out an MSP response */
static void print_response(struct msp_response r, const char *msg)
{
	switch (r.status) {
	case MSP_RESPONSE_BUSY:
		Serial.println(F("response: BUSY"));
		break;
	case MSP_RESPONSE_ERROR:
		Serial.print(F("response: ERROR: "));
		Serial.println(r.error_code, DEC);
		break;
	case MSP_RESPONSE_TRANSACTION_SUCCESSFUL:
		Serial.println(F("response: TRANSACTION SUCCESSFUL"));
		break;
	case MSP_RESPONSE_TRANSACTION_ABORTED:
		Serial.println(F("response: TRANSACTION ABORTED"));
		break;
	case MSP_RESPONSE_OK:
		Serial.println(F("response: OK"));
		break;
	}

	if (msg[0])
		Serial.println(msg);
}


/* Functions for printing data */
static void print_data(unsigned char *data, unsigned long len, PrintStyle pstyle)
{
	switch (pstyle) {
	case BYTES:
		print_bytes(data, len);
		break;
	case BITS:
		print_bits(data, len);
		break;
	case STRING:
		print_string(data, len);
		break;
	}
}

static void print_bytes(unsigned char *data, unsigned long len)
{
	char buf[5];
	unsigned long i;
	const unsigned long cols = 8;

	Serial.print(F("bytes:"));
	for (i = 0; i < len; ++i) {
		if (i % cols == 0)
			Serial.print("\n");
		else
			Serial.print(" ");

		snprintf(buf, 5, "0x%02X", data[i]);
		buf[4] = 0;
		Serial.print(buf);
	}
	Serial.print("\n");
}

static void print_bits(unsigned char *data, unsigned long len)
{
	char buf[11];
	unsigned long i, j;
	const unsigned long cols = 8;

	Serial.println(F("bits (most significant bit to the left):"));
	for (i = 0; i < len; i += cols) {
		// Print byte indices first
		for (j = i; (j < i + cols) && (j < len); ++j) {
			snprintf(buf, 11, "Byte %-5d", j);
			buf[10] = 0;
			Serial.print(buf);
		}
		Serial.print("\n");

		// Now print the bits in each byte
		for (j = i; (j < i + cols) && (j < len); ++j) {
			// Reverse order to print most significant bit first
			for (int k = 7; k >= 0; --k) {
				if ((1 << k) & data[j])
					buf[7-k] = '1';
				else
					buf[7-k] = '0';
			}
			buf[8] = ' ';
			buf[9] = ' ';
			buf[10] = 0;
			Serial.print(buf);
		}
		Serial.print("\n");
	}
}

static void print_string(unsigned char *data, unsigned long len)
{
	if (len == 0)
		len = 1;

	data[len-1] = 0;
	Serial.println(F("string:"));
	Serial.println((char *) data);
}

