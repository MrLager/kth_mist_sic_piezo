/**
 * @file      msp_obc_link.c
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Implements the core functionalities for handling transactions on
 *            the OBC side of MSP.
 */

#include <stdlib.h>

#include "msp_i2c_master.h"

#include "msp_constants.h"
#include "msp_debug.h"
#include "msp_endian.h"
#include "msp_opcodes.h"
#include "msp_seqflags.h"

#include "msp_obc_error.h"
#include "msp_obc_link.h"
#include "msp_obc_frame.h"

/* Configures the link as ready */
static void msp_set_as_ready(msp_link_t *lnk)
{
	lnk->state = MSP_LINK_STATE_READY;
	lnk->next_action = MSP_LINK_ACTION_DO_NOTHING;
	lnk->error_count = 0;

	lnk->transaction_id = 0;
	lnk->frame_id = 0;
	lnk->opcode = MSP_OP_NULL;

	lnk->total_length = 0;
	lnk->processed_length = 0;
}

static struct msp_response msp_response_error(int error_code)
{
	struct msp_response r;
	r.status = MSP_RESPONSE_ERROR;
	r.error_code = error_code;
	return r;
}

static struct msp_response msp_response_ok(void) {
	struct msp_response r;
	r.status = MSP_RESPONSE_OK;
	return r;
}

static struct msp_response msp_response_busy(void) {
	struct msp_response r;
	r.status = MSP_RESPONSE_BUSY;
	return r;
}

static struct msp_response msp_response_aborted(const msp_link_t *lnk)
{
	struct msp_response r;
	r.status = MSP_RESPONSE_TRANSACTION_ABORTED;
	r.error_code = 0;
	r.opcode = lnk->opcode;
	r.transaction_id = lnk->transaction_id;
	r.len = lnk->total_length;
	return r;
}

static struct msp_response msp_response_successful(const msp_link_t *lnk)
{
	struct msp_response r;
	r.status = MSP_RESPONSE_TRANSACTION_SUCCESSFUL;
	r.error_code = 0;
	r.opcode = lnk->opcode;
	r.transaction_id = lnk->transaction_id;
	r.len = lnk->total_length;
	return r;
}


/**
 * @brief Creates and configures a link to an experiment which has the
 *        specified properties. (Non-Blocking)
 * @param slave_address The address of the experiment.
 * @param flags The sequence flags to be used in the link. If transactions have
 *              taken place with this experiment in another link prior to this,
 *              then the sequence flags that were saved from that link should
 *              be used in this link as well.
 * @param buf A pointer to the dedicated buffer for this link. This buffer has
 *            to be allocated outside of the link. The size of this buffer
 *            be at least mtu + 5 (or 9 if mtu < 4).
 * @param mtu The MTU that will be used in the MSP communication with this
 *            experiment.
 * @return The configured link.
 *
 * Creates a link to an experiment. Does not need to be recreated when
 * communicating with the same experiment anew.
 */
msp_link_t msp_create_link(unsigned long slave_address,
						   msp_seqflags_t flags,
						   unsigned char *buf,
						   unsigned long mtu)
{
	msp_link_t lnk;

	/* Setup the persistent variables */
	lnk.slave_address = slave_address;
	lnk.buffer = buf;
	lnk.mtu = mtu;
	lnk.flags = flags;

	msp_set_as_ready(&lnk);

	return lnk;
}


/**
 * @brief Starts a transaction with the given opcode. (Non-Blocking)
 * @param lnk The link to start a transaction with.
 * @param opcode The opcode of the transaction.
 * @param len The number of bytes to send to the experiment. If the opcode is
 *            of type request or system command, this parameter must have the
 *            value 0.
 * @return An MSP response.
 *
 * This function is non-blocking as it only updates the state of the link.
 */
struct msp_response msp_start_transaction(msp_link_t *lnk,
                                          unsigned char opcode,
                                          unsigned long len)
{
	if (lnk == NULL)
		return msp_response_error(MSP_OBC_ERR_NULL_POINTER);

	if (len > MSP_MAX_DATA_LENGTH)
		return msp_response_error(MSP_OBC_ERR_INVALID_LENGTH);

	switch (MSP_OP_TYPE(opcode)) {
	case MSP_OP_TYPE_SYS:
		if (len != 0)
			return msp_response_error(MSP_OBC_ERR_LENGTH_NOT_ZERO);
		/* fall through */
	case MSP_OP_TYPE_SEND:
		lnk->state = MSP_LINK_STATE_SEND_TX_HEADER;
		lnk->next_action = MSP_LINK_ACTION_TX_HEADER;
		lnk->error_count = 0;
		lnk->transaction_id = msp_seqflags_get_next(&lnk->flags, opcode);
		lnk->frame_id = lnk->transaction_id;
		lnk->opcode = opcode;
		lnk->total_length = len;
		lnk->processed_length = 0;
		break;
	case MSP_OP_TYPE_REQ:
		if (len != 0)
			return msp_response_error(MSP_OBC_ERR_LENGTH_NOT_ZERO);

		lnk->state = MSP_LINK_STATE_REQ_TX_HEADER;
		lnk->next_action = MSP_LINK_ACTION_TX_HEADER;
		lnk->error_count = 0;
		lnk->transaction_id = 0;
		lnk->frame_id = 0;
		lnk->opcode = opcode;
		lnk->total_length = 0;
		lnk->processed_length = 0;
		break;
	default:
		return msp_response_error(MSP_OBC_ERR_INVALID_OPCODE);
	}

	return msp_response_ok();
}

/**
 * @brief Aborts an ongoing transaction. (Blocking)
 * @param lnk The link whose ongoing transaction should be aborted.
 * @return An MSP response.
 *
 * Blocking (send over I2C)
 *
 * Aborts the current transaction and sends a null frame to the experiment.
 * Even if something went wrong when trying to send the frame, the state of the
 * link will still be set to ready.
 *
 * If the link is not active in a transaction, then no null frame is sent and
 * an error response is returned.
 */
struct msp_response msp_abort_transaction(msp_link_t *lnk)
{
	int code;
	struct msp_obc_frame frame;
	unsigned long len;

	if (lnk == NULL)
		return msp_response_error(MSP_OBC_ERR_NULL_POINTER);

	/* Take no action if we already are in a transaction */
	if (lnk->state == MSP_LINK_STATE_READY)
		return msp_response_error(MSP_OBC_ERR_NOT_IN_A_TRANSACTION);

	/* Setup NULL frame */
	frame.type = MSP_OBC_FRAME_HEADER;
	frame.id = 0;
	frame.opcode = MSP_OP_NULL;
	frame.dl = 0;

	code = msp_obc_encode_frame(lnk, lnk->buffer, &len, frame);
	if (code) {
		lnk->error_count += 1;
		return msp_response_error(code);
	}

	/* Now at this point in time, we do not care about if the frame was sent
	 * over I2C or not. We set ourselves in the ready state no matter what. */

	msp_set_as_ready(lnk);

	code = msp_i2c_write(lnk->slave_address, lnk->buffer, len);
	if (code) {
		/* Do not increment the error counter here as we are aborting the
		 * transaction anyway. */
		return msp_response_error(code);
	}


	return msp_response_ok();
}

/**
 * @brief Sends a data frame in an ongoing transaction. (Blocking)
 * @param lnk The link with the ongoing transaction.
 * @param data Pointer to the data that should be inserted into the data field
 *             of the data frame.
 * @param datalen The length of the data that should be inserted into the data
 *                field of the data frame.
 * @return An MSP response.
 *
 * Blocking (send over I2C)
 *
 * Sends a data frame to the experiment with the data pointed to by the pointer
 * data and length datalen.
 *
 * This function will also update the state of the link.
 *
 * If the link is in an incorrect state or if any of the inputs are invalid
 * then no I2C transmission will be made and an error will be returned.
 */
struct msp_response msp_send_data_frame(msp_link_t *lnk,
                                        unsigned char *data,
                                        unsigned long datalen)
{
	int code;
	struct msp_obc_frame frame;
	unsigned long len;

	if (lnk == NULL || data == NULL || lnk->buffer == NULL)
		return msp_response_error(MSP_OBC_ERR_NULL_POINTER);

	if (lnk->next_action != MSP_LINK_ACTION_TX_DATA) {
		lnk->error_count += 1;
		return msp_response_error(MSP_OBC_ERR_INVALID_ACTION);
	}

	if (datalen != msp_next_data_length(lnk)) {
		lnk->error_count += 1;
		return msp_response_error(MSP_OBC_ERR_INVALID_LENGTH);
	}

	frame.type = MSP_OBC_FRAME_DATA;
	frame.id = lnk->frame_id;
	frame.opcode = MSP_OP_DATA_FRAME;
	frame.data = data;
	frame.datalen = datalen;

	code = msp_obc_encode_frame(lnk, lnk->buffer, &len, frame);
	if (code) {
		lnk->error_count += 1;
		return msp_response_error(code);
	}

	code = msp_i2c_write(lnk->slave_address, lnk->buffer, len);
	if (code) {
		lnk->error_count += 1;
		return msp_response_error(code);
	}

	/* Read acknowledge of data frame next */
	lnk->next_action = MSP_LINK_ACTION_RX_HEADER;

	return msp_response_ok();
}

/**
 * @brief Sends a header in an ongoing transaction. (Blocking)
 * @param lnk The link with the ongoing transaction.
 * @return An MSP response.
 *
 * Blocking (send over I2C)
 *
 * Sends a header frame to the experiment. The exact type of header frame that
 * is sent is determined by the state that the link is in.
 *
 * This function will also update the state of the link.
 *
 * If the link is in an incorrect state or if any of the inputs are invalid
 * then no I2C transmission will be made and an error will be returned.
 */
struct msp_response msp_send_header_frame(msp_link_t *lnk)
{
	struct msp_response r;
	int code;
	struct msp_obc_frame frame;
	unsigned long len;

	if (lnk == NULL || lnk->buffer == NULL)
		return msp_response_error(MSP_OBC_ERR_NULL_POINTER);

	if (lnk->next_action != MSP_LINK_ACTION_TX_HEADER) {
		lnk->error_count += 1;
		return msp_response_error(MSP_OBC_ERR_INVALID_ACTION);
	}

	/* Determine which type of header to send */
	switch (lnk->state) {
	case MSP_LINK_STATE_SEND_TX_HEADER:
		frame.type = MSP_OBC_FRAME_HEADER;
		frame.id = lnk->transaction_id;
		frame.opcode = lnk->opcode;
		frame.dl = lnk->total_length;
		break;
	case MSP_LINK_STATE_REQ_TX_HEADER:
		/* Send request header (DL0, FID0) */
		frame.type = MSP_OBC_FRAME_HEADER;
		frame.id = 0;
		frame.opcode = lnk->opcode;
		frame.dl = 0;
		break;
	case MSP_LINK_STATE_REQ_RX_DATA:
		frame.type = MSP_OBC_FRAME_HEADER;
		if (msp_next_data_length(lnk) == 0) {
			frame.id = lnk->transaction_id;
			frame.opcode = MSP_OP_T_ACK;
			frame.dl = 0;
			msp_debug_int("formatting T_ACK with id ", frame.id);
		} else {
			frame.id = lnk->frame_id;
			frame.opcode = MSP_OP_F_ACK;
			frame.dl = 0;
			msp_debug_int("formatting F_ACK with id ", frame.id);
		}
		break;
	default:
		msp_debug("Invalid state at 1st switch statement in msp_send_header_frame");
		return msp_response_error(MSP_OBC_ERR_INVALID_STATE);
	}

	code = msp_obc_encode_frame(lnk, lnk->buffer, &len, frame);
	if (code) {
		lnk->error_count += 1;
		return msp_response_error(code);
	}

	code = msp_i2c_write(lnk->slave_address, lnk->buffer, len);
	if (code) {
		lnk->error_count += 1;
		return msp_response_error(code);
	}

	/* Now check what the next action should be */
	if (frame.opcode == MSP_OP_T_ACK) {
		msp_seqflags_set(&lnk->flags, lnk->opcode, lnk->transaction_id);
		r = msp_response_successful(lnk);
		msp_set_as_ready(lnk);
		return r;
	}

	switch (lnk->state) {
	case MSP_LINK_STATE_SEND_TX_HEADER:
		/* Should read an ACK next */
		lnk->next_action = MSP_LINK_ACTION_RX_HEADER;
		break;
	case MSP_LINK_STATE_REQ_TX_HEADER:
		/* Should read the response header next */
		lnk->state = MSP_LINK_STATE_REQ_RX_RESPONSE;
		lnk->next_action = MSP_LINK_ACTION_RX_HEADER;
		break;
	case MSP_LINK_STATE_REQ_RX_DATA:
		/* Should read a data frame next */
		lnk->next_action = MSP_LINK_ACTION_RX_DATA;
		break;
	default:
		msp_debug("Invalid state at 2nd switch statement in msp_send_header_frame");
		lnk->error_count += 1;
		return msp_response_error(MSP_OBC_ERR_INVALID_STATE);
	}

	return msp_response_ok();
}

/**
 * @brief Reads a data frame in an ongoing transaction. (Blocking)
 * @param lnk The link with the ongoing transaction.
 * @param data Pointer to a buffer in which the data received from the data
 *             frame will be copied into.
 * @param datalen A pointer to a 32-bit unsigned integer where the size of
 *                the data field in the received data frame will be written.
 *                (I.e. the number of bytes copied over into the area pointed
 *                to by the pointer 'data'.)
 * @return An MSP response.
 *
 * Blocking (read over I2C)
 *
 * Receives a data frame from the experiment. The data that was present in the
 * data frame will be copied over into the area of memory pointed to by the
 * parameter data and the number of bytes copied will be written into the long
 * pointed to by datalen. However, if the experiment did send a header frame
 * instead, this will get interpreted accordingly as well and nothing will be
 * written to the area of memory pointed to by the parameter 'data'.
 *
 * This function will also update the state of the link.
 *
 * If the link is in an incorrect state or if any of the inputs are invalid
 * then no I2C transmission will be made and an error will be returned.
 */
struct msp_response msp_recv_data_frame(msp_link_t *lnk,
                                        unsigned char *data,
                                        unsigned long *datalen)
{
	int code;
	struct msp_response r;
	struct msp_obc_frame frame;
	unsigned long len;
	unsigned long rx_len;
	unsigned long i;

	if (lnk == NULL || data == NULL || datalen == NULL || lnk->buffer == NULL)
		return msp_response_error(MSP_OBC_ERR_NULL_POINTER);

	if (lnk->next_action != MSP_LINK_ACTION_RX_DATA) {
		lnk->error_count += 1;
		return msp_response_error(MSP_OBC_ERR_INVALID_ACTION);
	}

	len = msp_next_data_length(lnk);
	if (len == 0) {
		lnk->error_count += 1;
		return msp_response_error(MSP_OBC_ERR_INVALID_ACTION);
	}

	/* Make sure that rx_len is at least 9 so that we can decode an unexpected
	 * header frame if the experiment decides to send that */
	rx_len = len + 5;
	if (rx_len < 9)
		rx_len = 9;

	code = msp_i2c_read(lnk->slave_address, lnk->buffer, rx_len);
	if (code) {
		lnk->error_count += 1;
		return msp_response_error(code);
	}

	/* First try to decode it as a data frame */
	frame = msp_obc_decode_frame(lnk, lnk->buffer, len + 5);
	if (frame.type == MSP_OBC_FRAME_DATA) {
		/* Check that it is not a duplicate frame */
		if (frame.id != (lnk->frame_id ^ 1)) {
			lnk->error_count += 1;
			return msp_response_error(MSP_OBC_ERR_INVALID_FRAME);
		}

		/* All good, copy over the data and update link state */
		for (i = 0; i < frame.datalen; i++)
			data[i] = frame.data[i];

		*datalen = frame.datalen;

		lnk->frame_id = frame.id;
		lnk->next_action = MSP_LINK_ACTION_TX_HEADER;
		lnk->processed_length += frame.datalen;

		r.status = MSP_RESPONSE_OK;
		return r;
	}


	/* If that didn't work, we should try to decode it as a header frame */
	frame = msp_obc_decode_frame(lnk, lnk->buffer, 9);
	if (frame.type != MSP_OBC_FRAME_HEADER) {
		msp_debug("Expected data frame, got a corrupt frame");
		lnk->error_count += 1;
		lnk->next_action = MSP_LINK_ACTION_TX_HEADER;
		return msp_response_error(MSP_OBC_ERR_INVALID_FRAME);
	}
	
	msp_debug("Expected data frame, got header frame");

	/* If the frame was a NULL frame, we must abort the transaction */
	if (frame.opcode == MSP_OP_NULL) {
		r = msp_response_aborted(lnk);
		msp_set_as_ready(lnk);
		return r;
	}

	/* If it turns out that the experiment is busy, we report that */
	if (frame.opcode == MSP_OP_EXP_BUSY)
		return msp_response_busy();

	/* If we received the EXP_SEND header again, then we simply discard it as
	 * an error. */
	/*if (frame.opcode == MSP_OP_EXP_SEND && frame.id == link->transaction_id) {
		link->error_count += 1;
		link->next_action = MSP_OBC_ACTION_TX_HEADER;
		r.status = MSP_RESPONSE_ERROR;
		r.error_code = MSP_OBC_ERR_INVALID_FRAME;
		return r;
	}*/

	/* If none of the previous cases, we received some frame that we should not
	 * have received. Next we just send and ACK and hope for the best. */
	lnk->error_count += 1;
	lnk->next_action = MSP_LINK_ACTION_TX_HEADER;
	return msp_response_error(MSP_OBC_ERR_INVALID_FRAME);
}

/**
 * @brief Reads a header frame in an ongoing transaction. (Blocking)
 * @param lnk The link with the ongoing transaction.
 * @return An MSP response.
 *
 * Blocking (read over I2C)
 *
 * Receives a header frame from an experiment.
 *
 * This function will also update the state of the link.
 *
 * If the link is in an incorrect state or if any of the inputs are invalid
 * then no I2C transmission will be made and an error will be returned.
 */
struct msp_response msp_recv_header_frame(msp_link_t *lnk)
{
	int code;
	struct msp_response r;
	struct msp_obc_frame frame;
	unsigned long len;

	if (lnk == NULL || lnk->buffer == NULL)
		return msp_response_error(MSP_OBC_ERR_NULL_POINTER);

	if (lnk->next_action != MSP_LINK_ACTION_RX_HEADER) {
		lnk->error_count += 1;
		return msp_response_error(MSP_OBC_ERR_INVALID_ACTION);
	}

	code = msp_i2c_read(lnk->slave_address, lnk->buffer, 9);
	if (code) {
		lnk->error_count += 1;
		return msp_response_error(code);
	}

	frame = msp_obc_decode_frame(lnk, lnk->buffer, 9);
	if (frame.type != MSP_OBC_FRAME_HEADER) {
		lnk->error_count += 1;
		msp_debug("Expected header frame, got data frame or FCS mismatch");
		return msp_response_error(MSP_OBC_ERR_INVALID_FRAME);
	}

	if (frame.opcode == MSP_OP_NULL) {
		/* NULL frame always resets transaction */
		r = msp_response_aborted(lnk);
		msp_set_as_ready(lnk);
		msp_debug_hex("Aborting transaction with opcode: ", r.opcode);
		return r;
	}

	/* Take no further action if the experiment is busy */
	if (frame.opcode == MSP_OP_EXP_BUSY)
		return msp_response_busy();

	switch (lnk->state) {
	case MSP_LINK_STATE_SEND_TX_HEADER:
	case MSP_LINK_STATE_SEND_TX_DATA:
		/* We should get an ACK here */
		if (frame.opcode == MSP_OP_T_ACK) {
			/* Transaction Complete */
			msp_seqflags_set(&lnk->flags, lnk->opcode, lnk->transaction_id);
			r = msp_response_successful(lnk);
			msp_set_as_ready(lnk);
		} else if (frame.opcode == MSP_OP_F_ACK) {
			/* Treat the F_ACK differently depending on the state */
			len = msp_next_data_length(lnk);
			if (lnk->state == MSP_LINK_STATE_SEND_TX_HEADER) {
				/* Acknowledged, prepare for the first data frame */
				lnk->state = MSP_LINK_STATE_SEND_TX_DATA;
				lnk->next_action = MSP_LINK_ACTION_TX_DATA;
				lnk->frame_id = frame.id ^ 1;
				r = msp_response_ok();
			} else if (len < (lnk->total_length - lnk->processed_length)) {
				/* Acknowledged, setup data next frame */
				lnk->next_action = MSP_LINK_ACTION_TX_DATA;
				lnk->frame_id = frame.id ^ 1;
				lnk->processed_length += len;
				r = msp_response_ok();
			} else {
				/* We should've gotten a T_ACK here! */
				lnk->error_count += 1;
				msp_debug("Got an F_ACK when we should have gotten a T_ACK");
				r = msp_response_error(MSP_OBC_ERR_INVALID_FRAME);
			}
		} else {
			/* The only frames that we should received when sending data are
			 * ACK frames. */
			lnk->error_count += 1;
			msp_debug("Did not get an ACK frame when sending data");
			r = msp_response_error(MSP_OBC_ERR_INVALID_FRAME);
		}
		break;
	case MSP_LINK_STATE_REQ_RX_RESPONSE:
		/* The only header with should get in a request transaction is a
		 * response header. */
		if (frame.opcode == MSP_OP_EXP_SEND) {
			lnk->state = MSP_LINK_STATE_REQ_RX_DATA;
			lnk->next_action = MSP_LINK_ACTION_TX_HEADER;
			lnk->transaction_id = frame.id;
			lnk->frame_id = frame.id;
			lnk->processed_length = 0;
			/* first check if this is a duplicate transaction */
			if (msp_seqflags_is_set(&lnk->flags, lnk->opcode, frame.id)) {
				/* Just pretend that the experiment is sending 0 data if this
				 * is a duplicate transaction */
				lnk->total_length = 0;
			} else {
				lnk->total_length = frame.dl;
			}
			r = msp_response_ok();
		} else {
			/* Something went wrong... */
			lnk->error_count += 1;
			r = msp_response_error(MSP_OBC_ERR_INVALID_FRAME);
		}
		break;
	default:
		r = msp_response_error(MSP_OBC_ERR_INVALID_STATE);
		break;
	}
	
	return r;
}




/**
 * @brief Checks if the link is active in a transaction. (Non-Blocking)
 * @param lnk The link to be checked.
 * @return 1 if the link is active in a transaction, 0 otherwise.
 */
int msp_is_active(const msp_link_t *lnk)
{
	if (lnk == NULL)
		return 0;
	if (lnk->state != MSP_LINK_STATE_READY)
		return 1;
	else
		return 0;
}


/**
 * @brief Checks the next action that should be taken by the OBC in an ongoing
 *        transaction. (Non-Blocking)
 * @param lnk The link in an ongoing transaction.
 * @return The next action to take.
 */
msp_link_action_t msp_next_action(const msp_link_t *lnk)
{
	if (lnk == NULL)
		return MSP_LINK_ACTION_DO_NOTHING;
	return lnk->next_action;
}


/**
 * @brief Returns the size of the data field of the next data frame that is to
 *        be sent or received.
 * @param lnk The link in in whose ongoing transaction the data frame will be
 *            sent or received.
 * @return The size of the data field in bytes.
 */
unsigned long msp_next_data_length(const msp_link_t *lnk)
{
	unsigned long remaining;
	if (lnk == NULL)
		return 0;

	remaining = lnk->total_length - lnk->processed_length;
	if (remaining > lnk->mtu)
		return lnk->mtu;
	else
		return remaining;
}


/**
 * @brief Returns the offset of the data in the next data frame that is to be
 *        sent or received.
 * @param lnk The link in in whose ongoing transaction the data frame will be
 *            sent or received.
 * @return The offset of the data in bytes.
 *
 * In essence, this function returns the number of bytes that have been sent or
 * received successfully so far in the ongoing transaction. The offset should
 * be used to determine where you should start to send data from when calling
 * msp_send_data_frame() or where to insert data into after calling
 * msp_recv_data_frame().
 */
unsigned long msp_next_data_offset(const msp_link_t *lnk)
{
	if (lnk == NULL)
		return 0;
	return lnk->processed_length;
}


/**
 * @brief Returns the number of errors that have occurred in the ongoing
 *        transaction.
 * @param lnk The link with the ongoing transaction.
 * @return The number of errors that have occurred.
 */
int msp_error_count(const msp_link_t *lnk)
{
	if (lnk == NULL)
		return 0;
	return lnk->error_count;
}
