/**
 * @file      msp_obc_link.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Declares functions and defines data types for handling MSP
 *            transaction from the OBC side.
 *
 * @details
 * Declares functions for sending and receiving data over MSP from an OBC
 * perspective. Also defines a link struct for keeping track of the state
 * towards an experiment. The link struct includes variables that keep track of
 * things such as the address of the experiment, previous transactions-ID's,
 * the current transaction-ID, the opcode of the current transaction, etc.
 */

#ifndef MSP_OBC_LINK_H
#define MSP_OBC_LINK_H

#include "msp_seqflags.h"

/**
 * @brief An enum for representing the state of an MSP transaction in a link.
 */
typedef enum {
	MSP_LINK_STATE_READY, /**< Ready state. */
	MSP_LINK_STATE_SEND_TX_HEADER, /**< OBC Send, send initial header frame. */
	MSP_LINK_STATE_SEND_TX_DATA, /**< OBC Send, send data frame. */
	MSP_LINK_STATE_REQ_TX_HEADER, /**< OBC Request, send request header. */
	MSP_LINK_STATE_REQ_RX_RESPONSE, /**< OBC Request, read response header. */
	MSP_LINK_STATE_REQ_RX_DATA /**< OBC Request, read data frmae. */
} msp_link_state_t;

/**
 * @brief An enum describing the next action that a link should take in an
 *        ongoing transaction.
 */
typedef enum {
	MSP_LINK_ACTION_DO_NOTHING, /**< Take no action (not in a transaction). */
	MSP_LINK_ACTION_TX_HEADER, /**< Call msp_send_header_frame()  */
	MSP_LINK_ACTION_TX_DATA, /**< Call msp_send_data_frame()  */
	MSP_LINK_ACTION_RX_HEADER, /**< Call msp_recv_header_frame()  */
	MSP_LINK_ACTION_RX_DATA /**< Call msp_recv_data_frame()  */
} msp_link_action_t;

/**
 * @brief An enum describing the nature of an msp_response struct.
 */
typedef enum {
	MSP_RESPONSE_OK, /**< Action was taken successfully without error. */
	MSP_RESPONSE_ERROR, /**< An error occurred when taking action. */
	MSP_RESPONSE_BUSY, /**< The experiment notified that it was busy. */
	MSP_RESPONSE_TRANSACTION_SUCCESSFUL, /**< Ongoing transaction was
	                                          completed successfully. */
	MSP_RESPONSE_TRANSACTION_ABORTED /**< Ongoing transaction was aborted. */
} msp_response_type_t;

/**
 * @brief A struct for describing an MSP link towards a single experiment.
 */
typedef struct msp_link {
	/**
	 * @brief The experiment address.
	 */
	unsigned char slave_address;

	/**
	 * @brief Pointer to the buffer that will be used during MSP transactions.
	 */
	unsigned char *buffer;

	/**
	 * @brief The MTU of the experiment.
	 */
	unsigned long mtu;

	/**
	 * @brief Sequence flags for remembering previous transaction-ID's.
	 */
	msp_seqflags_t flags;


	/**
	 * @brief A variable for keeping track of the current link state.
	 */
	msp_link_state_t state;

	/**
	 * @brief A variable for keeping track of the next action in an MSP
	 *        transaction.
	 */
	msp_link_action_t next_action;

	/**
	 * @brief A counter of the total number of errors that has occurred in the
	 *        ongoing transaction.
	 */
	int error_count;

	/**
	 * @brief The transaction-ID of the ongoing transaction.
	 */
	unsigned char transaction_id;

	/**
	 * @brief The frame-ID of the last received data frame or the frame-ID of
	 *        the next data frame to be sent (depends on the transaction type).
	 *
	 * If the ongoing transaction type is of OBC Request, then this variable
	 * represent the frame-ID of the last successfully received data frame.
	 *
	 * If the ongoing transaction type is of OBC Send, then this variable
	 * represents the frame-ID of the data frame that is to be sent next.
	 */
	unsigned char frame_id;

	/**
	 * @brief The opcode of the ongoing transaction.
	 */
	unsigned char opcode;

	/**
	 * @brief The total number of bytes of data that will be sent or received
	 *        with the transaction.
	 *
	 * This does not count data that will potentially be sent or received in
	 * a duplicate data frame.
	 */
	unsigned long total_length;

	/**
	 * @brief The number of bytes of data that has been sent or received so far
	 *        in the transaction.
	 *
	 * This does not count data that has been sent or received in a duplicate
	 * data frame.
	 */
	unsigned long processed_length;
} msp_link_t;

/**
 * @brief A struct that contains information about how an MSP link action went.
 *
 * If the status of the response is not MSP_OBC_RESPONSE_TRANSACTION_SUCCESSFUL
 * or MSP_RESPONSE_TRANSACTION_ABORTED, then the fields 'opcode',
 * 'transaction_id', and 'len' does not have a set value.
 */
struct msp_response {
	/**
	 * @brief The response status.
	 */
	msp_response_type_t status;

	/**
	 * @brief The error code from the action. This is only set if the status is
	 *        set as MSP_RESPONSE_ERROR.
	 */
	int error_code;

	/**
	 * @brief The opcode of the successful or aborted transaction.
	 *
	 * This value is only defined if the status is set to 
	 * MSP_OBC_RESPONSE_TRANSACTION_SUCCESSFUL or
	 * MSP_RESPONSE_TRANSACTION_ABORTED.
	 */
	unsigned char opcode;

	/**
	 * @brief The transaction-ID of the successful or aborted transaction.
	 *
	 * This value is only defined if the status is set to 
	 * MSP_OBC_RESPONSE_TRANSACTION_SUCCESSFUL or
	 * MSP_RESPONSE_TRANSACTION_ABORTED.
	 */
	unsigned char transaction_id;

	/**
	 * @brief The length of the data that was received or sent in the
	 *        transaction.
	 *
	 * This value is only defined if the status is set to 
	 * MSP_OBC_RESPONSE_TRANSACTION_SUCCESSFUL or
	 * MSP_RESPONSE_TRANSACTION_ABORTED.
	 */
	unsigned long len;
};


msp_link_t msp_create_link(unsigned long slave_address,
                           msp_seqflags_t flags,
                           unsigned char *buf,
                           unsigned long mtu);

struct msp_response msp_start_transaction(msp_link_t *lnk,
                                          unsigned char opcode,
                                          unsigned long len);

struct msp_response msp_abort_transaction(msp_link_t *lnk);

struct msp_response msp_send_data_frame(msp_link_t *lnk,
                                        unsigned char *data,
                                        unsigned long datalen);

struct msp_response msp_send_header_frame(msp_link_t *lnk);

struct msp_response msp_recv_data_frame(msp_link_t *lnk,
                                        unsigned char *data,
                                        unsigned long *datalen);

struct msp_response msp_recv_header_frame(msp_link_t *lnk);


int msp_is_active(const msp_link_t *lnk);
msp_link_action_t msp_next_action(const msp_link_t *lnk);
unsigned long msp_next_data_length(const msp_link_t *lnk);
unsigned long msp_next_data_offset(const msp_link_t *lnk);
int msp_error_count(const msp_link_t *lnk);

#endif
