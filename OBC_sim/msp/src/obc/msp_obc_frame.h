/**
 * @file      msp_obc_frame.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     OBC side frame functionalities.
 *
 * @details
 * Defines a struct for handling frames in the OBC side of MSP along with
 * declaring corresponding encoding and decoding functions.
 */

#ifndef MSP_OBC_FRAME_H
#define MSP_OBC_FRAME_H

#include "msp_obc_link.h"

/**
 * @brief An enum for checking the type of an MSP frame.
 */
typedef enum {
	MSP_OBC_FRAME_ERROR, /**< An erroneous frame. */
	MSP_OBC_FRAME_HEADER, /**< A header frame. */
	MSP_OBC_FRAME_DATA /**< A data frame. */
} msp_obc_frametype_t;

/**
 * @brief A struct containing fields and identifiers of an MSP frame.
 */
struct msp_obc_frame {
	/**
	 * @brief The frame type.
	 *
	 *  - MSP_OBC_FRAME_HEADER indicates that this frame is a header frame.
	 *  - MSP_OBC_FRAME_DATA indicates that this frame is a data frame.
	 *  - MSP_OBC_FRAME_ERROR indicates that an error has occurred within this
	 *    frame, such as an FCS mismatch.
	 *
	 * If this field has any other value than MSP_OBC_FRAME_HEADER or
	 * MSP_OBC_FRAME_DATA, the values of all other fields in this struct are
	 * undefined.
	 */
	msp_obc_frametype_t type;

	/**
	 * @brief The frame-ID of the frame.
	 *
	 * This field should only take on the values 0 or 1. How this field should
	 * be interpreted depends on the value of the opcode field. If the opcode
	 * is of type SYS, SEND or if the opcode is MSP_OP_EXP_SEND (see
	 * msp_opcodes.h) then this field also determines the transaction-ID.
	 */
	unsigned char id;

	/**
	 * @brief The opcode of the frame.
	 *
	 * See msp_opcodes.h for macros to interpret this field.
	 */
	unsigned char opcode;

	/**
	 * @brief A pointer representing the data field in a data frame.
	 *
	 * The actual data field that this points to has to be allocated elsewhere.
	 * If the type of the frame is not MSP_OBC_FRAME_DATA, the value of this
	 * field is undefined.
	 */
	unsigned char *data;

	/**
	 * @brief The length of the data field in this frame measured in bytes.
	 *
	 * If the type of the frame is not MSP_OBC_FRAME_DATA, the value of this
	 * field is undefined.
	 */
	unsigned long datalen;

	/**
	 * @brief The value of the DL field in a header frame.
	 *
	 * The DL field in a header frame signals how many bytes will be sent
	 * throughout the entire transaction.
	 *
	 * If the type of the frame is not MSP_OBC_FRAME_HEADER, the value of this
	 * field is undefined.
	 */
	unsigned long dl;
};

struct msp_obc_frame msp_obc_decode_frame(const msp_link_t *lnk, unsigned char *src, unsigned long len);
int msp_obc_encode_frame(const msp_link_t *lnk, unsigned char *dest, unsigned long *len, struct msp_obc_frame frame);

#endif