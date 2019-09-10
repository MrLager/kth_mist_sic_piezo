/**
 * @file      msp_obc_error.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Contains definitions for all the internal error codes used for
 *            the OBC side the MSP library.
 *
 * Ranges [-9,-1] and [1,9] are reserved for I2C errors.
 *
 * An error <= -10 represents a pre-check error where a prerequisite to an
 * action has not been met. An example of this is if a pointer is NULL, the
 * length of the data to be sent is incorrect, etc. If this type of error is
 * returned, the state of the link will remain unchanged.
 *
 * An error >= 10 represents a general link error with no guarantees on
 * whether the state of the link has changed or not. 
 *
 * NOTE: Response types for link actions are not defined here.
 */

#ifndef MSP_OBC_ERROR_H
#define MSP_OBC_ERROR_H

/* Link (Pre-Check) Errors */
#define MSP_OBC_ERR_INVALID_OPCODE -10
#define MSP_OBC_ERR_LENGTH_NOT_ZERO -11
#define MSP_OBC_ERR_NOT_IN_A_TRANSACTION -12
#define MSP_OBC_ERR_INVALID_LENGTH -13
#define MSP_OBC_ERR_NULL_POINTER -14
#define MSP_OBC_ERR_INVALID_ACTION -15

/* I2C Errors: [-9,-1] and [1,9] */

/* Link (General) Errors */
#define MSP_OBC_ERR_INVALID_STATE 10
#define MSP_OBC_ERR_INVALID_FRAME 11

#endif
