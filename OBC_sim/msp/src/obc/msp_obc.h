/**
 * @file      msp_obc.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Includes all the commonly used headers in the OBC side of MSP.
 */

#ifndef MSP_OBC_H
#define MSP_OBC_H

#ifdef __cplusplus
extern "C" {
#endif
#include "msp_configuration.h"
#include "msp_constants.h"
#include "msp_crc.h"
#include "msp_endian.h"
#include "msp_opcodes.h"
#include "msp_seqflags.h"
#include "msp_obc_error.h"
#include "msp_obc_frame.h"
#include "msp_obc_link.h"
#include "msp_i2c_master.h"
#ifdef __cplusplus
}
#endif

#endif
