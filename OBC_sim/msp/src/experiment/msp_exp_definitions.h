/**
 * @file      msp_exp_definitions.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Contains defines for the experiment side of MSP.
 *
 * @details
 * Contains definitions that the experiment side of MSP uses during
 * communication. Some definitions can come from other files, in which case
 * this header only checks that they are actually defined. Each definition is
 * described here:
 *  - MSP_EXP_ADDR: The address of the experiment. Should be defined in
 *                  msp_configuration.h after running the configuration script.
 *  - MSP_EXP_MTU: The maximum transmission unit of the experiment This
 *                 specifies the maximum amount of data that can fit inside of
 *                 the data field of a frame. Should be defined in
 *                 msp_configuration.h after running the configuration script.
 *  - MSP_MAX_FRAME_SIZE: The maximum size an MSP frame can have. This
 *                        definition should be used to determine minimum size
 *                        of the buffers used to send or receive MSP frames.
 *                        This is defined in this file.
 */

#ifndef MSP_EXP_DEFINITIONS_H
#define MSP_EXP_DEFINITIONS_H

/* Import MSP_EXP_ADDR and MSP_EXP_MTU from the configuration file */
#include "msp_configuration.h"

#ifndef MSP_EXP_ADDR
#error MSP_EXP_ADDR not set
#endif

#ifndef MSP_EXP_MTU
#error MSP_EXP_MTU not set
#else
#define MSP_EXP_MAX_FRAME_SIZE (((MSP_EXP_MTU) + 5) > 9 ? ((MSP_EXP_MTU) + 5) : 9)
#endif


#endif /* MSP_EXP_DEFINITIONS_H */
