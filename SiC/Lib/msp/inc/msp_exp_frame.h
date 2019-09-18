/**
 * @file      msp_exp_frame.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Declares functions for handling MSP frames on the experiment side.
 */

#ifndef MSP_FRAME_H
#define MSP_FRAME_H

unsigned long msp_exp_frame_generate_fcs(const unsigned char *data, int from_obc, unsigned long len, char addr);
int msp_exp_frame_fcs_valid(const unsigned char *data, int from_obc, unsigned long len, char addr);
void msp_exp_frame_format_header(unsigned char *dest, unsigned char opcode, unsigned char frame_id, unsigned long dl, char addr);
void msp_exp_frame_format_empty_header(unsigned char *dest, unsigned char opcode , char addr);

#endif /* MSP_FRAME_H */
