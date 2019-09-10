/**
 * @file      msp_obc_frame.c
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Implements functionalities for encoding and decoding frames on
 *            the OBC side of MSP.
 */

#include "msp_crc.h"
#include "msp_endian.h"
#include "msp_opcodes.h"
#include "msp_obc_error.h"
#include "msp_obc_frame.h"
#include "msp_obc_link.h"

/**
 * @brief Decodes a sequence of bytes into an MSP frame.
 * @param lnk Pointer to the MSP link which context should be used to decode
 *            the frame.
 * @param src Pointer to the sequence of bytes.
 * @param len The number of bytes pointed to by src.
 * @return The decoded frame. If the bytes pointed to by src could not be
 *         decoded into a frame, this will be indicated by the type field of
 *         the frame having the value MSP_OBC_FRAME_ERROR.
 */
struct msp_obc_frame msp_obc_decode_frame(const msp_link_t *lnk, unsigned char *src, unsigned long len)
{
    struct msp_obc_frame frame;
    unsigned long remainder;
    unsigned long fcs;
    unsigned char pseudo_header;

    /* Set the frame as erroneous by default */
    frame.type = MSP_OBC_FRAME_ERROR;

    if (len < 6)
        return frame;

    pseudo_header = (lnk->slave_address << 1) | 0x01;

    remainder = msp_crc32(&pseudo_header, 1, 0);
    remainder = msp_crc32(src, len - 4, remainder);

    fcs = msp_from_bigendian32(src + (len - 4));

    if (fcs != remainder)
        return frame;

    frame.opcode = src[0] & 0x7F;
    frame.id = (src[0] >> 7) & 0x01;

    if (frame.opcode == MSP_OP_DATA_FRAME) {
        frame.type = MSP_OBC_FRAME_DATA;
        frame.data = src + 1;
        frame.datalen = len - 5;
    } else if (len == 9) { /* A header frame must have len == 9 */
        frame.type = MSP_OBC_FRAME_HEADER;
        frame.dl = msp_from_bigendian32(src + 1);
    }

    return frame;
}

/**
 * @brief Encodes an MSP frame into a sequence of bytes.
 * @param lnk Pointer to the MSP link which context should be used to encode
 *            the frame.
 * @param dest Pointer to the destination where the encoded sequence of bytes
 *             will be stored. The capacity of this region should be at least 9
 *             bytes if a header frame is encoded. If a data frame is being
 *             encoded, the capacity of this region must be at least 5 + the
 *             length of the data field.
 * @param len A pointer to a 32-bit unsigned integer where the resulting length
 *            of the encoded bytes will be written.
 * @param frame The MSP frame to be encoded.
 * @return 0 if the frame was successfully encoded, otherwise an error code
 *         from msp_obc_error.h is returned.
 */
int msp_obc_encode_frame(const msp_link_t *lnk, unsigned char *dest, unsigned long *len, struct msp_obc_frame frame)
{
    unsigned long i;
    unsigned long fcs;
    unsigned char pseudo_header;

    /* Do not encode erroneous frames */
    if (frame.type == MSP_OBC_FRAME_ERROR)
        return MSP_OBC_ERR_INVALID_FRAME;

    dest[0] = frame.opcode | ((frame.id << 7) & 0x80);

    if (frame.type == MSP_OBC_FRAME_DATA) {
        for (i = 0; i < frame.datalen; i++)
            dest[i + 1] = frame.data[i];
        
        *len = frame.datalen + 5;
    } else {
        msp_to_bigendian32(dest + 1, frame.dl);
        *len = 9;
    }

    pseudo_header = (lnk->slave_address << 1);

    fcs = msp_crc32(&pseudo_header, 1, 0);
    fcs = msp_crc32(dest, *len - 4, fcs);

    msp_to_bigendian32(dest + (*len - 4), fcs);

    return 0;
}
