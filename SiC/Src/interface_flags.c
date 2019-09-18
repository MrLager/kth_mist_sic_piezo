/****************************************************************************
 * FUNCTIONS USED TO INTERACT WITH SEQUENCE FLAGS                           *
 ****************************************************************************/

/**
 *****************************************************************************
 * @file interface_flags.c
 * @date 2019-06-26
 * @bug no known buggs
 * @brief functions to save and restore msp flags
 *****************************************************************************
 */

#include <stdbool.h>
#include "msp_exp_state.h"

void restore_seqflags(void)
{
    msp_seqflags_t seqflags;

    bool checksum_correct = EEPROM_read_buffer(seqflags.values, seqflags.inits);
    if (checksum_correct)
    {
        msp_exp_state_initialize(seqflags);
    }
    else
    {
        msp_exp_state_initialize(msp_seqflags_init());
    }
}

void save_seqflags(void)
{
    msp_seqflags_t seqflags = msp_exp_state_get_seqflags();

    EEPROM_write_buffer(seqflags.values, seqflags.inits);
}