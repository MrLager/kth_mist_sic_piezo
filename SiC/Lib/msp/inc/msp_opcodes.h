/**
 * @file      msp_opcodes.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Defines all the standard opcodes in MSP.
 *
 * @details
 * Defines all the standard opcodes in MSP as well as macros for categorizing
 * opcodes.
 */

#ifndef MSP_OPCODES_H
#define MSP_OPCODES_H

/* MSP Control Flow */
#define MSP_OP_NULL        0x00
#define MSP_OP_DATA_FRAME  0x01
#define MSP_OP_F_ACK       0x02
#define MSP_OP_T_ACK       0x03
#define MSP_OP_EXP_SEND    0x04
#define MSP_OP_EXP_BUSY    0x05

/* System Commands */
#define MSP_OP_ACTIVE      0x10
#define MSP_OP_SLEEP       0x11
#define MSP_OP_POWER_OFF   0x12

/* Standard OBC Request */
#define MSP_OP_REQ_PAYLOAD 0x20
#define MSP_OP_REQ_HK      0x21
#define MSP_OP_REQ_PUS     0x22

/* Standard OBC Read */
#define MSP_OP_SEND_TIME   0x30
#define MSP_OP_SEND_PUS    0x31


/* Values for determining opcode type */
#define MSP_OP_TYPE_CTRL 0x00
#define MSP_OP_TYPE_SYS  0x10
#define MSP_OP_TYPE_REQ  0x20
#define MSP_OP_TYPE_SEND 0x30
   
   

#define START_EXP_PIEZO        0x50
#define STOP_EXP_PIEZO         0x51
#define START_EXP_SIC          0X52
#define SIC_10V_OFF            0x53
#define PIEZO_5V_OFF           0x54
#define PIEZO_48V_OFF          0x55 
#define VBAT_OFF               0x56
   
#define REQ_PIEZO              0x60
#define REQ_SIC                0x61
/**
 * @brief Determines the opcode type.
 * @param opcode The opcode value.
 * @return The type of the opcode. It will return either MSP_OP_TYPE_CTRL,
 *         MSP_OP_TYPE_SYS, MSP_OP_TYPE_REQ, or MSP_OP_TYPE_SEND.
 */
#define MSP_OP_TYPE(opcode) ((opcode) & 0x30)

/**
 * @brief Determines whether the opcode is custom or not.
 * @param opcode The opcode value.
 * @return A non-zero value if the value represents a custom opcode. Otherwise
 *         0 is returned.
 */
#define MSP_OP_IS_CUSTOM(opcode) (((opcode) & 0x70) >= 0x50)

#endif /* MSP_OPCODES_H */
