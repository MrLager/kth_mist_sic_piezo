/**
 * @file      msp_i2c_common_due.h
 * @author    John Wikman
 * @copyright MIT License
 * @brief     Common definitions for the Arduino Due I2C drivers in MSP.
 *
 * @details
 * Common definitions used by both the slave and master I2C implementation on
 * Arduino Due.
 *
 * Certain TWI functionalities does not have associated handler functions (like
 * when handling PDC). Consult the platform specific component_twi.h file and
 * processor specification for how to handle these cases.
 */

#ifndef MSP_I2C_COMMON_DUE_H
#define MSP_I2C_COMMON_DUE_H

#include <include/twi.h> /* Atmel CMSIS Driver */
#include "variant.h"     /* WIRE pins and definitions */

#if WIRE_INTERFACES_COUNT < 1
	#error No WIRE interface found.
#endif

/* Pointer to TWI0 */
static Twi *twi = WIRE_INTERFACE;

static void msp_i2c_enable(void)
{
	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);

	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
	PIO_Configure(g_APinDescription[PIN_WIRE_SDA].pPort,
	              g_APinDescription[PIN_WIRE_SDA].ulPinType,
	              g_APinDescription[PIN_WIRE_SDA].ulPin,
	              g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
	PIO_Configure(g_APinDescription[PIN_WIRE_SCL].pPort,
	              g_APinDescription[PIN_WIRE_SCL].ulPinType,
	              g_APinDescription[PIN_WIRE_SCL].ulPin,
	              g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

	NVIC_SetPriority(WIRE_ISR_ID, 0);
	NVIC_EnableIRQ(WIRE_ISR_ID);

	/* Disable the PDC (Peripheral DMA Controller) */
	twi->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
}

static void msp_i2c_disable(void)
{
	/* Un-disable the PDC (Peripheral DMA Controller) */
	twi->TWI_PTCR &= ~(TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS);

	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);

	pmc_disable_periph_clk(WIRE_INTERFACE_ID);
}

/*****************************************************************************
 * TWI_STATUS defines. There are already defines for TWI_SR_TXRDY and a few  *
 * others, but this ensures that there is a define for each possible status. *
 *****************************************************************************/
static inline int twi_status_isset(int sr, int mask)
{
	/* This statement should not be in a define, hence the function */
	return ((sr & mask) == mask);
}

/**
 * @brief Checks if all bits from a mask are set in a TWI status register.
 * @param sr The TWI status register.
 * @param mask The bit mask to check against.
 * @return A non-zero value if all the bits from the mask are set in the status
 *         register, 0 otherwise.
 */
#define TWI_STATUS_ISSET(sr, mask) twi_status_isset(sr, mask)

#ifndef TWI_STATUS_TXCOMP
/**
 * @brief Checks if TWI_SR_TXCOMP is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_TXCOMP is set, 0 otherwise.
 */
#define TWI_STATUS_TXCOMP(sr) TWI_STATUS_ISSET(sr, TWI_SR_TXCOMP)
#endif

#ifndef TWI_STATUS_RXRDY
/**
 * @brief Checks if TWI_SR_RXRDY is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_RXRDY is set, 0 otherwise.
 */
#define TWI_STATUS_RXRDY(sr) TWI_STATUS_ISSET(sr, TWI_SR_RXRDY)
#endif

#ifndef TWI_STATUS_TXRDY
/**
 * @brief Checks if TWI_SR_TXRDY is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_TXRDY is set, 0 otherwise.
 */
#define TWI_STATUS_TXRDY(sr) TWI_STATUS_ISSET(sr, TWI_SR_TXRDY)
#endif

#ifndef TWI_STATUS_SVREAD
/**
 * @brief Checks if TWI_SR_SVREAD is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_SVREAD is set, 0 otherwise.
 */
#define TWI_STATUS_SVREAD(sr) TWI_STATUS_ISSET(sr, TWI_SR_SVREAD)
#endif

#ifndef TWI_STATUS_SVACC
/**
 * @brief Checks if TWI_SR_SVACC is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_SVACC is set, 0 otherwise.
 */
#define TWI_STATUS_SVACC(sr) TWI_STATUS_ISSET(sr, TWI_SR_SVACC)
#endif

#ifndef TWI_STATUS_GACC
/**
 * @brief Checks if TWI_SR_GACC is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_GACC is set, 0 otherwise.
 */
#define TWI_STATUS_GACC(sr) TWI_STATUS_ISSET(sr, TWI_SR_GACC)
#endif

#ifndef TWI_STATUS_OVRE
/**
 * @brief Checks if TWI_SR_OVRE is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_OVRE is set, 0 otherwise.
 */
#define TWI_STATUS_OVRE(sr) TWI_STATUS_ISSET(sr, TWI_SR_OVRE)
#endif

#ifndef TWI_STATUS_NACK
/**
 * @brief Checks if TWI_SR_NACK is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_NACK is set, 0 otherwise.
 */
#define TWI_STATUS_NACK(sr) TWI_STATUS_ISSET(sr, TWI_SR_NACK)
#endif

#ifndef TWI_STATUS_ARBLST
/**
 * @brief Checks if TWI_SR_ARBLST is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_ARBLST is set, 0 otherwise.
 */
#define TWI_STATUS_ARBLST(sr) TWI_STATUS_ISSET(sr, TWI_SR_ARBLST)
#endif

/* Unlike other defines, this is SCLWS instead of SCL_WS */
#ifndef TWI_STATUS_SCLWS
/**
 * @brief Checks if TWI_SR_SCLWS is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_SCLWS is set, 0 otherwise.
 *
 * Checks if TWI_SR_SCLWS is set in a TWI status register. Unlike similar
 * defines, the mask that is being checked against is named SCLWS instead of
 * SCL_WS.
 */
#define TWI_STATUS_SCLWS(sr) TWI_STATUS_ISSET(sr, TWI_SR_SCLWS)
#endif

#ifndef TWI_STATUS_EOSACC
/**
 * @brief Checks if TWI_SR_EOSACC is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_EOSACC is set, 0 otherwise.
 */
#define TWI_STATUS_EOSACC(sr) TWI_STATUS_ISSET(sr, TWI_SR_EOSACC)
#endif

#ifndef TWI_STATUS_ENDRX
/**
 * @brief Checks if TWI_SR_ENDRX is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_ENDRX is set, 0 otherwise.
 */
#define TWI_STATUS_ENDRX(sr) TWI_STATUS_ISSET(sr, TWI_SR_ENDRX)
#endif

#ifndef TWI_STATUS_ENDTX
/**
 * @brief Checks if TWI_SR_ENDTX is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_ENDTX is set, 0 otherwise.
 */
#define TWI_STATUS_ENDTX(sr) TWI_STATUS_ISSET(sr, TWI_SR_ENDTX)
#endif

#ifndef TWI_STATUS_RXBUFF
/**
 * @brief Checks if TWI_SR_RXBUFF is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_RXBUFF is set, 0 otherwise.
 */
#define TWI_STATUS_RXBUFF(sr) TWI_STATUS_ISSET(sr, TWI_SR_RXBUFF)
#endif

#ifndef TWI_STATUS_TXBUFE
/**
 * @brief Checks if TWI_SR_TXBUFE is set in a TWI status register.
 * @param sr The TWI status register.
 * @return A non-zero value if TWI_SR_TXBUFE is set, 0 otherwise.
 */
#define TWI_STATUS_TXBUFE(sr) TWI_STATUS_ISSET(sr, TWI_SR_TXBUFE)
#endif

#endif /* MSP_I2C_COMMON_DUE_H */
