#include "stm32l0xx_hal.h"

HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Wait until ADDR flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

    /* Wait until DIR flag is reset Receiver mode */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    while (hi2c->XferCount > 0U)
    {
      /* Wait until RXNE flag is set */
      if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        /* Disable Address Acknowledge */
        hi2c->Instance->CR2 |= I2C_CR2_NACK;

        /* Store Last receive data if any */
        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET)
        {
          /* Read data from RXDR */
          *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

          /* Increment Buffer pointer */
          hi2c->pBuffPtr++;

          hi2c->XferCount--;
        }

        return HAL_ERROR;
      }

      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
    }

    /* Wait until STOP flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Clear STOP flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Wait until BUSY flag is reset */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, Timeout, tickstart) != HAL_OK)
    {
      /* Disable Address Acknowledge */
      hi2c->Instance->CR2 |= I2C_CR2_NACK;
      return HAL_ERROR;
    }

    /* Disable Address Acknowledge */
    hi2c->Instance->CR2 |= I2C_CR2_NACK;

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

HAL_StatusTypeDef HAL_I2C_Slave_TxRx_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode        = HAL_I2C_MODE_SLAVE;
    hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

    /* Enable Address Acknowledge */
    hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->XferISR     = I2C_Slave_ISR_IT;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */

    /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
    I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT | I2C_XFER_TX_IT | I2C_XFER_LISTEN_IT);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

static void I2C_ITAddrCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
  uint8_t transferdirection;
  uint16_t slaveaddrcode;
  uint16_t ownadd1code;
  uint16_t ownadd2code;
  
  transferdirection = I2C_GET_DIR(hi2c);
  slaveaddrcode     = I2C_GET_ADDR_MATCH(hi2c);
  ownadd1code       = I2C_GET_OWN_ADDRESS1(hi2c);
  ownadd2code       = I2C_GET_OWN_ADDRESS2(hi2c);
  
  
  //I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);
  
  HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);

  /* Prevent unused argument(s) compilation warning */
  UNUSED(ITFlags);

  /* In case of Listen state, need to inform upper layer of address match code event */
  if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
  {
    transferdirection = I2C_GET_DIR(hi2c);
    slaveaddrcode     = I2C_GET_ADDR_MATCH(hi2c);
    ownadd1code       = I2C_GET_OWN_ADDRESS1(hi2c);
    ownadd2code       = I2C_GET_OWN_ADDRESS2(hi2c);
    


    /* If 10bits addressing mode is selected */
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
    {
      if ((slaveaddrcode & SlaveAddr_MSK) == ((ownadd1code >> SlaveAddr_SHIFT) & SlaveAddr_MSK))
      {
        slaveaddrcode = ownadd1code;
        hi2c->AddrEventCount++;
        if (hi2c->AddrEventCount == 2U)
        {
          /* Reset Address Event counter */
          hi2c->AddrEventCount = 0U;

          /* Clear ADDR flag */
          __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
          hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
          HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
        }
      }
      else
      {
        slaveaddrcode = ownadd2code;

        /* Disable ADDR Interrupts */
        I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
        HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
      }
    }
    /* else 7 bits addressing mode is selected */
    else
    {
      /* Disable ADDR Interrupts */
      I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
      hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
      HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
  }
  /* Else clear address flag only */
  else
  {
    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
  }
}

static void I2C_ITSlaveCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
  uint32_t tmpcr1value = READ_REG(hi2c->Instance->CR1);

  /* Clear STOP Flag */
  __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

  /* Disable all interrupts */
  I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT | I2C_XFER_RX_IT);

  /* Disable Address Acknowledge */
  hi2c->Instance->CR2 |= I2C_CR2_NACK;

  /* Clear Configuration Register 2 */
  I2C_RESET_CR2(hi2c);

  /* Flush TX register */
  I2C_Flush_TXDR(hi2c);

  /* If a DMA is ongoing, Update handle size context */
  if (I2C_CHECK_IT_SOURCE(tmpcr1value, I2C_CR1_TXDMAEN) != RESET)
  {
    if (hi2c->hdmatx != NULL)
    {
      hi2c->XferCount = (uint16_t)__HAL_DMA_GET_COUNTER(hi2c->hdmatx);
    }
  }
  else if (I2C_CHECK_IT_SOURCE(tmpcr1value, I2C_CR1_RXDMAEN) != RESET)
  {
    if (hi2c->hdmarx != NULL)
    {
      hi2c->XferCount = (uint16_t)__HAL_DMA_GET_COUNTER(hi2c->hdmarx);
    }
  }
  else
  {
    /* Do nothing */
  }

  /* Store Last receive data if any */
  if (I2C_CHECK_FLAG(ITFlags, I2C_FLAG_RXNE) != RESET)
  {
    /* Remove RXNE flag on temporary variable as read done */
    ITFlags &= ~I2C_FLAG_RXNE;

    /* Read data from RXDR */
    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

    /* Increment Buffer pointer */
    hi2c->pBuffPtr++;

    if ((hi2c->XferSize > 0U))
    {
      hi2c->XferSize--;
      hi2c->XferCount--;
    }
  }

  /* All data are not transferred, so set error code accordingly */
  if (hi2c->XferCount != 0U)
  {
    /* Set ErrorCode corresponding to a Non-Acknowledge */
    hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
  }

  hi2c->PreviousState = I2C_STATE_NONE;
  hi2c->Mode = HAL_I2C_MODE_NONE;
  hi2c->XferISR = NULL;

  if (hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
  {
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    I2C_ITError(hi2c, hi2c->ErrorCode);

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
    if (hi2c->State == HAL_I2C_STATE_LISTEN)
    {
      /* Call I2C Listen complete process */
      I2C_ITListenCplt(hi2c, ITFlags);
    }
  }
  else if (hi2c->XferOptions != I2C_NO_OPTION_FRAME)
  {
    /* Call the Sequential Complete callback, to inform upper layer of the end of Tranfer */
    I2C_ITSlaveSeqCplt(hi2c);

    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->State = HAL_I2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->ListenCpltCallback(hi2c);
#else
    HAL_I2C_ListenCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  /* Call the corresponding callback to inform upper layer of End of Transfer */
  else if (hi2c->State == HAL_I2C_STATE_BUSY_RX)
  {
    hi2c->State = HAL_I2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->SlaveRxCpltCallback(hi2c);
#else
    HAL_I2C_SlaveRxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
  else
  {
    hi2c->State = HAL_I2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->SlaveTxCpltCallback(hi2c);
#else
    HAL_I2C_SlaveTxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  }
}

