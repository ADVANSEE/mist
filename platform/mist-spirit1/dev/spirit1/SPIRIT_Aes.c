/**
 * @file    SPIRIT_Aes.c
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   Configuration and management of SPIRIT AES Engine.
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 */


/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Aes.h"
#include "SPIRIT_Spi_Driver.h"


/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_Aes
 * @{
 */


/**
 * @defgroup Aes_Private_TypesDefinitions       AES Private Types Definitions
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup Aes_Private_Defines                AES Private Defines
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup Aes_Private_Macros                 AES Private Macros
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup Aes_Private_Variables              AES Private Variables
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup Aes_Private_FunctionPrototypes     AES Private Function Prototypes
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup Aes_Private_Functions              AES Private Functions
 * @{
 */


/**
 * @brief  Enables or Disables the AES engine.
 * @param  xNewState new state for AES engine.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None
 */
void SpiritAesMode(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue = 0x00;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));

  /* Modifies the register value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= AES_MASK;
  }
  else
  {
    tempRegValue &= ~AES_MASK;
  }

  /* Writes the ANA_FUNC_CONF0 register to enable or disable the AES engine */
  g_xStatus = SpiritSpiWriteRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

}


/**
 * @brief  Writes the data to encrypt or decrypt into the AES_DATA_IN registers.
 * @param  pcBufferDataIn pointer to the user data buffer. The first byte of the array
 * 	   shall be the MSB byte and it will be put in the AES_DATA_IN[0] register, while
 * 	   the last one shall be the LSB and it will be put in the AES_DATA_IN[cDataLength-1]
 * 	   register. If data to write are less than 16 bytes the remaining AES_DATA_IN registers
 * 	   will be filled with bytes equal to 0. This parameter is an uint8_t*.
 * @param  cDataLength length of data in bytes.
 *         This parameter is an uint8_t.
 * @retval None
 */
void SpiritAesWriteDataIn(uint8_t* pcBufferDataIn, uint8_t cDataLength)
{
  uint8_t i, dataInArray[16];

  /* Verifies that there are no more than 16 bytes */
  (cDataLength>16) ? (cDataLength=16) : cDataLength;

  /* Fill the dataInArray with the data buffer, using padding */
  for(i=0;i<16;i++)
  {
    (i<(16 - cDataLength)) ? (dataInArray[i]=0):(dataInArray[i]=pcBufferDataIn[15-i]);

  }

  /* Writes the AES_DATA_IN registers */
  g_xStatus = SpiritSpiWriteRegisters(AES_DATA_IN_15_BASE, 16, dataInArray);

}


/**
 * @brief  Returns the encrypted or decrypted data from the AES_DATA_OUT register.
 * @param  pcBufferDataOut pointer to the user data buffer. The AES_DATA_OUT[0]
 *         register value will be put as first element of the buffer (MSB), while the
 *         AES_DAT_OUT[cDataLength-1] register value will be put as last element of the buffer (LSB).
 * 	   This parameter is a uint8_t*.
 * @param  cDataLength length of data to read in bytes.
 *         This parameter is a uint8_t.
 * @retval None
 */
void SpiritAesReadDataOut(uint8_t* pcBufferDataOut, uint8_t cDataLength)
{
  uint8_t address, dataOutArray[16];
  int i;

  /* Verifies that there are no more than 16 bytes */
  (cDataLength>16) ? (cDataLength=16) : cDataLength;

  /* Evaluates the address of AES_DATA_OUT from which start to read */
  address = AES_DATA_OUT_15_BASE+16-cDataLength;

  /* Reads the exact number of AES_DATA_OUT registers */
  g_xStatus = (SpiritSpiReadRegisters(address, cDataLength, dataOutArray));

  /* Copy in the user buffer the read values changing the order */
  for(i = (cDataLength-1); i>=0; i--)
  {
    *pcBufferDataOut = dataOutArray[i];
    pcBufferDataOut++;
  }

}


/**
 * @brief  Writes the encryption key into the AES_KEY_IN register.
 * @param  pcKey pointer to the buffer of 4 words containing the AES key.
 *         The first byte of the buffer shall be the most significant byte AES_KEY_0 of the AES key.
 *         The last byte of the buffer shall be the less significant byte AES_KEY_15 of the AES key.
 * 	   This parameter is an uint8_t*.
 * @retval None
 */
void SpiritAesWriteKey(uint8_t* pcKey)
{
  uint8_t pcTempKey[16]; 
  uint8_t i;
  for (i = 0; i < 16; i++)
    pcTempKey[15-i] = pcKey[i];
  
  /* Writes the AES_DATA_IN registers */
  g_xStatus = SpiritSpiWriteRegisters(AES_KEY_IN_15_BASE, 16, pcTempKey);

}

/**
 * @brief  Returns the encryption/decryption key from the AES_KEY_IN register.
 * @param  pcKey  pointer to the buffer of 4 words (16 bytes) containing the AES key.
 *         The first byte of the buffer shall be the most significant byte AES_KEY_0 of the AES key.
 *         The last byte of the buffer shall be the less significant byte AES_KEY_15 of the AES key.
 *         This parameter is an uint8_t*.
 * @retval None
 */
void SpiritAesReadKey(uint8_t* pcKey)
{
  uint8_t pcTempKey[16];
  uint8_t i;

  /* Reads the AES_DATA_IN registers */
  g_xStatus = SpiritSpiReadRegisters(AES_KEY_IN_15_BASE, 16, pcTempKey);


  for (i = 0; i < 16; i++)
    pcKey[i] = pcTempKey[15-i];

}



/**
 * @brief  Derives the decryption key from a given encryption key.
 * @param  None.
 * @retval None.
 */
void SpiritAesDeriveDecKeyFromEnc(void)
{
  /* Sends the COMMAND_AES_KEY command */
  g_xStatus = SpiritSpiCommandStrobes(COMMAND_AES_KEY);

}


/**
 * @brief  Executes the encryption operation.
 * @param  None.
 * @retval None.
 */
void SpiritAesExecuteEncryption(void)
{
  /* Sends the COMMAND_AES_ENC command */
  g_xStatus = SpiritSpiCommandStrobes(COMMAND_AES_ENC);

}


/**
 * @brief  Executes the decryption operation.
 * @param  None.
 * @retval None.
 */
void SpiritAesExecuteDecryption(void)
{
  /* Sends the COMMAND_AES_DEC command */
  g_xStatus = SpiritSpiCommandStrobes(COMMAND_AES_DEC);

}


/**
 * @brief  Executes the key derivation and the decryption operation.
 * @param  None.
 * @retval None.
 */
void SpiritAesDeriveDecKeyExecuteDec(void)
{
  /* Sends the COMMAND_AES_KEY_DEC command */
  g_xStatus = SpiritSpiCommandStrobes(COMMAND_AES_KEY_DEC);

}


/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */



/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
