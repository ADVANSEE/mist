/**
 * @file    SPIRIT_Calibration.c
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   Configuration and management of SPIRIT VCO-RCO calibration.
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
#include "SPIRIT_Calibration.h"
#include "SPIRIT_Spi_Driver.h"




/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_Calibration
 * @{
 */


/**
 * @defgroup Calibration_Private_TypesDefinitions       Calibration Private Types Definitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Calibration_Private_Defines                Calibration Private Defines
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup Calibration_Private_Macros                 Calibration Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Calibration_Private_Variables              Calibration Private Variables
 * @{
 */

/**
 *@}
 */



/**
 * @defgroup Calibration_Private_FunctionPrototypes     Calibration Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Calibration_Private_Functions              Calibration Private Functions
 * @{
 */

/**
 * @brief  Enables or Disables the RCO calibration.
 * @param  xNewState new state for RCO calibration.
           This parameter can be S_ENABLE or S_DISABLE.
 * @retval None.
 */
void SpiritCalibrationRco(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(PROTOCOL2_BASE, 1, &tempRegValue);

  /* Build new value for the register */
  if(xNewState==S_ENABLE)
    tempRegValue |= PROTOCOL2_RCO_CALIBRATION_MASK;
  else
    tempRegValue &= ~PROTOCOL2_RCO_CALIBRATION_MASK;

  /* Writes register to enable or disable the RCO calibration */
  g_xStatus = SpiritSpiWriteRegisters(PROTOCOL2_BASE, 1, &tempRegValue);

}


/**
 * @brief  Enables or Disables the VCO calibration.
 * @param  xNewState new state for VCO calibration.
           This parameter can be S_ENABLE or S_DISABLE.
 * @retval None.
 */
void SpiritCalibrationVco(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(PROTOCOL2_BASE, 1, &tempRegValue);

   /* Build new value for the register */
  if(xNewState==S_ENABLE)
    tempRegValue |= PROTOCOL2_VCO_CALIBRATION_MASK;
  else
    tempRegValue &= ~PROTOCOL2_VCO_CALIBRATION_MASK;

  /* Writes register to enable or disable the VCO calibration */
  g_xStatus = SpiritSpiWriteRegisters(PROTOCOL2_BASE, 1, &tempRegValue);

}


/**
 * @brief  Sets the RCO calibration words.
 * @param  cRwt RWT word for RCO calibration.
 *         This parameter can be a value of uint8_t.
 * @param  cRfb RFB word for RCO calibration.
 *         This parameter can be a value of uint8_t.
 * @retval None.
 */
void SpiritCalibrationSetRcoCalWords(uint8_t cRwt, uint8_t cRfb)
{
  uint8_t tempRegValue[2];

  /* Check the parameters */
  s_assert_param(IS_RFB_WORD(cRfb));
  s_assert_param(IS_RWT_WORD(cRwt));

  /* Build the value of RWT and the MSbits of the RFB word */
  tempRegValue[0] = (cRwt << 4) | (cRfb >> 1);

  /* Reads the register value to update the LSbit of RFB */
  g_xStatus = SpiritSpiReadRegisters(RCO_VCO_CALIBR_IN1_BASE, 1, &tempRegValue[1]);

  /* Build new value for the register */
  tempRegValue[1] = (tempRegValue[1] & 0x7F) | (cRfb<<7);

  /* Writes the new value for RCO calibration words */
  g_xStatus = SpiritSpiWriteRegisters(RCO_VCO_CALIBR_IN2_BASE, 2, tempRegValue);

}


/**
 * @brief  Returns the RCO calibration words.
 * @param  pcRwt pointer to the variable in which the RWT word has to be stored.
 *         This parameter is a variable of uint8_t*.
 * @param  pcRfb pointer to the variable in which the RFB word has to be stored.
 *         This parameter is a variable of uint8_t*.
 * @retval None.
 */
void SpiritCalibrationGetRcoCalWords(uint8_t* pcRwt, uint8_t* pcRfb)
{
  uint8_t tempRegValue[2];

  /* Reads the registers values */
  g_xStatus = SpiritSpiReadRegisters(RCO_VCO_CALIBR_OUT1_BASE, 2, tempRegValue);

  /* Build the RWT value */
  (*pcRwt) = tempRegValue[0] >> 4;
  /* Build the RFB value */
  (*pcRfb) = (tempRegValue[0] & 0x0F)<<1 | (tempRegValue[1]>>7);

}


/**
 * @brief  Returns the VCO calibration data from internal VCO calibrator.
 * @param  None.
 * @retval uint8_t VCO calibration data word.
 */
uint8_t SpiritCalibrationGetVcoCalData(void)
{
  uint8_t tempRegValue;

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(RCO_VCO_CALIBR_OUT0_BASE, 1, &tempRegValue);

  /* Build and returns the VCO calibration data value */
  return (tempRegValue & 0x7F);

}


/**
 * @brief  Sets the VCO calibration data to be used in TX mode.
 * @param  cVcoCalData calibration data word to be set.
 *         This parameter is a variable of uint8_t.
 * @retval None.
 */
void SpiritCalibrationSetVcoCalDataTx(uint8_t cVcoCalData)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_VCO_CAL_DATA(cVcoCalData));

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(RCO_VCO_CALIBR_IN1_BASE, 1, &tempRegValue);

  /* Build the value to be written */
  tempRegValue &= 0x80;
  tempRegValue |= cVcoCalData;

  /* Writes the new value of calibration data in TX */
  g_xStatus = SpiritSpiWriteRegisters(RCO_VCO_CALIBR_IN1_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the actual VCO calibration data used in TX mode.
 * @param  None.
 * @retval uint8_t Calibration data word used in TX mode.
 */
uint8_t SpiritCalibrationGetVcoCalDataTx(void)
{
  uint8_t tempRegValue;

  /* Reads the register containing the calibration data word used in TX mode */
  g_xStatus = SpiritSpiReadRegisters(RCO_VCO_CALIBR_IN1_BASE, 1, &tempRegValue);

  /* Mask the VCO_CALIBR_TX field and returns the value */
  return (tempRegValue & 0x7F);

}


/**
 * @brief  Sets the VCO calibration data to be used in RX mode.
 * @param  cVcoCalData calibration data word to be set.
 *         This parameter is a variable of uint8_t.
 * @retval None.
 */
void SpiritCalibrationSetVcoCalDataRx(uint8_t cVcoCalData)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_VCO_CAL_DATA(cVcoCalData));

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(RCO_VCO_CALIBR_IN0_BASE, 1, &tempRegValue);

  /* Build the value to be written */
  tempRegValue &= 0x80;
  tempRegValue |= cVcoCalData;

  /* Writes the new value of calibration data in RX */
  g_xStatus = SpiritSpiWriteRegisters(RCO_VCO_CALIBR_IN0_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the actual VCO calibration data used in RX mode.
 * @param  None.
 * @retval uint8_t Calibration data word used in RX mode.
 */
uint8_t SpiritCalibrationGetVcoCalDataRx(void)
{
  uint8_t tempRegValue;

  /* Reads the register containing the calibration data word used in TX mode */
  g_xStatus = SpiritSpiReadRegisters(RCO_VCO_CALIBR_IN0_BASE, 1, &tempRegValue);

  /* Mask the VCO_CALIBR_RX field and returns the value */
  return (tempRegValue & 0x7F);

}


/**
 * @brief  Sets the VCO calibration window.
 * @param  xRefWord value of REFWORD corresponding to the Ref_period according to the formula: CALIBRATION_WIN = 11*Ref_period/fxo.
           This parameter can be a value of @ref VcoWin.
 * @retval None.
 */
void SpiritCalibrationSetVcoWindow(VcoWin xRefWord)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_VCO_WIN(xRefWord));

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue);

  /* Build the values to be written */
  tempRegValue &= 0xFC;
  tempRegValue |= xRefWord;

  /* Writes the new value of VCO calibration window */
  g_xStatus = SpiritSpiWriteRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the VCO calibration window.
 * @param  None.
 * @retval VcoWin Value of REFWORD corresponding to the Ref_period according to the formula: CALIBRATION_WIN = 11*Ref_period/fxo.
 *         This parameter can be a value of @ref VcoWin.
 */
VcoWin SpiritCalibrationGetVcoWindow(void)
{
  uint8_t tempRegValue1, tempRegValue2;
  VcoWin refWord;

  /* Reads the register containing the REFWORD value */
  g_xStatus = SpiritSpiReadRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue1);

  /* Reads the Xtal configuration */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue2);

  /* Mask the REFWORD field */
  tempRegValue1 &= 0x03;

  /* Mask the 24_26_MHz_SELECT field */
  tempRegValue2 = ((tempRegValue2 & 0x40)>>6);

  /* In case of 26 MHz crystal */
  if(tempRegValue2)
  {
    switch(tempRegValue1)
    {
    case 0:
      refWord = CALIB_TIME_6_77_US_26MHZ;
      break;
    case 1:
      refWord = CALIB_TIME_13_54_US_26MHZ;
      break;
    case 2:
      refWord = CALIB_TIME_27_08_US_26MHZ;
      break;
    case 3:
      refWord = CALIB_TIME_54_15_US_26MHZ;
      break;
    }
  }

  /* In case of 24 MHz crystal */
  else
  {
    switch(tempRegValue1)
    {
    case 0:
      refWord = CALIB_TIME_7_33_US_24MHZ;
      break;
    case 1:
      refWord = CALIB_TIME_14_67_US_24MHZ;
      break;
    case 2:
      refWord = CALIB_TIME_29_33_US_24MHZ;
      break;
    case 3:
      refWord = CALIB_TIME_58_67_US_24MHZ;
      break;
    }
  }

  return refWord;

}

/**
 * @brief  Selects a VCO.
 * @param  xVco can be VCO_H or VCO_L according to which VCO select.
 *         This parameter can be a value of @ref VcoSel.
 * @retval None.
 */
void SpiritCalibrationSelectVco(VcoSel xVco)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_VCO_SEL(xVco));
  
  SpiritSpiReadRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue);
  
  tempRegValue &= 0xF9;
  
  if(xVco == VCO_H)
  {
    tempRegValue |= 0x02;
    
  }
  else
  {
    tempRegValue |= 0x04;
  }
  SpiritSpiWriteRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue);  
  
}



/**
 * @brief  Returns the VCO selected.
 * @param  void.
 * @retval VCO_H or VCO_L according to which VCO selected.
 *         This parameter can be a value of @ref VcoSel.
 */
VcoSel SpiritCalibrationGetVcoSelecttion(void)
{
  uint8_t tempRegValue;
  
  SpiritSpiReadRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue);
  
  tempRegValue = (tempRegValue>>1)&0x3;
  
  if(tempRegValue == 0x01)
  {
    return VCO_H;
    
  }
  else
  {
    return VCO_L;
  }
  
}


/**
 *@}
 */

/**
 *@}
 */


/**
 *@}
 */



/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
