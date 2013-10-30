/**
 * @file    SPIRIT_General.c
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   Configuration and management of SPIRIT General functionalities.
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
#include "SPIRIT_General.h"
#include "SPIRIT_Spi_Driver.h"


/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_General
 * @{
 */


/**
 * @defgroup General_Private_TypesDefinitions   General Private Types Definitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup General_Private_Defines            General Private Defines
 * @{
 */

#define s_assert_param(x) 


/**
 *@}
 */


/**
 * @defgroup General_Private_Macros             General Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup General_Private_Variables          General Private Variables
 * @{
 */

/**
 * @brief  Spirit version global variable.
 *         This static variable of @ref SpiritVersion type.
 */
static SpiritVersion s_xSpiritVersion = SPIRIT_VERSION_2_0;

/**
 *@}
 */


/**
 * @defgroup General_Private_FunctionPrototypes         General Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup General_Private_Functions                          General Private Functions
 * @{
 */

/**
 * @brief  Enables or Disables the output of battery level detector.
 * @param  xNewState new state for battery level detector.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None
 */
void SpiritGeneralBatteryLevel(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));

  /* Reads the ANA_FUNC_CONF0_BASE register value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

  /* Build the value to be stored */
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= BATTERY_LEVEL_MASK;
  }
  else
  {
    tempRegValue &= ~BATTERY_LEVEL_MASK;
  }

  /* Writes the new value */
  g_xStatus = SpiritSpiWriteRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);
}


/**
 * @brief  Sets the battery level.
 * @param  xBatteryLevel new state for battery level.
 *         This parameter can be a value of @ref BatteryLevel.
 * @retval None.
 */
void SpiritGeneralSetBatteryLevel(BatteryLevel xBatteryLevel)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_BLD_LVL(xBatteryLevel));

  /* Reads the ANA_FUNC_CONF1_BASE register value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF1_BASE, 1, &tempRegValue);

  /* Build the value to be stored */
  tempRegValue &= ~ANA_FUNC_CONF1_SET_BLD_LVL_MASK;
  switch(xBatteryLevel){
    case BLD_LVL_2_7_V:
      tempRegValue |= BLD_LVL_2_7;
      break;
    case BLD_LVL_2_5_V:
      tempRegValue |= BLD_LVL_2_5;
      break;
    case BLD_LVL_2_3_V:
      tempRegValue |= BLD_LVL_2_3;
      break;
    case BLD_LVL_2_1_V:
      tempRegValue |= BLD_LVL_2_1;
      break;
  }

  /* Writes the new value */
  g_xStatus = SpiritSpiWriteRegisters(ANA_FUNC_CONF1_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the settled battery level.
 * @param  None.
 * @retval BatteryLevel Settled battery level. This parameter can be a value of @ref BatteryLevel.
 */
BatteryLevel SpiritGeneralGetBatteryLevel(void)
{
  uint8_t tempRegValue;

  /* Reads the ANA_FUNC_CONF1_BASE register value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF1_BASE, 1, &tempRegValue);

  /* Mask the battery level field and returns the settled battery level */
  return ((BatteryLevel)(tempRegValue & ANA_FUNC_CONF1_SET_BLD_LVL_MASK));

}


/**
 * @brief  Enables or Disables the output of brown out detector.
 * @param  xNewState new state for brown out detector.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None.
 */
void SpiritGeneralBrownOut(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));

  /* Reads the ANA_FUNC_CONF0_BASE register value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

  /* Build the value to be stored */
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= BROWN_OUT_MASK;
  }
  else
  {
    tempRegValue &= ~BROWN_OUT_MASK;
  }

  /* Writes value on register */
  g_xStatus = SpiritSpiWriteRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

}


/**
 * @brief  Sets High Power Mode.
 * @param  xNewState new state for High Power Mode.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None.
 */
void SpiritGeneralHighPwr(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));

  /* Reads the ANA_FUNC_CONF0_BASE register value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

  /* Build the value to write */
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= HIGH_POWER_MODE_MASK;
  }
  else
  {
    tempRegValue &= ~HIGH_POWER_MODE_MASK;
  }

  /* Writes the new value on register */
  g_xStatus = SpiritSpiWriteRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

}


/**
 * @brief  Sets External Reference.
 * @param  xExtMode new state for the external reference.
 *         This parameter can be: MODE_EXT_XO or MODE_EXT_XIN.
 * @retval None.
 */
void SpiritGeneralSetExtRef(ModeExtRef xExtMode)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_MODE_EXT(xExtMode));

  /* Reads the ANA_FUNC_CONF0_BASE register value */
  SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

  /* Build the value to write */
  if(xExtMode == MODE_EXT_XO)
  {
    tempRegValue |= EXT_REF_MASK;
  }
  else
  {
    tempRegValue &= ~EXT_REF_MASK;
  }

  /* Writes value on register */
  g_xStatus = SpiritSpiWriteRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns External Reference.
 * @param  None.
 * @retval ModeExtRef Settled external reference.
 *         This parameter can be: MODE_EXT_XO or MODE_EXT_XIN.
 */
ModeExtRef SpiritGeneralGetExtRef(void)
{
  uint8_t tempRegValue;

  /* Reads the ANA_FUNC_CONF0_BASE register value and return the result */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);

  /* Mask the EXT_REF field field and returns the settled reference signal */
  return ((ModeExtRef)((tempRegValue & 0x10)>>4));

}


/**
 * @brief  Sets XO gm at startup.
 * @param  xGm transconductance value of XO at startup.
 *         This parameter can be a value of @ref GmConf.
 * @retval None.
 */
void SpiritGeneralSetXoGm(GmConf xGm)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_GM_CONF(xGm));

  /* Reads the ANA_FUNC_CONF1_BASE register value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF1_BASE, 1, &tempRegValue);

  /* Build the value to write */
  tempRegValue &= ~ANA_FUNC_CONF1_GMCONF_MASK;
  switch(xGm){
    case GM_SU_13_2:
      tempRegValue |= GM_13_2;
      break;
    case GM_SU_18_2:
      tempRegValue |= GM_18_2;
      break;
    case GM_SU_21_5:
      tempRegValue |= GM_21_5;
      break;
    case GM_SU_25_6:
      tempRegValue |= GM_25_6;
      break;
    case GM_SU_28_8:
      tempRegValue |= GM_28_8;
      break;
    case GM_SU_33_9:
      tempRegValue |= GM_33_9;
      break;
    case GM_SU_38_5:
      tempRegValue |= GM_38_5;
      break;
    case GM_SU_43_0:
      tempRegValue |= GM_43_0;
      break;
  }

  /* Writes new value on register */
  g_xStatus = SpiritSpiWriteRegisters(ANA_FUNC_CONF1_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the configured XO gm at startup.
 * @param  None.
 * @retval GmConf Settled XO gm. This parameter can be a value of @ref GmConf.
 */
GmConf SpiritGeneralGetXoGm(void)
{
  uint8_t tempRegValue;

  /* Reads the ANA_FUNC_CONF1_BASE register value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF1_BASE, 1, &tempRegValue);

  /* Mask the GM_CONF field field and returns the settled transconductance of the XO at startup */
  return ((GmConf)((tempRegValue & 0x1C)>>2));

}


/**
 * @brief  Returns the settled packet format.
 * @param  None.
 * @retval PacketType Settled packet type. This parameter can be a value of @ref PacketType.
 */
PacketType SpiritGeneralGetPktType(void)
{
  uint8_t tempRegValue;

  /* Reads the PROTOCOL1 register */
  g_xStatus = SpiritSpiReadRegisters(PCKTCTRL3_BASE, 1, &tempRegValue);

  /* cast and return value */
  return (PacketType)(tempRegValue>>6);

}



/**
 * @brief  Returns device part number.
 * @param  None.
 * @retval uint8_t Device part number.
 */
uint8_t SpiritGeneralGetDevicePartNumber(void)
{
  uint8_t tempRegValue;

  /* Reads the register value containing the device part number */
  g_xStatus = SpiritSpiReadRegisters(DEVICE_INFO1_PARTNUM, 1, &tempRegValue);

  return tempRegValue;

}


/**
 * @brief  Returns device version number.
 * @param  None.
 * @retval uint8_t Device version number.
 */
uint8_t SpiritGeneralGetDeviceVersionNumber(void)
{
  uint8_t tempRegValue;

  /* Reads the register value containing the device version number */
  g_xStatus = SpiritSpiReadRegisters(DEVICE_INFO0_VERSION, 1, &tempRegValue);

  return tempRegValue;

}


/**
 * @brief  Sets SPIRIT RF board version.
 * @param  SPIRIT RF board version.
 * @retval None.
 */
void SpiritGeneralSetSpiritVersion(SpiritVersion xSpiritVersion)
{
  s_xSpiritVersion = xSpiritVersion;

}

/**
 * @brief  Returns SPIRIT RF board version.
 * @param  None.
 * @retval SPIRIT RF board version.
 */
SpiritVersion SpiritGeneralGetSpiritVersion(void)
{
  return s_xSpiritVersion;

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
