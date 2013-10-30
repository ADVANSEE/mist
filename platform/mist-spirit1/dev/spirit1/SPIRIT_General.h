/**
 * @file    SPIRIT_General.h
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPIRIT_GENERAL_H
#define __SPIRIT_GENERAL_H


/* Includes ------------------------------------------------------------------*/

#include "SPIRIT_Regs.h"
#include "SPIRIT_Types.h"


#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @defgroup SPIRIT_General     General
 * @brief Configuration and management of SPIRIT General functionalities.
 * @details See the file <i>@ref SPIRIT_General.h</i> for more details.
 * @{
 */

/**
 * @defgroup General_Exported_Types     General Exported Types
 * @{
 */


/**
 * @brief  SPIRIT ModeExtRef enumeration
 */

typedef enum
{
  MODE_EXT_XO = 0,
  MODE_EXT_XIN = !MODE_EXT_XO
} ModeExtRef;

#define IS_MODE_EXT(MODE)   (MODE == MODE_EXT_XO || \
                             MODE == MODE_EXT_XIN)


/**
 * @brief  SPIRIT BatteryLevel enumeration
 */

typedef enum
{
  BLD_LVL_2_7_V = 0,
  BLD_LVL_2_5_V = 1,
  BLD_LVL_2_3_V = 2,
  BLD_LVL_2_1_V = 3
} BatteryLevel;

#define IS_BLD_LVL(MODE)  (MODE == BLD_LVL_2_7_V || \
                           MODE == BLD_LVL_2_5_V || \
                           MODE == BLD_LVL_2_3_V || \
                           MODE == BLD_LVL_2_1_V)


/**
 * @brief  SPIRIT GmConf enumeration
 */

typedef enum
{
  GM_SU_13_2 = 0,
  GM_SU_18_2,
  GM_SU_21_5,
  GM_SU_25_6,
  GM_SU_28_8,
  GM_SU_33_9,
  GM_SU_38_5,
  GM_SU_43_0
} GmConf;

#define IS_GM_CONF(MODE)    (MODE == GM_SU_13_2 || \
                             MODE == GM_SU_18_2 || \
                             MODE == GM_SU_21_5 || \
                             MODE == GM_SU_25_6 || \
                             MODE == GM_SU_28_8 || \
                             MODE == GM_SU_33_9 || \
                             MODE == GM_SU_38_5 || \
                             MODE == GM_SU_43_0)


/**
 * @brief  SPIRIT packet type enumeration
 */

typedef enum
{
  PKT_BASIC = 0x00,
  PKT_RFU,
  PKT_MBUS,
  PKT_STACK

} PacketType;

#define IS_PKT_TYPE(TYPE)    (TYPE == PKT_BASIC || \
                             TYPE == PKT_RFU| \
                             TYPE == PKT_MBUS || \
                             TYPE == PKT_STACK || \
                             )


/**
 * @brief  SPIRIT version type enumeration
 */

typedef enum
{
  SPIRIT_VERSION_2_0 = 0x00,
  SPIRIT_VERSION_2_1,
  SPIRIT_VERSION_3_0
} SpiritVersion;


/**
 * @}
 */


/**
 * @defgroup General_Exported_Constants         General Exported Constants
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup General_Exported_Macros            General Exported Macros
 * @{
 */
#define SpiritGeneralLibraryVersion() "Spirit1_Libraries_v.3.0.0"


/**
 * @}
 */


/**
 * @defgroup General_Exported_Functions         General Exported Functions
 * @{
 */


void SpiritGeneralBatteryLevel(SpiritFunctionalState xNewState);
void SpiritGeneralSetBatteryLevel(BatteryLevel xBatteryLevel);
BatteryLevel SpiritGeneralGetBatteryLevel(void);
void SpiritGeneralBrownOut(SpiritFunctionalState xNewState);
void SpiritGeneralHighPwr(SpiritFunctionalState xNewState);
void SpiritGeneralSetExtRef(ModeExtRef xExtMode);
ModeExtRef SpiritGeneralGetExtRef(void);
void SpiritGeneralSetXoGm(GmConf xGm);
GmConf SpiritGeneralGetXoGm(void);
PacketType SpiritGeneralGetPktType(void);

uint8_t SpiritGeneralGetDevicePartNumber(void);
uint8_t SpiritGeneralGetDeviceVersionNumber(void);

void SpiritGeneralSetSpiritVersion(SpiritVersion xSpiritVersion);
SpiritVersion SpiritGeneralGetSpiritVersion(void);

/**
 * @}
 */

/**
 * @}
 */


/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
